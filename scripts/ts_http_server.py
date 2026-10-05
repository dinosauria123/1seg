#!/usr/bin/env python3
"""stdin の MPEG-TS を HTTP で配信する最小サーバ。

使い方: ts_http_server.py [ポート]
  stdin  → HTTP クライアントへ MPEG-TS を塊で送る

設計上の 2 点（どちらも実測 2026-09-26 で判明した問題）:

1) 新規接続時に先頭 KEEP バイトを先頭に付ける。
   クライアントが接続した時点で既に放送の途中なので、そのまま流し始めると
   H.264 の SPS/PPS を見失い `non-existing PPS 0 referenced` / `no frame!`
   で復号できない。KEEP は PAT/PMT/SPS/PPS/IDR を含む開始点。

2) KEEP は新規接続時 ONLY。既存クライアントに繰り返し送ると、
   KEEP の末尾と現在チャンクの間で continuity counter が飛び、
   デコーダが以降すべて continuity error として破棄する
   （実測: 映像 PID 0x581 の CC 違反率 44%）。
"""
import http.server
import queue
import socketserver
import sys
import threading

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080

# 新しいクライアントに送り直す先頭バイト数。必ず 188 の倍数。
# 中途半端で切ると後続が全部ずれる。1seg の GOP は約 1〜2 秒（≒100〜200 パケット）
# なので 1400 パケット（263,200 B）あれば必ず SPS/PPS/IDR を含む。
KEEP_BYTES = 188 * 1400

# PSI の PID。CC は規格 (ISO/IEC 13818-1 2.4.3.4) により 0 固定。
#   PAT = 0x0000、PMT = 0x1FC8（program 10240）
# rust 側 psi.rs の PID_VIDEO / PID_PMT と対。
PSI_PIDS = {0x0000, 0x1FC8}

lock = threading.Lock()
clients: list = []          # 接続中の wfile
keep: bytes = b""           # 先頭の KEEP_BYTES
want_keep = True            # 新規接続検出フラグ

# pump → writer の受け渡しキュー。
#
# **pump は絶対に write してはいけない。** `self.wfile` は Python の
# BufferedWriter で、相手が読まないと `write()`/`flush()` がスレッドごと
# ブロックする。実測では pump が chunk#3 を書いてから `futex_do_wait` で
# 永久に停止し、サーバは `rchar` +50,000 B/s なのに `wchar` +0 B/s、
# 接続中の VLC は Position 0 のままだった（2026-09-26）。
# 読み取り（pump）と書き込み（writer）を別スレッドに分ければ、
# ブロックしても読み取りは止まらない。
_outq: "queue.Queue[bytes]" = queue.Queue(maxsize=64)


def fix_continuity(buf: bytearray, state: dict) -> None:
    """PID ごとに continuity counter を連番に詰め直す（in-place）。

    復調側で 1〜2 パケット落ちると、以降その PID の CC がずれたままになる。
    MPEG-TS 仕様では「CC が 1 欠けた時点で以降すべて continuity error」と
    みなすため、デコーダは以降のストリームを全部破棄する（実測: 映像 PID
    0x581 の CC 違反率 99.9%、VLC で「画面が壊れながら再生」）。

    落ちたデータの穴は戻せないので番号だけ詰め直す。Rust 側の
    `isdbt_dsp::ts::ContinuityTracker` と同じ処理。

    `state` はチャンクをまたいで PID ごとの状態を保持する（dict）。
    これを渡さないとチャンク境界で CC が 1 つずれる。
    """
    last = state
    i = 0
    n = len(buf)
    while i + 188 <= n:
        if buf[i] != 0x47:
            i += 1
            continue
        b1 = buf[i + 1]
        pid = ((b1 & 0x1F) << 8) | buf[i + 2]
        pusi = b1 & 0x40
        # NULL PID (0x1fff) は CC の追跡対象から除外する（実装依存のため）。
        # 追跡すると他の PID の計算を汚染する（実測: 映像 PID の CC が全部ずれる）。
        if pid == 0x1FFF:
            i += 188
            continue
        if (b1 >> 7) & 1:          # transport_error_indicator は CC を消費しない
            i += 188
            continue
        # adaptation_field_control: 0b10/0b11 は先頭が adaptation field で
        # その場合のみ payload がある。0b01/0b11 は payload あり → CC +1。
        afc = (buf[i + 3] >> 4) & 0x03
        has_payload = afc in (0b01, 0b11)
        cc = buf[i + 3] & 0x0F
        prev = last.get(pid)
        # CC は素通しする。
        #
        # Rust 側の psi.rs が注入ごとに CC をインクリメントしている
        # （PAT/PMT とも pusi=1）。ここで 0 に固定すると同一 PID に
        # CC=0 が 2 回来て libdvbpsi が `TS duplicate (received 0, expected 1)`
        # を 40 回以上出し続けて止まる（実測 2026-09-26: Position 0）。
        # 逆に Rust 側を 0 固定にすると `TS discontinuity` で同じ結果。
        # よって **CC の唯一の情報源は Rust 側の注入順** とし、
        # Python では上書きしない（holeless passthrough）。
        if pid in PSI_PIDS:
            # PSI (PAT/PMT) は Rust 側が規格どおり CC=0 固定で出力する。
            # ここで再計算すると Rust 0 vs Python 期待 N で必ずずれる
            # （実測: discontinuity / duplicate の両方を観測）。
            last[pid] = (0, pusi)
            i += 188
            continue
        if prev is not None and has_payload:
            pcc, _ = prev
            expect = (pcc + 1) & 0x0F
            if cc != expect:
                buf[i + 3] = (buf[i + 3] & 0xF0) | expect
                cc = expect
        last[pid] = (cc, pusi)
        i += 188


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        global want_keep
        self.send_response(200)
        self.send_header("Content-Type", "video/mp2t")
        self.send_header("Cache-Control", "no-cache")
        # 無限ストリームなので Content-Length は付けない。代わりに chunked
        # transfer-encoding を使う: 長さ不明だが「まだ続く」ことを HTTP/1.1 で
        # 正しく表現する。keep-alive 単独では VLC がボディの終わりを判断できず
        # 停止した（実測 2026-09-26: 6.6 MB 送受信・ACK 済みでも CPU 0%、0:00）。
        self.send_header("Transfer-Encoding", "chunked")
        self.end_headers()
        with lock:
            # **`self.wfile` は使わず raw ソケットへ直接書く。**
            #
            # BaseHTTPRequestHandler.wfile は BufferedWriter で、
            # `write()` + `flush()` は送信バッファが埋まるか相手が読まないと
            # スレッドごとブロックする。実測では writer スレッドが
            # 「first send ok (13,912 B)」の直後に `futex_do_wait` で
            # 永久に停止し、wchar +0 B/s、VLC は Position 0 のままだった
            # （2026-09-26）。
            #
            # raw ソケット + send timeout なら、詰んでも例外で抜けて
            # 次のチャンクを処理できる。
            self.connection.settimeout(5.0)
            self.raw_sock = self.connection
            clients.append(self.connection)
            # この新規クライアントに 1 回だけ keep を送らせる。
            want_keep = True
        try:
            while not self.connection.fileno() == -1:
                threading.Event().wait(0.5)
        except Exception:
            pass
        finally:
            with lock:
                if self.connection in clients:
                    clients.remove(self.connection)
                # クライアントが居なくなったら、次の接続で keep を再送する。
                if not clients:
                    want_keep = True


def _enqueue(payload: bytes) -> None:
    """writer スレッドへ 1 チャンクを渡す。キューが満杯なら古い分を捨てる。"""
    try:
        _outq.put_nowait(payload)
    except queue.Full:
        # クライアントが読めていない。最新が優先なので 1 つ取りしてから入れる。
        try:
            _outq.get_nowait()
        except queue.Empty:
            pass
        try:
            _outq.put_nowait(payload)
        except queue.Full:
            pass


def writer() -> None:
    """キューから chunk framing を付けソケットへ書く唯一のスレッド。

    `self.wfile` は BufferedWriter なので、**このスレッドがブロックしても
    pump（読み取り）は止まらない**。これが 0:00 問題のsolutions。
    """
    reported = False
    while True:
        try:
            payload = _outq.get(timeout=0.5)
        except queue.Empty:
            continue
        with lock:
            targets = list(clients)
        dead = []
        for w in targets:
            try:
                # Transfer-Encoding: chunked なので chunk 形式で書く。
                # 終端の 0-chunk は接続を閉じたときだけ返す。通常は流し続ける。
                # bytes に対する % 演算は無いので hex は自前で整形する。
                w.sendall(f"{len(payload):x}\r\n".encode() + payload + b"\r\n")
            except Exception as e:
                # 黙って捨てると「クライアントは居るのに永久に 0:00」になる。
                # raw ソケット + settimeout(5.0) なので、詰んでも 5 秒で例外に
                # なりここで抜ける（ブロックしたままULATIONしない）。
                print(f"client write failed: {e!r}", file=sys.stderr, flush=True)
                dead.append(w)
        if dead:
            with lock:
                for w in dead:
                    if w in clients:
                        clients.remove(w)
        if targets and not reported:
            reported = True
            print(f"writer: first send ok ({len(payload)} B)", file=sys.stderr, flush=True)


def pump():
    """stdin から読んで、全クライアントへ送る。"""
    global keep, want_keep
    # チャンクをまたいで PID ごとの CC 状態を保持する。rust 側の
    # ContinuityTracker と同じ役目。これが無いと chunk 境界で CC がずれる。
    cc_state = {}
    # FIFO を non-blocking にして、要求サイズ分が揃うまで待たないようにする。
    # ブロック読みの状態で `anon_pipe_read` に張り付き.Send-Q=0、VLC は 0:00 のまま
    # だった（実測 2026-09-26）。
    import os as _os
    import fcntl as _fcntl
    _raw = _os.fdopen(_os.dup(sys.stdin.fileno()), "rb", 0)
    _fcntl.fcntl(_raw.fileno(), _fcntl.F_SETFL, _os.O_NONBLOCK)
    _carry = bytearray()
    while True:
        try:
            data = _raw.read(188 * 100)
        except (BlockingIOError, InterruptedError):
            data = None
        if not data:
            # 読める分がなければ（non-blocking の空読み or EOF）少し待って再試行。
            threading.Event().wait(0.02)
            continue
        # TS パケット境界（188 の倍数）を守る。read が 188 で割り切れない
        # 長さで返すと、それ以降の全パケットが 1 バイトずつずれて
        # 0x47 が合わなくなり、デコードが全滅する。
        _carry += data
        n = (len(_carry) // 188) * 188
        if n == 0:
            continue
        chunk = bytes(_carry[:n])
        del _carry[:n]
        if len(keep) < KEEP_BYTES:
            keep = (keep + chunk)[:KEEP_BYTES]
        payload = None
        with lock:
            if want_keep and clients:
                # KEEP は「その時点の過去 TS」なので、そのまま混ぜると現在の
                # chunk と continuity counter が矛盾する（実測: 映像 PID の
                # CC 違反率 99.9%）。連結した列の CC を正規化してから送る。
                # 新規接続なので CC 状態はリセットする。
                joined = bytearray(keep + chunk)
                fix_continuity(joined, {})
                payload = bytes(joined)
                want_keep = False
            else:
                # 毎チャンク CC を詰め直す（復調の穴を吸収する）。
                buf = bytearray(chunk)
                fix_continuity(buf, cc_state)
                payload = bytes(buf)
            targets = list(clients)
        # socket への write はロックの外側で行う。ロックの中でブロックすると
        # クライアント追加も他の送信も全部止まる（実測: 1 人の遅いクライアントで
        # 全体が停止し、VLC は接続したまま 0:00）。
        #
        # **pump スレッドをブロックさせない。** `self.wfile` は Python の
        # BufferedWriter（既定 8192 B）で、`write()` + `flush()` は
        # 相手が読まないとスレッドごと停止する。実測では chunk#3 を書いてから
        # スレッドが `futex_do_wait` で永久に止まり、wchar +0 B/s、
        # クライアントは接続したまま Position 0 のままだった（2026-09-26）。
        #
        # 対策は 2 つ:
        #   (a) ソケット send buffer を-buffering しないdux に send timeout
        #   (b) 送信を専用スレッドに退避して pump は read だけ続ける
        # ここでは (a)+(b) の軽い方として、**書き込みは専用 writer スレッド**に
        # 渡し、pump は決して write でブロックしない。
        _enqueue(bytes(payload))


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


if __name__ == "__main__":
    def _pump_guarded():
        """pump() が例外で死んでも 1 プロセス全体が生き続けるようにする。

        thread は素の例外で静かに死ぬ。pump が止まるとサーバは
        200 OK を返し続けるのに 1 バイトも送らず、VLC は
        「ずっと再生待ち」で永久に止まる（実測 2026-09-26）。
        """
        try:
            pump()
        except BaseException:
            import traceback
            print("pump died:", file=sys.stderr, flush=True)
            traceback.print_exc()

    threading.Thread(target=_pump_guarded, daemon=True).start()
    threading.Thread(target=writer, daemon=True).start()
    with Server(("127.0.0.1", PORT), Handler) as httpd:
        print(f"TS HTTP server on http://127.0.0.1:{PORT}/stream.ts", file=sys.stderr)
        httpd.serve_forever()
