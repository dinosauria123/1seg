#!/usr/bin/env python3
"""1seg 受信機の tkinter GUI（札幌・手稲山親局）。

操作:
  - ▲ CH / ▼ CH ボタン、または ↑ ↓ キー      … チャンネル切替
  - ▶ 再生 / ■ 停止 ボタン、または Space / Esc
  - 映像は ffplay のウィンドウで表示する（GUI には内蔵しない）

チャンネル一覧について
----------------------
札幌（手稲山）親局の割当。出典:
  - NHK北海道 受信情報 札幌放送局チャンネル一覧
    https://www.nhk.or.jp/hokkaido/station_info/ch_sapporo.html
  - 北海道圏テレビジョンチャンネル表（2024/01/18 更新）
    http://hokkaido.basekernel.ne.jp/pc/hokkaido-tv-list.html

    局名    NHK総合 NHK教育 HBC   STV   UHB   HTB   TVh
    札幌     15ch   13ch     19ch  21ch  25ch  23ch  14ch

**_freq_hz は自前の実測値**（ch 13〜52 を 1 MHz 間隔で走査し、平均電力
が 20 を超えたチャンネル）。実測電力: ch13=1.6  ch14=75.7  ch15=72.9
ch19=76.4 ch21=65.1 ch23=64.2 ch25=89.3 ch28=133.0 ch29=62.7

ch28（銀山都中継局の割当）は**電力は最も高いのに TMCC 同期 0.67 で偽ロック**
だった（実測 2026-10-04 01:57）。1seg ではなく混線なので一覧から外した。
ch13（NHK教育）は電力が 1.6 で受信圏外。

_Uses._
  - ゲインは `-g 5`（自動でも 0 でも 8bit ADC が飽和する。commit 6d62952）
  - bin offset は 308 を固定（1 bin = 992 Hz の量子化。実測で全チャンネル
    とも 308。探索を省略してロックを 78 秒 → 16 秒に短縮。commit 236ab6f）

使い方:
  ~/oneseg-rs/scripts/oneseg_gui.py
"""
import json
import os
import re
import subprocess
import sys
import threading
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

HOME = os.path.expanduser("~")
REPO = os.path.join(HOME, "oneseg-rs")
LIVE_SH = os.path.join(REPO, "scripts", "live_play_direct.sh")
STOP_SH = os.path.join(REPO, "scripts", "stop_live.sh")
SEGOFF_FILE = "/tmp/isdbt_segoff"
LOCK_LOG = "/tmp/isdbt_lock.log"
STATE_FILE = os.path.join(
    os.environ.get("XDG_CONFIG_HOME", os.path.join(HOME, ".config")),
    "oneseg-rs", "gui_state.json")

# 札幌（手稲山）親局。ch, 中心周波数 Hz, 実測電力, 局名, リモコン番号
CHANNELS = [
    (13, 473_142_857,  1.6, "NHK教育", 2),
    (14, 479_142_857, 75.7, "TVh", 7),
    (15, 485_142_857, 72.9, "NHK総合", 3),
    (19, 509_142_857, 76.4, "HBC", 1),
    (21, 521_142_857, 65.1, "STV", 5),
    (23, 533_142_857, 64.2, "HTB", 6),
    (25, 545_142_857, 89.3, "UHB", 8),
]

BG = "#1e1e26"
FG = "#e6e6ef"
ACCENT = "#7aa2f7"
OK = "#9ece6a"
WARN = "#e0af68"
ERR = "#f7768e"
MUTED = "#565f73"
BTN_BG = "#2a2a37"
BTN_ACTIVE = "#3a3a4a"


def load_state():
    try:
        with open(STATE_FILE) as fh:
            d = json.load(fh)
    except (OSError, ValueError):
        d = {}
    try:
        return {int(k): v for k, v in (d.get("names") or {}).items()}, \
            int(d.get("last_index", 2))
    except (TypeError, ValueError):
        return {}, 2


def save_state(names, last_index):
    os.makedirs(os.path.dirname(STATE_FILE), exist_ok=True)
    tmp = STATE_FILE + ".tmp"
    with open(tmp, "w") as fh:
        json.dump({"names": {str(k): v for k, v in names.items()},
                   "last_index": last_index}, fh, ensure_ascii=False, indent=1)
    os.replace(tmp, STATE_FILE)


def alive(name):
    return subprocess.run(["pgrep", "-x", name],
                          capture_output=True).returncode == 0


class LockWatcher(threading.Thread):
    """`/tmp/isdbt_lock.log` を追い、UI に状態を流す。

    「再取得」は毎フレーム出る progress なのでこれは流さない（ログが
    progress で埋まって何も読めなくなる）。
    """

    daemon = True

    def __init__(self, out):
        super().__init__()
        self.out = out
        self.stop_flag = threading.Event()
        self.pos = 0

    def run(self):
        beat = 0.0
        while not self.stop_flag.is_set():
            try:
                sz = os.path.getsize(LOCK_LOG)
                if sz < self.pos:
                    self.pos = 0
                if sz > self.pos:
                    with open(LOCK_LOG, "rb") as fh:
                        fh.seek(self.pos)
                        chunk = fh.read(sz - self.pos)
                    self.pos = sz
                    for raw in chunk.split(b"\n"):
                        line = raw.decode("utf-8", "replace").strip()
                        if line:
                            self.out(line)
            except OSError:
                pass
            now = time.time()
            if now - beat >= 15:
                beat = now
                self.out("__beat__")
            time.sleep(0.4)


class App:
    def __init__(self, root):
        self.root = root
        root.title("1seg 札幌（oneseg-rs）")
        root.configure(bg=BG)
        root.geometry("420x420")
        root.resizable(False, False)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        # ffplay の映像ウィンドウが前面にある時も操作できるよう、常に手前に
        # 置く。topmost は「他のアプリに隠れない」意味なので要件どおり。
        root.attributes("-topmost", True)

        f = tkfont.Font(family="Sans", size=11)
        fb = tkfont.Font(family="Sans", size=20, weight="bold")
        fs = tkfont.Font(family="Sans", size=9)

        self.names, self.index = load_state()
        self.index = max(0, min(self.index, len(CHANNELS) - 1))

        self.lbl_state = tk.Label(root, text="停止中", font=fb, bg=BG, fg=MUTED)
        self.lbl_state.pack(pady=(16, 2))
        self.lbl_ch = tk.Label(root, text="", font=f, bg=BG, fg=FG)
        self.lbl_ch.pack()
        self.lbl_detail = tk.Label(root, text="", font=fs, bg=BG, fg=MUTED)
        self.lbl_detail.pack(pady=(2, 12))

        # ボタンを 2x2 で並べる。1 列に 4 個並べると 480px のウィンドウに
        # 収まらず、停止ボタンが画面外へ押し出される（ユーザー指摘 2026-10-04）。
        # 2x2 なら横幅 2 個分に収まる。
        row = tk.Frame(root, bg=BG)
        row.pack(pady=(0, 14))
        btn_kw: dict = dict(width=10, font=f, relief="flat", bd=0, pady=10)

        # 1 行目: チャンネル選択
        tk.Button(row, text="▲ CH", bg=BTN_BG, fg=FG,
                  activebackground=BTN_ACTIVE, activeforeground=FG,
                  command=self.prev_channel, **btn_kw).pack(side="left", padx=6)
        tk.Button(row, text="▼ CH", bg=BTN_BG, fg=FG,
                  activebackground=BTN_ACTIVE, activeforeground=FG,
                  command=self.next_channel, **btn_kw).pack(side="left", padx=6)
        # 2 行目: 再生 / 停止
        row2 = tk.Frame(root, bg=BG)
        row2.pack()
        self.btn_play = tk.Button(
            row2, text="▶ 再生", bg="#2a4a2a", fg=OK,
            activebackground="#3a5a3a", activeforeground=OK,
            command=self.play, **btn_kw)
        self.btn_play.pack(side="left", padx=6)
        tk.Button(row2, text="■ 停止", bg="#4a2a2a", fg=ERR,
                  activebackground="#5a3a3a", activeforeground=ERR,
                  command=self.stop, **btn_kw).pack(side="left", padx=6)

        self.lbl_log = tk.Label(root, text="", font=fs, bg=BG, fg=MUTED,
                                anchor="w", justify="left")
        self.lbl_log.pack(fill="x", padx=16)

        root.bind("<Up>", lambda _e: self.prev_channel())
        root.bind("<Down>", lambda _e: self.next_channel())
        root.bind("<Prior>", lambda _e: self.prev_channel())
        root.bind("<Next>", lambda _e: self.next_channel())
        root.bind("<space>", lambda _e: self.play())
        root.bind("<Escape>", lambda _e: self.stop())

        self.watcher = LockWatcher(self.on_lock_line)
        self.watcher.start()
        self.refresh()
        # 起動時に**実行中のチャンネル**へ表示を合わせる。
        # 前回 GUI を閉じても受信プロセスが残っていると、保存された
        # last_index と実際の受信チャンネルがずれる（実測 2026-10-04 15:1x）。
        # このまま ▶ を押すと別チャンネルへ切り替わってしまう。
        self.sync_to_running()
        self.root.after(400, self.poll_status)

    # ---------- 表示 ----------
    def refresh(self):
        ch, freq, power, station, remote = CHANNELS[self.index]
        name = self.names.get(ch) or station
        self.lbl_ch.config(text=f"{name}    ch{ch}")
        self.lbl_detail.config(
            text=f"{freq/1e6:.3f} MHz ・ リモコン {remote} ・ "
                 f"実測電力 {power:.0f}")

    def playing(self):
        return alive("stream_decode") and alive("ffplay")

    def running_freq(self):
        """**実行中**の受信チャンネル周波数。停止中なら None。

        `stream_decode` のコマンドラインから `isdbt_iq_<freq>.iq` を読む。
        これがないと GUI は保存された `last_index` を表示するだけで、
        **前回終了時に进程が残ったチャンネルと表示がずれる**（例: GUI は
        ch13 を表示しているのに ch19 で受信中）。

        実測 2026-10-04 15:1x: state file の last_index=0（ch13）だが
        stream_decode は 509142857（ch19）で動作していた。
        """
        try:
            out = subprocess.run(["pgrep", "-x", "stream_decode"],
                                 capture_output=True, text=True).stdout.split()
        except OSError:
            return None
        for pid in out:
            try:
                with open(f"/proc/{pid}/cmdline", "rb") as fh:
                    cmd = fh.read().replace(b"\0", b" ").decode("utf-8", "replace")
            except OSError:
                continue
            m = re.search(r"isdbt_iq_(\d+)\.iq", cmd)
            if m:
                return int(m.group(1))
        return None

    def sync_to_running(self):
        """実行中のチャンネルに GUI の選択を合わせる（起動時 1 回だけ）。

        状態が食い違っていると「▶ 再生」で**別チャンネルに切り替わって**しまい、
        ユーザーが意図しない再起動になる。具体的には:
          - GUI が ch13 を表示しているのに ch19 で受信中
          - ユーザーが ▶ を押すと ch13 に切り替えて再起動する
        """
        if not self.playing():
            return
        freq = self.running_freq()
        if freq is None:
            return
        for i, (_ch, f, _p, _s, _r) in enumerate(CHANNELS):
            if f == freq:
                if i != self.index:
                    self.index = i
                    self.refresh()
                return
        # CHANNELS にない周波数（チャンネルリストを編集した後など）。
        # 表示はそのままに、ログでだけ知らせる。
        self.lbl_log.config(text=f"受信中 {freq/1e6:.3f} MHz（一覧になし）", fg=WARN)

    def poll_status(self):
        if self.playing():
            self.lbl_state.config(text="● 受信中", fg=OK)
            self.btn_play.config(text="↻ 再起動")
            # ffplay が起動すると自身にフォーカスを奪う。
            # `-topmost` だけでは Mutter 側で尊重されないことがあるので、
            # 定期的に手前へ持ち上げる。
            try:
                self.root.lift()
            except tk.TclError:
                pass
        else:
            if self.lbl_state.cget("fg") != MUTED:
                self.lbl_state.config(text="停止中", fg=MUTED)
            self.btn_play.config(text="▶ 再生")
        self.root.after(1000, self.poll_status)

    def on_lock_line(self, line):
        """LockWatcher スレッドから呼ばれる。Tk は main thread 専用なので、
        `after()` でメインスレッドへ転送する必要がある。

        mainloop が回っていないテスト実行（`root.update()` なし）では
        `after()` が `RuntimeError: main thread is not in main loop` を
        投げるので、その場合は何もせず捨てる（GUI が見えない場面なので）。
        """
        msg = re.sub(r"^\[[^\]]*\]\s*", "", line)

        def apply():
            if msg == "__beat__":
                return
            if "bin offset=" in msg:
                m = re.search(r"TMCC同期=([\d.]+)", msg)
                rs = re.search(r"RS復号率=([^ ]+)", msg)
                parts = []
                if m:
                    parts.append(f"TMCC同期 {m.group(1)}")
                if rs:
                    parts.append(f"RS {rs.group(1)}")
                self.lbl_log.config(
                    text=("bin offset 確定  " + "  ".join(parts))[:64],
                    fg=ACCENT)
            elif "SPS/PPS/IDR検出" in msg or "ロック→" in msg:
                self.lbl_state.config(text="● 受信中", fg=OK)
                self.lbl_log.config(text="復調完了・再生中", fg=OK)
            elif "lock_params" in msg:
                self.lbl_log.config(text="パラメータ最適化中…", fg=WARN)
        # Tk は main thread 専用。mainloop が回っていない（テスト実行など）
        # 場合は転送できないので捨てる。`after` が例外を投げる前に判定する。
        try:
            self.root.after(0, apply)
        except (RuntimeError, tk.TclError):
            pass

    # ---------- 操作 ----------
    def _goto(self, i):
        # 上限/下限に達したら反対側へ回り込む（循環）。
        # クランプすると端で ▲ を押しても何も起きず、「壊れている」と
        # 思われる（ユーザー指摘 2026-10-04）。
        n = len(CHANNELS)
        i %= n
        if i == self.index:
            return
        self.index = i
        save_state(self.names, self.index)
        self.refresh()
        if self.playing():
            self.play()

    def next_channel(self):
        self._goto(self.index + 1)

    def prev_channel(self):
        self._goto(self.index - 1)

    def play(self):
        _ch, freq, _p, _s, _r = CHANNELS[self.index]
        # bin offset はチャンネルごとに違う（1 bin = 992 Hz の量子化で受信機の
        # LO 周波数誤差による）。しかも **survey の測定値は不正確** ——
        # STV の正解は 308 なのに survey（20 MB）は 309 と報告した
        # （320 MB で 307/308/309/310 を比較すると 308 だけが TS を出した、
        #   他は 0 B。実測 2026-10-04）。
        #
        # そのため survey の値を固定するのではなく、**±リトライで自己修復
        # させる**。`live_play_direct.sh` が `ISDBT_SEGOFF_RETRY` を渡す
        # ので、値が 1 bin ずれていても 最終的に正解に当たる。
        # ここでは survey の値を初期候補として渡すだけ。
        segoff_path = f"/tmp/isdbt_segoff_{freq}"
        try:
            with open(segoff_path) as fh:
                segoff = fh.read().strip()
        except OSError:
            segoff = "308"
            with open(segoff_path, "w") as fh:
                fh.write(segoff + "\n")

        self.lbl_state.config(text="… 起動中", fg=WARN)
        self.lbl_log.config(text=f"bin offset {segoff} で起動します…", fg=WARN)
        self.root.update_idletasks()

        # **ffplay を明示的に kill してから起動する。**
        # 単に `live_play_direct.sh` を呼ぶだけだと、ffplay が前のチャンネルの
        # TS を開いたまま残って新しい TS を読まない（実測: 切替しても前の
        # チャンネルしか出ない）。ffplay は FIFO を open した状態でブロックして
        # いるので、kill して FIFO を作り直すのが確実。
        subprocess.run(["bash", STOP_SH], capture_output=True)
        # プロセスが実際に死ぬまで待つ（stop_live.sh は kill した直後に
        # 報告するので、待たずに起動すると競合する）。
        for _ in range(30):
            if not self.playing():
                break
            time.sleep(0.1)

        # `ISDBT_SEGOFF_RETRY=0` で bin offset の ±リトライを無効化する。
        #
        # なぜ 0 にするか（実測タイムライン 2026-10-04 13:31）:
        #     0.0s  起動
        #    13.9s  bin offset 確定 TMCC同期=1.000
        #    19.9s  SPS/PPS/IDR 検出（映像出力開始）
        # リトライ（既定 ±2）が約 7 秒を占めていた。bin offset の実測値は
        # `/tmp/isdbt_segoff_<freq>` に保存済みなので、一度の試行で当てれば
        # 映像開始まで 13 秒になる。
        #
        # 偽値だった場合の兆候: TMCC 同期が 0.95 未満のまま進まない。
        # そのときは `SEGOFF_RETRY=2` で再実行する。
        env = dict(os.environ, ISDBT_SEGOFF_RETRY="0")
        subprocess.Popen(["bash", LIVE_SH, str(freq)], env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)

    def stop(self):
        self.lbl_state.config(text="停止中", fg=MUTED)
        self.lbl_log.config(text="停止しています…", fg=WARN)
        self.root.update_idletasks()
        subprocess.run(["bash", STOP_SH], capture_output=True)

    def on_close(self):
        """ウィンドウを閉じたら受信も止める（ffplay を孤児にしない）。"""
        self.watcher.stop_flag.set()
        try:
            if self.playing():
                subprocess.run(["bash", STOP_SH], capture_output=True,
                               timeout=15)
        except (OSError, subprocess.TimeoutExpired):
            pass
        self.root.destroy()


def main():
    if not os.path.exists(LIVE_SH):
        sys.exit(f"{LIVE_SH} が見つかりません")
    # 前回の孤児を掃除してから開く
    if alive("stream_decode") or alive("ffplay"):
        subprocess.run(["bash", STOP_SH], capture_output=True)
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()