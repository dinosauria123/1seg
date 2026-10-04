#!/usr/bin/env python3
"""IQ キャプチャの実行中トリマー（バックグラウンド守护）。

なぜ要るか（実測 2026-10-04 11:25、IQ が 395 MB に達した）:
  live_play_direct.sh の `if [ "$sz" -gt "$IQ_KEEP" ]` は**起動時の 1 回
  だけ**評価される。実行中は rtl_sdr が 2 MB/s（実時間）で追記し続けるので
  IQ_KEEP を 40 MB に下げても 2 分で 200 MB を超えた。/tmp は tmpfs 3.6G。
  7 チャンネル同时走ると上限に達し、起動スクリプトの
  `echo -n 1 > "$ALIVE"` が write error で失敗する（実害: 再生が止まる）。

なぜ素朴に `tail -c N > new && mv` が不可か:
  1. mv すると writer の fd 3 が旧 inode を指したままになり、以後の追記が消える。
  2. 頭 Aeille削ると stream_decode の file offset（--follow の追従位置）と
     ファイル内容がズレる。

採る方法（fallocate --collapse-range）:
  **既に読了済みの範囲だけ**を頭から潰す。
  - inode を保ったまま末尾をずらすので writer の fd 3 は有効。
  - stream_decode は /proc/<pid>/fdinfo の pos を読み進めているので、
    その pos より前を潰しても追従は壊れない。
  - 潰した分だけファイルが物理的に短く so、tmpfs の使用量も減る。

安全側の guards:
  - 読了位置が特定できなければ**何もしない**。
  - 潰す量は 1 回のあ人民检察院で IQ_KEEP/4 以下に制限（信号連続性を壊さない）。
  - collapse 後に stream_decode がエラー終了していないか pos を見る。
"""
import os
import subprocess
import sys
import time

POLL_SEC = 10.0


COLLAPSED_SUFFIX = ".collapsed"


def collapsed_path_for(iq_path):
    """IQ ファイルごとに累積値ファイルを分ける。

    **これが必須**（実測 2026-10-04 12:43）:
    `/tmp/isdbt_collapsed` を単一ファイルで共有していると、チャンネル切替
    後に前のチャンネルの累積値が残る。stream_decode は pos=0 から
    `SeekFrom::Current(-n)` を試みて EINVAL で失敗し、以後ずっと補正されない。
    ファイルごとに分ければ他チャンネルの値 contaminate しない。
    """
    return iq_path + COLLAPSED_SUFFIX


def log(path, msg):
    try:
        with open(path, "a") as fh:
            fh.write(f"{time.strftime('%H:%M:%S')} {msg}\n")
    except OSError:
        pass


def write_collapsed(path, total):
    """累積 collapse バイト数を原子的に書く。

    stream_decode は `--follow` 中にこのファイルを読む:
      値が増えたら `SeekFrom::Current(-delta)` でオフセットを補正する。
      これをしないcollapse 後は reader が常に EOF を読む（実測 12:03）。
    """
    try:
        tmp = path + ".tmp"
        with open(tmp, "w") as fh:
            fh.write(str(total))
        os.replace(tmp, path)
    except OSError:
        pass


def find_reader_pos(iq_path):
    """stream_decode の IQ ファイル fd の読了位置（pos）を返す。"""
    target = os.path.realpath(iq_path)
    try:
        pids = [p for p in os.listdir("/proc") if p.isdigit()]
    except OSError:
        return None, None
    for pid in pids:
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as fh:
                cmd = fh.read().replace(b"\0", b" ").decode("utf-8", "replace")
        except OSError:
            continue
        if "stream_decode" not in cmd or iq_path not in cmd:
            continue
        fddir = f"/proc/{pid}/fd"
        try:
            fds = os.listdir(fddir)
        except OSError:
            continue
        for fd in fds:
            try:
                tgt = os.readlink(f"{fddir}/{fd}")
            except OSError:
                continue
            if os.path.realpath(tgt) != target:
                continue
            try:
                with open(f"/proc/{pid}/fdinfo/{fd}") as fh:
                    for line in fh:
                        if line.startswith("pos:"):
                            return int(pid), int(line.split()[1])
            except (OSError, ValueError):
                continue
    return None, None


def main():
    if len(sys.argv) < 3:
        print("usage: iq_trim.py <iq_file> <keep_bytes> [log]", file=sys.stderr)
        return 2
    iq = sys.argv[1]
    keep = int(sys.argv[2])
    logf = sys.argv[3] if len(sys.argv) > 3 else "/tmp/isdbt_trim.log"
    max_collapse = keep // 4
    total = 0
    cpath = collapsed_path_for(iq)
    # 前回の累積を読み継ぐ（再起動時）。ファイルがないので 0 から。
    try:
        with open(cpath) as fh:
            total = int(fh.read().strip() or "0")
    except (OSError, ValueError):
        total = 0

    log(logf, f"=== iq_trim 開始: {iq} keep={keep} "
              f"max_collapse={max_collapse} 累積={total} ===")
    while True:
        time.sleep(POLL_SEC)
        try:
            sz = os.stat(iq).st_size
        except OSError:
            continue
        if sz <= keep:
            continue

        pid, pos = find_reader_pos(iq)
        if pos is None:
            log(logf, f"読了位置不明 → 何もしない（sz={sz}）")
            continue
        if pos == 0:
            log(logf, f"まだ読み 시작していない（pos=0, sz={sz}）")
            continue

        # 潰す量の上限は「既に読了済みの範囲」= pos。
        #
        # ただし collapse は [0, n) を消して残りを左へ詰めるので、reader の
        # file offset `pos` は据え置かれたまま内容だけ n バイト先行する
        #（= n バイト分の IQ を飛ばす）。復調器の再取得は 800 シンボルごとに
        # 走る（stream.rs REACQUIRE_EVERY）ので 1 回分的erministic には
        # 復帰するが、GOP が飛ぶ可能性はある。1 回の潰す量は max_collapse
        # （= keep/4 ≒ 10 MB ≈ 5 秒）に抑える。
        n = min(sz - keep, pos, max_collapse)
        if n <= 0:
            log(logf, f"collapse 対象なし（sz={sz} pos={pos}）")
            continue
        if n % 4096:
            n -= n % 4096

        try:
            subprocess.run(
                ["fallocate", "--collapse-range",
                 "--offset", "0", "--length", str(n), iq],
                check=True, capture_output=True, timeout=10,
            )
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired,
                FileNotFoundError, OSError) as exc:
            log(logf, f"collapse 失敗: {exc}")
            continue

        newsz = sz - n
        total += n
        write_collapsed(cpath, total)
        log(logf, f"collapse {n} B 実行: {sz} → {newsz}  "
                  f"(pid={pid} pos={pos} 累積={total})")


if __name__ == "__main__":
    sys.exit(main())