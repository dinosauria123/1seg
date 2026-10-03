#!/usr/bin/env python3
"""復調ロックの到達を非同期で記録する。

ロック到達までに 60〜90 秒かかり、`live_play_direct.sh` はその場で wait するため、
起動した側は戻りを待たずに「ロックした」と知ることができない。
このスクリプトは /tmp/isdbt_live.log を逐次読みし、ロック関連の
マイルストーンだけを時刻付きで /tmp/isdbt_lock.log に追記し続ける。

使い方:
  nohup python3 scripts/lock_watch.py > /dev/null 2>&1 &
  tail -f /tmp/isdbt_lock.log
停止:
  ~/oneseg-rs/scripts/stop_live.sh   （PID ファイル経由で停止する）
"""
import os
import re
import time

LOG = "/tmp/isdbt_live.log"
PLOG = "/tmp/isdbt_ffplay.log"
OUT = "/tmp/isdbt_lock.log"

# ロック関連のマイルストーンだけ残す。「再取得」は毎フレーム出る progress で、
# 通過させるとログが progress で埋まって何も読めなくなる。
MILESTONE = re.compile(
    r"bin offset=|lock_params|ロック|SPS/PPS/IDR|TS 出力|終了:|真のロック"
)
STATUS = re.compile(
    r"(\d+\.\d\d) A-V: *([+-]?[\d.]+).*?vq=\s*(\S+).*?aq=\s*(\S+)"
)
ERR = re.compile(r"error while decoding|corrupt \(stream|out of range")


def append(line):
    with open(OUT, "a") as fh:
        fh.write(f"[{time.strftime('%H:%M:%S')}] {line}\n")
        fh.flush()


def tail_new(path, pos):
    """新規バイトだけ返す。ファイルが truncate されても壊れない。"""
    try:
        sz = os.path.getsize(path)
    except OSError:
        return b"", pos
    if sz < pos:          # stop_live.sh が : > で truncate した
        pos = 0
    if sz == pos:
        return b"", pos
    try:
        with open(path, "rb") as fh:
            fh.seek(pos)
            return fh.read(sz - pos), sz
    except OSError:
        return b"", pos


def main():
    append("=== lock_watch 開始 ===")
    log_pos = 0
    plog_pos = 0
    last_status = 0.0
    err_window = 0
    err_total = 0
    while True:
        # --- 復調ログ: マイルストーンのみ ---
        data, log_pos = tail_new(LOG, log_pos)
        for raw in data.split(b"\n"):
            line = raw.decode("utf-8", "replace").strip()
            if line and MILESTONE.search(line):
                # bin offset の長い候補リストは要点だけ残す
                append(re.sub(r" 候補=\[.*?\]", "", line))

        # --- ffplay ログ: 10 秒ごとに再生状態とエラー増分 ---
        now = time.time()
        if now - last_status >= 10:
            last_status = now
            pdata, plog_pos = tail_new(PLOG, plog_pos)
            txt = pdata.decode("utf-8", "replace")
            m = STATUS.search(txt)
            if m:
                append(f"ffplay {m.group(1)}s A-V={m.group(2)} "
                       f"vq={m.group(3)} aq={m.group(4)}")
            # この 10 秒窓で増えたエラー数。累積値と区別できるよう分けて出す。
            err_window = len(ERR.findall(txt))
            err_total += err_window
            if err_window:
                append(f"  デコードエラー +{err_window} 件 "
                       f"(累積 {err_total})")
        time.sleep(0.5)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass