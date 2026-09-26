#!/usr/bin/env bash
# ライブ 1seg 視聴（ffplay）。バックグラウンド起動でも stdin に依存しない。
#
# 使い方: ./scripts/live_ffplay.sh [周波数Hz]
# 停止:   pkill -x ffplay; pkill -x stream_decode; pkill -x rtl_sdr
set -uo pipefail
cd ~/oneseg-rs
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export PATH="$HOME/.hermes/tools/ffmpeg-9.0.1-linux-x64/bin:$PATH"

FREQ=${1:-485142857}
FS=1015873
FIFO=/tmp/isdbt_ff.ts
LOG=/tmp/isdbt_ff.log
rm -f "$FIFO"; mkfifo "$FIFO"
: > "$LOG"

# ffplay を先に起動して FIFO の reader を作る。
nohup ffplay -autoexit -window_title "1seg ${FREQ}Hz" -i "$FIFO" \
      >/tmp/isdbt_ffplay.log 2>&1 &
FF_PID=$!
sleep 2

# 受信側: rtl_sdr → stream_decode（PAT/PMT 注入） → FIFO
(
  rtl_sdr -f "$FREQ" -s "$FS" -g 0 - 2>>"$LOG" \
  | ./target/release/examples/stream_decode - - 2>>"$LOG" \
  > "$FIFO"
) >/dev/null 2>&1 &

echo "ffplay 起動: pid=$FF_PID  周波数=$FREQ  ログ=$LOG"
