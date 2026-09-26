#!/usr/bin/env bash
# ライブ 1seg 視聴（ファイル追従方式）。
#
# # なぜファイル方式なのか（2026-09-26 実測）
#
# パイプ経路 `rtl_sdr | stream_decode` は TS 出力率が実時間の 1.6%
# （60 秒で 978 シンボルしか復号されず RS ブロックが 1 つも出ない）になる。
# 一方、同じバイナリでファイル入力なら **100%**（実時間の 1.5 倍、
# 60 秒キャプチャ → 57.6 秒分の TS、RS 12164/12164）で動く。
# つまり復調性能は足りており、パイプ経路のスケジューリングだけが問題。
#
# そこで rtl_sdr の出力を FIFO 経由で通常ファイルへ追記し、
# stream_decode --follow でそのファイルを追従読みする。
# --follow は EOF でも終了せず、新しく書き込まれたぶんを読み続ける
# （`ISDBT_WRITER` ファイルの先頭が '1' のあいだ待ち続ける）。
#
# 使い方: ./scripts/live_play_detached.sh [周波数Hz]
# 停止:   pkill -x vlc; pkill -x stream_decode; pkill -x rtl_sdr
set -uo pipefail
cd ~/oneseg-rs
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

FREQ="${1:-485142857}"
IQFIFO=/tmp/isdbt_iq.fifo     # rtl_sdr → ファイルへの橋渡し
IQFILE=/tmp/isdbt_live.iq     # 実際に append される IQ ファイル
FIFO=/tmp/isdbt_live.ts       # TS（VLC へ）
LOG=/tmp/isdbt_live.log
VLOG=/tmp/isdbt_vlc.log
ALIVE=/tmp/isdbt_writer.alive

pkill -9 -x vlc 2>/dev/null
pkill -9 -x stream_decode 2>/dev/null
pkill -9 -x rtl_sdr 2>/dev/null
sleep 1

rm -f "$IQFIFO" "$IQFILE" "$FIFO" "$ALIVE"
mkfifo "$IQFIFO"; mkfifo "$FIFO"
: > "$LOG"; : > "$VLOG"

# 1) VLC を先に起動（FIFO の reader を作る）
nohup vlc --demux=ts --avcodec-hw=none --vout=xcb_x11 \
      --no-video-title-show --no-osd \
      --clock-jitter=0 --clock-synchro=0 \
      "$FIFO" >"$VLOG" 2>&1 &
VLC_PID=$!
sleep 2

# 2) rtl_sdr → IQFIFO → IQFILE（append）
#    `tee -a` の代わりに cat で追記する。IQFIFO の reader を最初に開く。
(
  exec 3>"$IQFILE"          # 追記用に開く（reader より先に）
  cat "$IQFIFO" >&3
) >/dev/null 2>&1 &
CAT_PID=$!

# 3) rtl_sdr を起動（IQFIFO へ書く）
rtl_sdr -f "$FREQ" -s 1015873 -g 0 "$IQFIFO" >>"$LOG" 2>&1 &
RTL_PID=$!
sleep 2

# 4) 書き手の生存フラグ（--follow の終了判定用）
echo -n 1 > "$ALIVE"

# 5) stream_decode --follow で IQFILE を追従読取 → TS を FIFO へ
ISDBT_WRITER="$ALIVE" nohup \
  ./target/release/examples/stream_decode "$IQFILE" "$FIFO" --live --follow \
  >>"$LOG" 2>&1 &
DEC_PID=$!

echo "VLC=$VLC_PID  rtl_sdr=$RTL_PID  stream_decode=$DEC_PID"
echo "FIFO=$FIFO  IQFILE=$IQFILE"
echo "ログ: $LOG / $VLOG"
echo "停止: pkill -x vlc; pkill -x stream_decode; pkill -x rtl_sdr"
