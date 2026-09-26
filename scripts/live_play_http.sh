#!/usr/bin/env bash
# ライブ 1seg 視聴（ファイル追従 + TCP 配信方式）。
#
# # なぜこの構成なのか（2026-09-26 実測）
#
# 1) パイプ経路 `rtl_sdr | stream_decode` は TS 出力率が実時間の 1.6%
#    （60 秒で 978 シンボルしか復号されず RS ブロックが 1 つも出ない）。
#    同じバイナリでファイル入力なら 100%（実時間の 1.5 倍）なので、
#    復調性能は足りておりパイプ経路のスケジューリングだけが問題。
#    → rtl_sdr の出力を IQ ファイルへ追記し、`--follow` で追従読みする。
#
# 2) TS の受け渡しは FIFO ではなく TCP（http）にする。
#    FIFO 経由では TS は 49 kB/s（実時間の 122%）で確実に届いていたのに
#    VLC の再生位置が 0 のまま止まる（VLC ログにエラーなし、CPU 0%）だった。
#    http 配信なら VLC 側が「サーバーが，推出すれば再生」という通常の
#    経路を使うため、この症状が出ない。
#
# 使い方: ./scripts/live_play_http.sh [周波数Hz]
# 停止:   pkill -x vlc; pkill -x stream_decode; pkill -x rtl_sdr
set -uo pipefail
cd ~/oneseg-rs
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

FREQ="${1:-485142857}"
PORT="${PORT:-8080}"
IQFIFO=/tmp/isdbt_iq.fifo
IQFILE=/tmp/isdbt_live.iq
LOG=/tmp/isdbt_live.log
VLOG=/tmp/isdbt_vlc.log
ALIVE=/tmp/isdbt_writer.alive

pkill -9 -x vlc 2>/dev/null
pkill -9 -x stream_decode 2>/dev/null
pkill -9 -x rtl_sdr 2>/dev/null
pkill -9 -f "http.*$PORT" 2>/dev/null
sleep 1

rm -f "$IQFIFO" "$IQFILE" "$ALIVE"
mkfifo "$IQFIFO"
: > "$LOG"; : > "$VLOG"

# /tmp は tmpfs（実測 3.6 GB）。診断用の IQ キャプチャを放置すると
# quota exceeded でデコーダの TS 出力が丸ごと失敗し、プレイヤーが
# 「0:00 のまま黒画面」で止まる（実測 2026-09-26）。
echo "起動前 /tmp 空き: $(df -h /tmp | awk 'NR==2{print $4}')"

# 1) rtl_sdr → IQFIFO → IQFILE（append）
( exec 3>"$IQFILE"; cat "$IQFIFO" >&3 ) >/dev/null 2>&1 &
CAT_PID=$!
sleep 1

# 2) rtl_sdr を起動
rtl_sdr -f "$FREQ" -s 1015873 -g 0 "$IQFIFO" >>"$LOG" 2>&1 &
RTL_PID=$!
sleep 2
echo -n 1 > "$ALIVE"

# 3) stream_decode --follow で IQFILE を追従読取 → TCP で配信
#    パイプ接続は `nohup A | B &` では input が正しく繋がらないことがあるため、
#    命名パイプ（TSFIFO）で明示的に繋ぐ。stdbuf -oL で行バッファリングを無効化し、
#    低遅延にする。
TSFIFO=/tmp/isdbt_ts.fifo
rm -f "$TSFIFO"; mkfifo "$TSFIFO"

ISDBT_WRITER="$ALIVE" nohup \
  stdbuf -oL ./target/release/examples/stream_decode "$IQFILE" "$TSFIFO" --live --follow \
  >>"$LOG" 2>&1 &
DEC_PID=$!
sleep 1

# サーバーは FIFO を stdin として読む（先に reader を開かないと writer がブロックする）
nohup python3 "$HOME/oneseg-rs/scripts/ts_http_server.py" "$PORT" < "$TSFIFO" \
  >>"$LOG" 2>&1 &
SRV_PID=$!
sleep 3

# 4) VLC（http で購読）
nohup vlc --demux=ts --avcodec-hw=none --vout=xcb_x11 \
      --no-video-title-show --no-osd \
      --clock-jitter=0 --clock-synchro=0 \
      --network-caching=2000 \
      "http://127.0.0.1:$PORT/stream.ts" >"$VLOG" 2>&1 &
VLC_PID=$!

echo "VLC=$VLC_PID  rtl_sdr=$RTL_PID  dec=$DEC_PID  srv=$SRV_PID"
echo "URL: http://127.0.0.1:$PORT/stream.ts"
echo "ログ: $LOG / $VLOG"
echo "停止: pkill -x vlc; pkill -x stream_decode; pkill -x rtl_sdr"
