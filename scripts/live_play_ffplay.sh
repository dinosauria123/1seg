#!/usr/bin/env bash
# ライブ 1seg 視聴（ffplay + HTTP 方式）。
#
# # なぜ ffplay か（2026-09-26 実測）
#
# データ経路は完全に見えており、TS も健全:
#   - TS 出力 47〜49 kB/s（実時間 40,000 B/s の 119〜122%）
#   - 188 バイト整列 True、sync byte 不正 0
#   - ffprobe が H.264 320x180 + AAC 48kHz を認識
#   - ffmpeg が HTTP ストリームから 3 フレームを正常にデコード
#
# 一方 VLC は TCP 接続（ESTAB）・読み取り 1.5 MB 済み・ログにエラーなし
# にもかかわらず、CPU 0.1%、MPRIS の Position が 0 のまま進まない。
# `--demux=ts` / `--demux=avformat` の両方とも同じ症状。
# つまり VLC の mpegts demuxer 側固有の問題で、入力データの問題ではない。
#
# そこでプレイヤーを ffplay（ffmpeg 系）に切り替える。ffmpeg 系の
# 復号経路はこのストリームで正常にフレームを出力できることを実証済み。
#
# 使い方: ./scripts/live_play_ffplay.sh [周波数Hz]
# 停止:   ./scripts/stop_live.sh
set -uo pipefail
cd ~/oneseg-rs
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

FREQ="${1:-485142857}"
PORT="${PORT:-8080}"
IQFIFO=/tmp/isdbt_iq.fifo
IQFILE=/tmp/isdbt_live.iq
TSFIFO=/tmp/isdbt_ts.fifo
LOG=/tmp/isdbt_live.log
PLOG=/tmp/isdbt_ffplay.log
ALIVE=/tmp/isdbt_writer.alive

bash "$HOME/oneseg-rs/scripts/stop_live.sh" >/dev/null 2>&1
sleep 1

rm -f "$IQFIFO" "$IQFILE" "$TSFIFO" "$ALIVE"
mkfifo "$IQFIFO"; mkfifo "$TSFIFO"
: > "$LOG"; : > "$PLOG"

# 1) rtl_sdr → IQFIFO → IQFILE（append）
( exec 3>"$IQFILE"; cat "$IQFIFO" >&3 ) >/dev/null 2>&1 &
sleep 1

# 2) rtl_sdr を起動
rtl_sdr -f "$FREQ" -s 1015873 -g 0 "$IQFIFO" >>"$LOG" 2>&1 &
RTL_PID=$!
sleep 2
echo -n 1 > "$ALIVE"

# 3) stream_decode --follow で IQFILE を追従読取 → TSFIFO
ISDBT_WRITER="$ALIVE" nohup \
  stdbuf -oL ./target/release/examples/stream_decode "$IQFILE" "$TSFIFO" --live --follow \
  >>"$LOG" 2>&1 &
DEC_PID=$!
sleep 1

# 4) HTTP サーバ（FIFO を stdin として読む）
nohup python3 "$HOME/oneseg-rs/scripts/ts_http_server.py" "$PORT" < "$TSFIFO" \
  >>"$LOG" 2>&1 &
SRV_PID=$!
sleep 3

# 5) ffplay で再生
#    -fflags nobuffer / -flags low_delay で低遅延、-autoexit は付けない
#    （ユーザーは自分で閉じるまで再生を続けてほしい）。
nohup ffplay -fflags nobuffer -flags low_delay -framedrop \
  -window_title "1seg 札幌NHK総合" \
  "http://127.0.0.1:$PORT/stream.ts" > "$PLOG" 2>&1 &
FFPLAY_PID=$!

echo "ffplay=$FFPLAY_PID  rtl_sdr=$RTL_PID  dec=$DEC_PID  srv=$SRV_PID"
echo "URL: http://127.0.0.1:$PORT/stream.ts"
echo "ログ: $LOG / $PLOG"
echo "停止: $HOME/oneseg-rs/scripts/stop_live.sh"
