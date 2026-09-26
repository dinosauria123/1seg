#!/usr/bin/env bash
# ライブ 1seg 視聴（ffmpeg remux → VLC 直接接続）。
#
# # なぜ HTTP サーバを使わないのか（2026-09-26 実測）
#
# ts_http_server.py の経路は「curl/ffmpeg/ffplay は同じ URL から
# 正常にデコードできるのに VLC だけ 0:00 で止まる」状態が最後まで
# 解消できなかった。サーバ自身は健全だった:
#   - curl で 20 秒に 1.27 MB 取得（63 KB/s）
#   - ffprobe が h264 320x180 / aac 48kHz を認識
#   - ffplay / ffmpeg も同じ URL から実フレームをデコード
# 一方で VLC は rchar +23 B/s、CPU 0%、Position 0 のまま。
#
# 切り分けの結果、VLC は HTTP chunked ストリームを自前 libdvbpsi で
# 処理ponsibility 持っており、PAT (PID 0) の continuity error 1 件で
# demux を再起動して 0:00 に戻る。VLC 側の問題なので、
# **サーバを挟まず ffmpeg の remux 済み TS を FIFO で VLC へ渡す。**
#
# 使い方: ./scripts/live_play_vlc.sh [周波数Hz]
# 停止:   ./scripts/stop_live.sh
set -uo pipefail
cd ~/oneseg-rs
export DISPLAY="${DISPLAY:-:0}"

FREQ="${1:-485142857}"
IQFIFO=/tmp/isdbt_iq.fifo
IQFILE=/tmp/isdbt_live.iq
RAWFIFO=/tmp/isdbt_ts.fifo
TSFIFO=/tmp/isdbt_mux.fifo
LOG=/tmp/isdbt_live.log
VLOG=/tmp/isdbt_vlc.log
ALIVE=/tmp/isdbt_writer.alive

bash "$HOME/oneseg-rs/scripts/stop_live.sh" >/dev/null 2>&1
sleep 1

# /tmp は tmpfs（実測 3.6 GB）。診断用 IQ を放置すると quota exceeded で
# デコーダの TS 出力が丸ごと失敗する（実測 2026-09-26）。
echo "起動前 /tmp 空き: $(df -h /tmp | awk 'NR==2{print $4}')"

rm -f "$IQFIFO" "$IQFILE" "$RAWFIFO" "$TSFIFO" "$ALIVE"
mkfifo "$IQFIFO"; mkfifo "$RAWFIFO"; mkfifo "$TSFIFO"
: > "$LOG"; : > "$VLOG"

# 1) rtl_sdr → IQFIFO → IQFILE（append して grow させる）
#    パイプ直接だと background 起動時に stdin が閉じて TS 出力が 63 B/s
#    まで落ちるため、named FIFO を挟んでから通常ファイルへ流す。
( exec 3>"$IQFILE"; cat "$IQFIFO" >&3 ) >/dev/null 2>&1 &
sleep 1

# 2) rtl_sdr（485142857 Hz / 1015873 sps / gain 0 が実測でвич好的）
rtl_sdr -f "$FREQ" -s 1015873 -g 0 -b 8 "$IQFIFO" >>"$LOG" 2>&1 &
RTL_PID=$!
sleep 2
echo -n 1 > "$ALIVE"

# 3) stream_decode --follow で IQFILE を追従読取 → RAWFIFO
ISDBT_WRITER="$ALIVE" nohup \
  stdbuf -oL ./target/release/examples/stream_decode "$IQFILE" "$RAWFIFO" --live --follow \
  >>"$LOG" 2>&1 &
DEC_PID=$!
sleep 1

# 4) ffmpeg で remux（-c copy は再符号化しないので軽い）
nohup ffmpeg -hide_banner -loglevel warning \
  -fflags +genpts+igndts -use_wallclock_as_timestamps 0 \
  -f mpegts -i "$RAWFIFO" \
  -c copy -f mpegts -muxdelay 0 -muxpreload 0 "$TSFIFO" \
  >>"$LOG" 2>&1 &
MUX_PID=$!
sleep 2

# 5) VLC（**FIFO を直接読む**。HTTP も ts_http_server も使わない）
#    VLC の MRL は名前付きパイプでも開ける。stdin を直接渡す
#    `vlc -` は background で fd 0 が閉じて "MRL 'fd://0' を開けません"
#    になるが、FIFO なら writer（ffmpeg）が居続けるので開ける。
nohup vlc --demux=ts --avcodec-hw=none --vout=xcb_x11 \
  --no-video-title-show --no-osd \
  --clock-jitter=0 --clock-synchro=0 --network-caching=3000 \
  < "$TSFIFO" > "$VLOG" 2>&1 &
VLC_PID=$!

echo "VLC=$VLC_PID rtl_sdr=$RTL_PID dec=$DEC_PID mux=$MUX_PID"
echo "経路: rtl_sdr → IQFILE → stream_decode → RAWFIFO → ffmpeg remux → VLC"
echo "ログ: $LOG / $VLOG"
echo "停止: $HOME/oneseg-rs/scripts/stop_live.sh"