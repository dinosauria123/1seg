#!/usr/bin/env bash
# ライブ 1seg 視聴（HTTP サーバ無し・TS FIFO を ffplay に直接渡す）。
#
# なぜ HTTP を挟まないのか（2026-10-03 実測）
#
# ts_http_server.py を挟むと 200 OK + chunked ヘッダまでは出るが、
# 1 バイトもボディが届かないままクライアントはタイムアウトする
# （curl: 0 bytes received、サーバ側 pump は rchar 2.4MB まで進む）。
# 経路を 1 つ挟むだけで「ヘッダは出る・データは出ない」という壊れ方になり、
# 実測しても切り分けに時間がかかる。ffplay は FIFO を直接読めるので
# サーバ層を完全に外してデコード経路だけを検証する。
#
# 使い方: ./scripts/live_play_direct.sh [周波数Hz]
# 停止:   ./scripts/stop_live.sh
set -uo pipefail
cd ~/oneseg-rs
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

FREQ="${1:-485142857}"
# ゲインは **-g 5**。自動でも 0 でも飽和する（実測 2026-10-03 23:2x）。
#
#   gain   rms    dBFS   ADC飽和(rail)   RS復号率   SNR
#     0    93.1   -2.7      34.1%         0.840     5.1 dB
#     5    51.4   -7.9       1.5%         0.980    13.7 dB
#     8     -     -           -            -      12.6 dB
#    10     -     -           -            -      11.4 dB
#    12     -     -           -            -      11.3 dB
#    15    85.6   -3.9      19.8%         0.820     6.9 dB
#   20-49  85.3   -3.5      23.8%           -         -
#
# `-g 0` は「自動ゲイン」ではなく手動 0 dB で、RTL2832U 側で最大 VGA 利得に
# 落ちるため 8bit ADC の 34% が端値 0/255 に張り付く。**信号は弱いのではなく
# 強すぎて飽和していた。** `-g` を付けない真の自動ゲインも rail 34.4% で
# 同じく飽和するため、DS-DT-305BK + 札幌の 1seg では AGC が機能していない。
# rail を最小化できる -g 5 が実測最適。
GAIN="${GAIN:-5}"
IQFIFO=/tmp/isdbt_iq.fifo
IQFILE=/tmp/isdbt_live.iq
TSFIFO=/tmp/isdbt_ts.fifo
LOG=/tmp/isdbt_live.log
PLOG=/tmp/isdbt_ffplay.log
ALIVE=/tmp/isdbt_writer.alive

# ロック監視を起動する。ロック到達まで 1 分以上かかるため、
# このスクリプトは起動した側にロック到達を待たせない。
# lock_watch.py は /tmp/isdbt_live.log を逐次読みして到達を
# /tmp/isdbt_lock.log に時刻付きで記録し続ける。
#   監視開始: tail -f /tmp/isdbt_lock.log
#   監視停止: pkill -f lock_watch.py
#
# 注意: **stop_live.sh より後ろで起動すること。** stop_live.sh は
# lock_watch.py も（PID ファイル経由で）kill するので、先に起動すると
# 即座に殺される。
bash "$HOME/oneseg-rs/scripts/stop_live.sh" >/dev/null 2>&1
sleep 1

PIDF=/tmp/isdbt_launcher.pid
: > /tmp/isdbt_lock.log
nohup python3 "$HOME/oneseg-rs/scripts/lock_watch.py" >/dev/null 2>&1 &
WATCH_PID=$!
# stop_live.sh が PID ファイルで落とせるように記録する（`pgrep -f` は
# 呼び出し元シェルにマッチして自爆するため使わない）。
echo "$WATCH_PID" > "$PIDF"
echo "lock_watch=$WATCH_PID  →  tail -f /tmp/isdbt_lock.log"

rm -f "$IQFIFO" "$IQFILE" "$TSFIFO" "$ALIVE"
mkfifo "$IQFIFO"; mkfifo "$TSFIFO"
: > "$LOG"; : > "$PLOG"

# 1) rtl_sdr → IQFIFO → IQFILE（append）。stream_decode は --follow で
#    IQFILE を追従して読むので、生 IQ はここで溜め込む。
( exec 3>"$IQFILE"; cat "$IQFIFO" >&3 ) >/dev/null 2>&1 &
sleep 1

# 2) rtl_sdr を起動
rtl_sdr -f "$FREQ" -s 1015873 -g "$GAIN" "$IQFIFO" >>"$LOG" 2>&1 &
sleep 2
echo -n 1 > "$ALIVE"

# 3) stream_decode --follow で IQFILE を追従読取 → TSFIFO
ISDBT_WRITER="$ALIVE" nohup \
  stdbuf -oL ./target/release/examples/stream_decode "$IQFILE" "$TSFIFO" --live --follow \
  >>"$LOG" 2>&1 &
DEC_PID=$!
sleep 1

# 4) TS FIFO を ffplay に直接渡す（HTTP サーバなし）
nohup ffplay -fflags nobuffer -flags low_delay -framedrop \
  -window_title "1seg 札幌NHK総合" \
  -f mpegts -i "$TSFIFO" > "$PLOG" 2>&1 &
FFPLAY_PID=$!

echo "ffplay=$FFPLAY_PID  dec=$DEC_PID  lock_watch=$WATCH_PID"
echo "ロック監視: tail -f /tmp/isdbt_lock.log   ← 1〜2 分待つとロック到達が出る"
echo "ログ: $LOG / $PLOG"
echo "停止: $HOME/oneseg-rs/scripts/stop_live.sh  （lock_watch も一緒に止まる）"