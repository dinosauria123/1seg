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
IQFILE=""      # 周波数ごとに決まる（下で設定）
TSFIFO=""      # 周波数ごとに決まる（下で設定）
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

rm -f "$IQFIFO" "$ALIVE"
: > "$LOG"; : > "$PLOG"

# IQ ファイルと TS FIFO は**チャンネルごとに分離**する。
#
# IQ_KEEP で保持してロックを 78 秒 → 16 秒に短縮するのは**同じチャンネルを
# 再起動するときだけ**。チャンネルを切り替えると古い IQ が混ざるので、
# ファイル名を周波数ごとに分ける（`/tmp/isdbt_iq_<freq>.iq`）。
#
# **これが「切替しても前のチャンネルしか出ない」真因**（実測 2026-10-04）。
#  - IQ ファイルが ch15 のままだと、ch19 に切り替えても stream_decode は
#    まず ch15 の古いデータから復調し始める。bin offset 探索は新しい周波数で
#    走るので TMCC 同期は 1.000 になるのに、中身は前のチャンネルになる。
#  - TS FIFO を共有していると、前チャンネルの TS が FIFO に残留して
#    ffplay がそちらを読み続ける。
#
# 検証: ch19 単独で IQ を取り直すと RS 0.926 / PTS 差 -0.009 秒で正常。
#       チャンネルの TS はハッシュが全部違う（f0a5/eb36/b3c5…）ので
#       復調自体は正しく、経路の残留だけが問題だった。
IQFILE="/tmp/isdbt_iq_${FREQ}.iq"
TSFIFO="/tmp/isdbt_ts_${FREQ}.fifo"
IQ_KEEP="${IQ_KEEP:-400000000}"
if [ -f "$IQFILE" ]; then
  sz=$(stat -c%s "$IQFILE" 2>/dev/null || echo 0)
  if [ "$sz" -gt "$IQ_KEEP" ]; then
    # 末尾 $IQ_KEEP バイトだけ残す（先頭を捨ててから完全に作り直す）
    tail -c "$IQ_KEEP" "$IQFILE" > "$IQFILE.new" && mv "$IQFILE.new" "$IQFILE"
    echo "IQ ファイルを $IQ_KEEP バイトに切り詰め"
  fi
fi

# FIFO は TSFIFO が確定してから作る。**`rm -f` より後ろに置く。**
# 順序を逆にすると古い FIFO が消えずに残り、`mkfifo` が `File exists` で
# 失敗して ffplay が起動しない（実測 2026-10-04 02:34「画像が出ない」）。
rm -f "$TSFIFO"
mkfifo "$IQFIFO"; mkfifo "$TSFIFO"

# 1) rtl_sdr → IQFIFO → IQFILE（append）。stream_decode は --follow で
#    IQFILE を追従して読むので、生 IQ はここで溜め込む。
#    append (`>>`) で開く。`: > "$IQFILE"` で truncate しないこと。
( exec 3>>"$IQFILE"; cat "$IQFIFO" >&3 ) >/dev/null 2>&1 &
sleep 1

# 2) rtl_sdr を起動
rtl_sdr -f "$FREQ" -s 1015873 -g "$GAIN" "$IQFIFO" >>"$LOG" 2>&1 &
sleep 2
echo -n 1 > "$ALIVE"

# 3) stream_decode --follow で IQFILE を追従読取 → TSFIFO
#
# **bin offset はチャンネルごとに違う**（実測 2026-10-04 03:0x）。
#
# bin offset の自動探索（`rank_segment_offsets` → RS 復号率で絞り込み）は
# IQ ファイル全体を走査するため、保持量を増やすと比例して時間がかかる。
# 実測: 400 MB（200 秒分）で**ロック到達まで 63 秒**。IQ を消していた
# 頃は 78 秒。IQ を多く保持しても探索時間は減らない。
#
# 1 bin = 992 Hz の量子化なので、bin offset は**チャンネルごとに違う**。
#
# **測定値は参考。±RETRY で自己修復する**
#
# 自動探索は IQ ファイル全体を走査するので、IQ 量を増やしても 63 秒
# かかる。固定して回避する。
#
# **しかし固定値の測定は不正確**（実測 2026-10-04）:
#   STV の 4 通りの capture 比較（同一 IQ 320 MB、`ISDBT_SEGOFF_RETRY=0`）
#       offset=307  TS=0 B          失敗
#       offset=308  TS=1,572,244 B  **これが正解**
#       offset=309  TS=0 B          失敗
#       offset=310  TS=0 B          失敗
#   一方 survey（20 MB 測定）は 309 と報告していた。つまり survey の値は
#   **偽値**。bin offset の正解は 1 bin の精度で決まるので、20 MB の測定
#   だけでは wrong bin を掴む。
#
# したがって survey の値をそのまま固定するのは危険。**±RETRY で自己修復
# させる**のが正しい（ユーザーの助言: 「ロックしなければ別の数値で試す」）。
# `ISDBT_SEGOFF_RETRY` の分だけ周回するので、survey の値が 1 bin ずれて
# いても 最終的に正解に当たる。
#
# 実測値（`scripts/diag/seg_offset_survey.py`、2026-10-04 03:1x、参考値）:
#   ch13 NHK教育 307      ch14 TVh     308      ch15 NHK総合 308
#   ch19 HBC     308      ch21 STV     309      ch23 HTB     309
#   ch25 UHB     309
#
# 周波数ごとの値を survey が `/tmp/isdbt_segoff_<freq>` に保存する。
# 値が無いチャンネルは 308 を仮定する（リトライが正解に当たるので、
# 63 秒の自動探索より速い）。
SEGOFF_RETRY="${SEGOFF_RETRY:-2}"
SEGOFF_FILE="/tmp/isdbt_segoff_${FREQ}"
if [ ! -f "$SEGOFF_FILE" ]; then
  echo "警告: $SEGOFF_FILE が無い（bin offset を 308 と仮定）。"
  echo "      測り直すには: python3 scripts/diag/seg_offset_survey.py"
  echo "308" > "$SEGOFF_FILE"
fi
SEGOFF="$(cat "$SEGOFF_FILE")"
echo "bin offset: ISDBT_SEGOFF=$SEGOFF ±$SEGOFF_RETRY  ($SEGOFF_FILE)"
ISDBT_WRITER="$ALIVE" ISDBT_SEGOFF="$SEGOFF" ISDBT_SEGOFF_RETRY="$SEGOFF_RETRY" \
  nohup \
  stdbuf -oL ./target/release/examples/stream_decode "$IQFILE" "$TSFIFO" --live --follow \
  >>"$LOG" 2>&1 &
DEC_PID=$!
sleep 1

# 4) TS FIFO を ffplay に直接渡す（HTTP サーバなし）
#
# **A/V 同期について（2026-10-04 実測）**
#
# 現在の設定: `-fflags nobuffer` を外し、`-probesize`/`-analyzeduration`
# を 1 MB / 1 秒に絞った設定。修正前は音声が 4 秒以上遅れていた。
#
# 真因は **stream_decode 側の PtsNormalizer** で、ffplay のオプションでは
# 直らない。PTS を PID ごとに正規化していたため、映像と音声の放送側基準が
# 3.31 時間（11930.46 秒）離れていた。`crates/isdbt-dsp/src/ts.rs` で
# base を PID ごとに持つ設計から、全 PID 共有の単一 base に変えた。
#
# 修正前後の実測（`scripts/diag/ts_av_offset.py`）:
#     修正前  映像 26890.15 s / 音声 38820.62 s  差 **+11930.46 s**
#     修正後  映像/音声                        差 **-0.017 s**
# ユーザー確認: 「音声が画像と一致した」
#
# なお `aq`（ffplay の音声キュー）は約 440 KB で平衡するが、これは
# ファイル入力でも同程度なので SDL/PipeWire 側の通常バッファで、
# 映像/音声の相対関係には影響しない（A-V は ±0.012 s）。
nohup ffplay -flags low_delay -framedrop \
  -probesize 1000000 -analyzeduration 1000000 \
  -window_title "1seg 札幌NHK総合" \
  -f mpegts -i "$TSFIFO" > "$PLOG" 2>&1 &
FFPLAY_PID=$!

echo "ffplay=$FFPLAY_PID  dec=$DEC_PID  lock_watch=$WATCH_PID"
echo "ロック監視: tail -f /tmp/isdbt_lock.log   ← 1〜2 分待つとロック到達が出る"
echo "ログ: $LOG / $PLOG"
echo "停止: $HOME/oneseg-rs/scripts/stop_live.sh  （lock_watch も一緒に止まる）"