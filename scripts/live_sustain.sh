#!/usr/bin/env bash
# ライブ境界追跡の持続性試験: rtl_sdr → stream_decode --live を一定時間走らせ、
# TS 出力が成長し続けるか（= 境界が破綻しないか）を確認する。
#
# 使い方: ./scripts/live_sustain.sh [秒数] [出力ts]
set -uo pipefail
cd ~/oneseg-rs
SECS="${1:-100}"
OUT="${2:-/tmp/live_sustain.ts}"
LOG=/tmp/live_sustain.log

pkill -9 -x stream_decode 2>/dev/null
pkill -9 -x rtl_sdr 2>/dev/null
sleep 1
: > "$LOG"
rm -f "$OUT"

# 先に SECS 秒キャプチャし、その IQ を --live で復調する。
# パイプ（rtl_sdr | stream_decode）だと、stream_decode の reader.read() が
# 通常ファイルの現在端で EOF(=Ok(0)) になりループを抜けてしまうため。
# キャプチャ長を十分長く取ることで、境界が破綻せず復号が続くかを検証できる
# （元バグ: 約 7 秒で RS 出力が恒久停止した）。
rtl_sdr -f 485142857 -s 1015873 -g 0 "$IQ" > /tmp/sustain_rtl.log 2>&1 &
RP=$!
echo "キャプチャ中: ${SECS}秒 ..."
sleep "$SECS"
kill -9 $RP 2>/dev/null
sleep 1
echo "=== IQ: $(stat -c%s "$IQ" 2>/dev/null) B ==="

ISDBT_DEBUG=1 ./target/release/examples/stream_decode "$IQ" "$OUT" --live >> "$LOG" 2>&1
echo "=== decoder 終了コード: $? ==="
echo "=== 最終サイズ: $(stat -c%s "$OUT" 2>/dev/null) B ==="
echo "=== 再取得回数: $(grep -c '再取得: ' "$LOG" 2>/dev/null) ==="
echo "=== RS停止: $(grep -c 'RS 停止' "$LOG" 2>/dev/null) ==="
echo "=== 再取得ログ（末尾8件） ==="
grep '再取得: ' "$LOG" | tail -8
