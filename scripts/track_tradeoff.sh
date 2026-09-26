#!/usr/bin/env bash
# 再取得周期（ISDBT_REACQ）による速度/精度のトレードオフを測る。
# 同じ IQ に対して再取得無効/有効で復調し、TS 出力量と 再取得 Δ を比較する。
set -uo pipefail
cd ~/oneseg-rs

CAP=${1:-30}   # キャプチャ秒
IQ=/tmp/trade.iq

pkill -9 -x stream_decode 2>/dev/null
pkill -9 -x rtl_sdr 2>/dev/null
sleep 1
rm -f "$IQ"
rtl_sdr -f 485142857 -s 1015873 -g 0 "$IQ" >/dev/null 2>&1 &
RP=$!
sleep "$CAP"
kill -9 $RP 2>/dev/null
sleep 1
echo "IQ: $(stat -c%s "$IQ") B (${CAP}秒)"

for R in 0 800; do
  OUT="/tmp/trade_$R.ts"
  rm -f "$OUT"
  ISDBT_REACQ=$R ./target/release/examples/stream_decode "$IQ" "$OUT" --live > "/tmp/trade_$R.log" 2>&1
  S=$(stat -c%s "$OUT" 2>/dev/null || echo 0)
  BL=$(grep -E "ブロック復号" "/tmp/trade_$R.log" | tail -1)
  NQ=$(grep -c "再取得:" "/tmp/trade_$R.log" 2>/dev/null || echo 0)
  echo "REACQ=$R → $S B / ${CAP}秒 = $(echo "scale=1;$S/$CAP"|bc) B/s | $BL | 再取得=$NQ"
done
