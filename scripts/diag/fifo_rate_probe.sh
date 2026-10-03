#!/usr/bin/env bash
# FIFO 経由の TS 出力レートと PCR の進みを実測する。
#
# ffplay を起動しないので、FIFO を独占して読み取れる。
# PCR の進みと実時間の比が 1.0 未満なら、復調器が実時間より速く
# TS を出している = バッファが成長する = 音聲が遅れる。
set -uo pipefail
cd ~/oneseg-rs

bash scripts/stop_live.sh >/dev/null 2>&1
sleep 2

IQF=/tmp/m_iq.fifo
TSF=/tmp/m_ts.fifo
IQ=/tmp/m_live.iq
rm -f "$IQF" "$TSF" "$IQ"
mkfifo "$IQF" "$TSF"

( exec 3>"$IQ"; cat "$IQF" >&3 ) >/dev/null 2>&1 &
sleep 1
rtl_sdr -f 485142857 -s 1015873 -g 5 "$IQF" >/dev/null 2>&1 &
sleep 2
echo -n 1 > /tmp/m_writer.alive
ISDBT_WRITER=/tmp/m_writer.alive stdbuf -oL \
  ./target/release/examples/stream_decode "$IQ" "$TSF" --live --follow \
  >/tmp/m_dec.log 2>&1 &

echo "ロック待ち（最大 150 秒）..."
locked=0
for i in $(seq 1 150); do
  if grep -q "SPS/PPS/IDR検出" /tmp/m_dec.log 2>/dev/null; then
    locked=1
    echo "ロック確認（${i} 秒）"
    break
  fi
  sleep 1
done
if [ "$locked" = 0 ]; then
  echo "ロックせず。停止する。"
  bash scripts/stop_live.sh >/dev/null 2>&1
  exit 1
fi

python3 - <<'PY'
import os, time
fd = os.open('/tmp/m_ts.fifo', os.O_RDONLY | os.O_NONBLOCK)
t0 = time.time()
total = 0
first = last = None
while time.time() - t0 < 30:
    try:
        c = os.read(fd, 188 * 200)
    except BlockingIOError:
        time.sleep(0.01); continue
    if not c:
        time.sleep(0.01); continue
    total += len(c)
    for i in range(len(c) // 188):
        o = i * 188
        if c[o] != 0x47: continue
        b1, b2, b3 = c[o+1], c[o+2], c[o+3]
        if ((b3 >> 4) & 3) not in (2, 3): continue
        if c[o+4] < 7 or not (c[o+5] & 0x10): continue
        b = c[o+6:o+12]
        base = b[0]<<25 | b[1]<<17 | b[2]<<9 | b[3]<<1 | b[4]>>7
        v = base + ((b[4] & 1) << 8 | b[5]) / 300.0
        if first is None: first = (time.time() - t0, v)
        last = (time.time() - t0, v)
os.close(fd)

w = time.time() - t0
rate = total / w
print()
print(f"実時間        {w:.2f} s")
print(f"読み取り      {total} B")
print(f"TS レート     {rate:.0f} B/s  = {rate*8/1000:.1f} kbit/s")
print()
if first and last:
    wi = last[0] - first[0]
    pi = last[1] - first[1]
    print(f"PCR 進み      {pi:.3f} s")
    print(f"実時間        {wi:.3f} s")
    print(f"比            {pi/wi:.4f}")
    print()
    # PCR 進みに相当するバイトレート
    per_pcr = total / pi if pi else 0
    print(f"PCR 1 秒あたりのバイト数 {per_pcr:.0f} B/s")
    print(f"実出力 / PCR 基準 = {rate/per_pcr:.4f}")
    print()
    if pi / wi < 0.98:
        print("→ PCR が実時間より遅い。TS が実時間より速く出ている。")
        print("  その差がバッファに積，此次音聲が遅れる。")
    elif pi / wi > 1.02:
        print("→ PCR が実時間より速い。TS が実時間より遅く出ている。")
    else:
        print("→ PCR は実時間に追従。TS 出力はリアルタイム正しい。")
        print("  遅延は FIFO/avio 側（ffplay 側）にある。")
PY

bash scripts/stop_live.sh >/dev/null 2>&1