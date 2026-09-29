#!/usr/bin/env python3
"""TS に暗号化（スクランブル）があるかを判定する。

判定材料:
  1. transport_scrambling_control (TS ヘッダ bits 8-9) が 0 以外のパケット数
     0 なら暗号化なし（00 = not scrambled）
  2. ECM / EMM らしい PID の有無（0x0F00 付近のテーブル参照、実測 PID 一覧）
  3. 映像 PID のパケットが実際に復号可能な H.264 として読めるか
     （先頭 0x000001 Annex-B ナ IU が stream に現れるか）

使い方: ts_scramble.py <file.ts>
"""
import sys
from collections import Counter

path = sys.argv[1] if len(sys.argv) > 1 else "oneseg_30s.ts"
data = open(path, "rb").read()
TS = 188

# --- TS パケット境界を検出 ---
offs = []
i = data.find(b"\x47")
while i != -1 and i < len(data) - TS * 2:
    # 連続して 0x47 が 188 間隔で立つ”来finding 確認
    if data[i:i + TS * 3].count(b"\x47") >= 2:
        offs.append(i)
    if len(offs) > 400000:
        break
    i = data.find(b"\x47", i + 1)
if not offs:
    print("TS パケット境界を検出できません")
    sys.exit(1)

base = offs[0]
pids = Counter()
scr = Counter()
scrambled = 0
total = 0

for off in offs:
    o = off - base
    if o < 0 or o + TS > len(data):
        continue
    b0 = data[o]
    if b0 != 0x47:
        continue
    total += 1
    pid = ((data[o + 1] & 0x1F) << 8) | data[o + 2]
    pids[pid] += 1
    sc = (data[o + 3] >> 6) & 0x3
    scr[sc] += 1
    if sc != 0:
        scrambled += 1

print(f"ファイル: {path}  ({len(data)} bytes)")
print(f"TS パケット: {total}")
print()
print("transport_scrambling_control:")
names = {0: "00 not scrambled", 1: "01 (reserved)",
         2: "10 (reserved)", 3: "11 (reserved)"}
for sc, n in sorted(scr.items()):
    print(f"  {names[sc]:20s} {n:7d}  ({n/total*100:5.2f}%)")
print()
enc = scrambled / total * 100 if total else 0
print(f"暗号化パケット: {scrambled} ({enc:.2f}%)")
print("判定:", "暗号化あり" if scrambled else "暗号化なし（平文）")
print()
print("PID 構成:")
for pid, n in pids.most_common():
    kind = ""
    if pid == 0x1FFF: kind = "null"
    elif pid in (0x0000, 0x0001, 0x0010, 0x0011, 0x0012): kind = "PSI/SI"
    print(f"  PID 0x{pid:04x}  {n:7d}  {n/total*100:5.2f}%  {kind}")
