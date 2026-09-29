#!/usr/bin/env python3
"""PID ごとに transport_scrambling_control を集計する。

全パNICをまとめて数えると、null/PSI パケットが 0 であるために
「暗号化率」が看似iously高く出る。映像 PID に限定して見る必要がある。

さらに、スクランブル化が『連続区間』か『散発』かも見る:
  - 散発 = 復調ビット誤り（現状の主 suspects）
  - 連続区間 = 本物の暗号化（特定 PID の連続パケットが一気に化ける）
"""
import sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/live.ts"
data = open(path, "rb").read()
TS = 188

# 先頭の 0x47 を基準に 188 固定 stride で読む（境界を厳密に合わせる）
base = -1
for i in range(min(len(data), 4096)):
    if data[i] != 0x47:
        continue
    ok = all(data[i + k * TS] == 0x47 for k in range(1, 20))
    if ok:
        base = i
        break
if base < 0:
    print("TS 境界を検出できません"); sys.exit(1)

n = (len(data) - base) // TS
scr = defaultdict(lambda: [0, 0])
runs = defaultdict(int)
cur_pid, cur_state = -1, 0

for k in range(n):
    o = base + k * TS
    b = data[o]
    if b != 0x47:
        continue
    pid = ((data[o + 1] & 0x1F) << 8) | data[o + 2]
    sc = (data[o + 3] >> 6) & 0x3
    scr[pid][0] += 1
    if sc:
        scr[pid][1] += 1
    if pid == 0x0581:
        if sc == cur_state and cur_pid == pid:
            runs[sc] = max(runs[sc], runs[sc] + 1)
        else:
            runs[sc] = 1
            cur_pid, cur_state = pid, sc

print(f"{path}: {n} パケット (base offset {base})")
print()
print(f"{'PID':>8s} {'総数':>8s} {'scr≠0':>8s} {'割合':>8s}")
for pid in sorted(scr, key=lambda p: -scr[p][0])[:12]:
    tot, s = scr[pid]
    if tot == 0:
        continue
    print(f"0x{pid:04x}   {tot:8d} {s:8d} {s/tot*100:7.2f}%")
print()
print("映像 PID 0x0581 のスクランブル連続区間長:")
for sc in sorted(runs):
    if sc:
        print(f"  sc={sc}: 最長連続 {runs[sc]} パケット")
if not any(runs):
    print("  なし（0x0581 は全パケット sc=0）")
