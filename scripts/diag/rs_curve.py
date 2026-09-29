#!/usr/bin/env python3
"""訂正不能率の推移を**先頭から全点**出力する（間引かない）。

`rs_timeseries.py` は 22 点に間引くので「どこから崩れ始めるか」が見えない。
劣化の開始点と、その直前に何が起きているかを特定するため全点出す。
"""
import subprocess, os, re, sys

BIN = "./target/release/examples/stream_decode"
f = sys.argv[1] if len(sys.argv) > 1 else "captures/atk3.iq"
env = dict(os.environ, ISDBT_DEBUG="1")
p = subprocess.run([BIN, f, "/dev/null", "--live"],
                   capture_output=True, text=True, env=env, timeout=3600)

rows = []
for ln in p.stderr.splitlines():
    m = re.search(r"訂正blk=(\d+) 訂正bit=(\d+) 総blk=(\d+) drop=(\d+)", ln)
    if m:
        rows.append(tuple(int(m.group(i)) for i in (1, 2, 3, 4)))

if not rows:
    print("no data"); sys.exit(1)

print(f"{'blk':>9s} {'rate':>9s}")
prev = rows[0]
first_bad = None
for i, (rc, rb, seen, rd) in enumerate(rows):
    d_seen = seen - prev[2]
    d_rd = rd - prev[3]
    rate = d_rd / d_seen * 100 if d_seen else 0.0
    if first_bad is None and rate > 50:
        first_bad = i
    if i % max(1, len(rows) // 120) == 0 or (first_bad is not None and abs(i - first_bad) <= 6):
        print(f"{seen:9d} {rate:8.2f}%")
    prev = (rc, rb, seen, rd)

print(f"\n総 [dbg] 行 = {len(rows)}, 最終 総blk = {rows[-1][2]}")
if first_bad is not None:
    print(f"rate>50% の最初の行 = index {first_bad} (総blk={rows[first_bad][2]})")
