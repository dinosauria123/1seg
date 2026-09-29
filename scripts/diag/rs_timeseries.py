#!/usr/bin/env python3
"""訂正不能率と synd の時系列（正しい分母で）。

**重要な注意**: 以前の `[dbg]` 行は `rs_err={訂正blk}/{訂正bit}blk` と表示して
いたが、第 2 の値は**ブロック数ではなく訂正されたビット総数**だった
（ラベル `blk` が誤り）。初版スクリプトはこれを「総ブロック数」と読み、
`synd/blk = 0.063` という**全く別の量**を計算していた。その数値を根拠に
「SINR 不足が原因」と結論したのは誤り。

現在 `[dbg]` 行のラベルを直し `総blk=`（RS 復号器が受け取ったブロック総数、
訂正不要を含む）を追加したので、正しい分母で計算できる。
"""
import subprocess, os, re, sys

BIN = "./target/release/examples/stream_decode"
env = dict(os.environ, ISDBT_DEBUG="1")
f = sys.argv[1] if len(sys.argv) > 1 else "/tmp/rec_diag.iq"
p = subprocess.run([BIN, f, "/dev/null", "--live"],
                   capture_output=True, text=True, env=env, timeout=1800)

rows = []
for ln in p.stderr.splitlines():
    m = re.search(r"訂正blk=(\d+) 訂正bit=(\d+) 総blk=(\d+) drop=(\d+)", ln)
    if m:
        rows.append((int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4))))

if not rows:
    print("no data"); sys.exit(1)

print(f"{'総blk':>9s} {'訂正blk':>8s} {'訂正bit':>8s} {'drop':>7s} "
      f"{'bit/訂正blk':>12s} {'drop/総blk':>11s}")
step = max(1, len(rows) // 22)
prev = rows[0]
for i in range(0, len(rows), step):
    rc, rb, seen, rd = rows[i]
    if seen <= prev[2]:
        continue
    d_seen = seen - prev[2]
    d_rc = rc - prev[0]
    d_rb = rb - prev[1]
    d_rd = rd - prev[3]
    bpb = d_rb / d_rc if d_rc else 0.0
    print(f"{seen:9d} {rc:8d} {rb:8d} {rd:7d} "
          f"{bpb:12.2f} {d_rd/d_seen*100:10.1f}%")
    prev = (rc, rb, seen, rd)
