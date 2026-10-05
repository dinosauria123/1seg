#!/usr/bin/env python3
"""reacquire の却下（reject）が実際に起きているか、その位置と影響を集計する。

`process()` の境界補正ブロックは `if delta.abs() < sym/2 { ... } else { reject }`
の **外側** に書かれているため、却下された peak でも
`self.cur += syms*lk.sym` / `self.k += syms` が実行される。`syms` は
`round(delta/sym)` なので、却下条件 `|delta| >= sym/2` を満たす peak は
必ず |syms| >= 1 のジャンプを導入する。

このスクリプトは `[k]` 行（境界補正のたびに ISDBT_DEBUG で出力される）を
解析し、blk 位置・Δ・syms の相関と「却下ジャンプが起きた回数」を出す。
"""
import subprocess, os, re, sys, collections

BIN = "./target/release/examples/stream_decode"
f = sys.argv[1] if len(sys.argv) > 1 else "captures/atk3.iq"
env = dict(os.environ, ISDBT_DEBUG="1")
p = subprocess.run([BIN, f, "/dev/null", "--live"],
                   capture_output=True, text=True, env=env, timeout=3600)

rows = []
reacq = 0
for ln in p.stderr.splitlines():
    m = re.search(r"再取得: cur=(\d+) → peak=(\d+) \(Δ(-?\d+)\) metric=([\d.]+)", ln)
    if m:
        reacq += 1
    k = re.search(r"\[k\] cur=(\d+) k=(\d+) kmod4=(\d+) Δ=(-?\d+) syms=(-?\d+) adj=(-?\d+) frac=([+-][\d.]+) metric=([\d.]+)", ln)
    if k:
        rows.append(dict(cur=int(k.group(1)), k=int(k.group(2)),
                         kmod4=int(k.group(3)), delta=int(k.group(4)),
                         syms=int(k.group(5)), adj=int(k.group(6)),
                         frac=float(k.group(7)), metric=float(k.group(8))))
    if "raq" in ln or "却下" in ln or "reject" in ln:
        print("NOTE:", ln[:160])

if not rows:
    print("no [k] rows"); sys.exit(1)

sym = 1152
print(f"\nreacquire 試行数 = {reacq}, [k] 行 = {len(rows)}")
print(f"{'blk':>8s} {'k':>8s} {'k%4':>5s} {'delta':>8s} {'syms':>6s} "
      f"{'|d|>=sym/2':>11s} {'metric':>8s} {'frac':>7s}")

jump_rejected = 0
nsym = collections.Counter()
for i, r in enumerate(rows):
    bad = abs(r["delta"]) >= sym // 2
    if bad:
        nsym[r["syms"]] += 1
        if r["syms"] != 0:
            jump_rejected += 1
    if i % max(1, len(rows) // 30) == 0 or bad:
        print(f"{i:8d} {r['k']:8d} {r['kmod4']:5d} {r['delta']:8d} "
              f"{r['syms']:6d} {str(bad):>11s} {r['metric']:8.4f} {r['frac']:+7.3f}")

print(f"\n却下された peak のうち syms != 0（= 1 シンボル以上のジャンプ導入）: "
      f"{jump_rejected} / {len(rows)}")
print(f"syms 内訳（却下時）: {dict(nsym)}")
nz = sum(1 for r in rows if r["syms"] != 0)
print(f"全体での syms != 0: {nz} / {len(rows)}")
