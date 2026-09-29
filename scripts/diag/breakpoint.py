#!/usr/bin/env python3
"""訂正不能率的どこで「跳ね上がる」かを特定し、再取得の��ードと照合する。

症状: 先頭 1/4 は drop 0.1%、末尾 1/4 は 59.9%。途中で一度だけ跳ね上が���、
      その後回復しない。SINR でも PRBS 周期でもないなら、
      「再取得（reacquisition）後に PRBS 位相がリセットされず、
      以降 block_idx がずれたまま」か「同期が滑って復帰していない」か。
"""
import os, re, subprocess, sys

BIN = "./target/release/examples/stream_decode"
IQ = sys.argv[1] if len(sys.argv) > 1 else "/tmp/rec_diag.iq"
env = dict(os.environ, ISDBT_DEBUG="1")

r = subprocess.run([BIN, IQ, "/dev/null", "--live"],
                   capture_output=True, text=True, env=env, timeout=2400)

pts = []
for ln in r.stderr.splitlines():
    m = re.search(r"訂正blk=(\d+) 訂正bit=(\d+) 総blk=(\d+) drop=(\d+)", ln)
    if m:
        pts.append((int(m.group(3)), int(m.group(4))))
    if ln.startswith("[lock]") or "reacq" in ln.lower() or "再取得" in ln:
        print("  LOCK:", ln.strip()[:150])

if len(pts) < 4:
    print("データ不足"); sys.exit(1)

print(f"\n{'総blk':>9s} {'drop/総blk':>11s}  {'前区間比':>9s}")
prev = pts[0]
for i in range(1, len(pts)):
    ds = pts[i][0] - prev[0]
    dd = pts[i][1] - prev[1]
    if ds <= 0:
        continue
    r_ = dd / ds * 100
    # 前区間からの跳ね上がり量を出す
    print(f"{pts[i][0]:9d} {r_:10.1f}%  {dd:8d}  {'<<< 跳ね上がり' if r_ > 20 else ''}")
    prev = pts[i]

# 累積 drop が最初に 1% を超える総blk を求める
print("\n累積 drop 率の推移:")
first_bad = None
for i in range(1, len(pts)):
    ds = pts[i][0] - pts[0][0]
    dd = pts[i][1] - pts[0][1]
    if ds > 0 and dd / ds > 0.01:
        first_bad = pts[i][0]
        break
if first_bad:
    print(f"  drop が 1% を超えた総blk = {first_bad} / {pts[-1][0]} "
          f"({first_bad/pts[-1][0]*100:.1f}% の時点)")
else:
    print("  1% 超えなし")
