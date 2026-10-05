#!/usr/bin/env python3
"""PRBS リセット周期を走査し、どれが「先頭も後半も安定」か判定する。

症状: 固定 64 では先頭 2 フレームは完璧、後半で訂正不能が累積する。
仮説: 周期が 64 以外。先頭だけの探索（128 blk = 周期64の2周期）では
      周期 32 でも 64 でも 78 でも区別できず、最後の候補を掴む。
      実装の kept bit 数 204×384×2×3/4 = 117,504 が 64 ブロック分
      (96,256) と一致しない点が矛盾。

判定: 長い capture で drop/総blk が周期 無関係に低ければその周期が正解。
"""
import os, re, subprocess, sys

BIN = "./target/release/examples/stream_decode"
IQ = sys.argv[1] if len(sys.argv) > 1 else "/tmp/rec_diag.iq"
PERIODS = [int(x) for x in (sys.argv[2].split(",") if len(sys.argv) > 2
                            else "16,24,32,48,52,64,78,96,128".split(","))]

print(f"capture: {IQ}  ({os.path.getsize(IQ)/1e6:.1f} MB)")
print(f"{'period':>7s} {'総blk':>8s} {'bit/訂正blk':>12s} "
      f"{'drop/総blk':>11s}  {'先頭1/4':>9s} {'末尾1/4':>9s}  評価")

for rp in PERIODS:
    env = dict(os.environ, ISDBT_DEBUG="1", ISDBT_RPERIOD=str(rp))
    try:
        r = subprocess.run([BIN, IQ, "/dev/null", "--live"],
                           capture_output=True, text=True, env=env, timeout=2400)
    except subprocess.TimeoutExpired:
        print(f"{rp:7d}  タイムアウト"); continue

    rows = []
    for ln in r.stderr.splitlines():
        m = re.search(r"訂正blk=(\d+) 訂正bit=(\d+) 総blk=(\d+) drop=(\d+)", ln)
        if m:
            rows.append((int(m.group(1)), int(m.group(2)),
                         int(m.group(3)), int(m.group(4))))
    if not rows:
        print(f"{rp:7d}  データなし"); continue

    c, b, s, d = rows[-1]
    if s == 0:
        print(f"{rp:7d}  総blk=0"); continue
    bpb = b / c if c else 0.0
    overall = d / s * 100

    # 経時系列を 4 分割して、先頭と末尾の drop 率を比較する。
    # これが「後半で崩れる」を示す。
    q = max(1, len(rows) // 4)
    def rate(seg):
        if len(seg) < 2: return 0.0
        ds = seg[-1][2] - seg[0][2]
        dd = seg[-1][3] - seg[0][3]
        return dd / ds * 100 if ds else 0.0
    head = rate(rows[:q])
    tail = rate(rows[-q:])

    if overall < 5 and tail < 15:
        verdict = "★安定"
    elif overall < 15:
        verdict = "改善"
    elif tail > head * 3 and tail > 20:
        verdict = "後半崩壊"
    else:
        verdict = "="
    print(f"{rp:7d} {s:8d} {bpb:12.2f} {overall:10.1f}%  "
          f"{head:8.1f}% {tail:8.1f}%  {verdict}")
