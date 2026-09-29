#!/usr/bin/env python3
"""共用アンテナ端子直結時のノイズ源特定。

1) 470-608 MHz を 1 MHz 刻みで掃引し平均電力を測る
2) ピーク（＝放送チャネル）を列挙する
3) 各ピークで 10 秒キャプチャし TMCC ロックと synd/blk を測る

判定:
  - チャネルごとに synd/blk が大きく違う → そのチャネルの干渉/環境の問題
  - どのチャネルでもほぼ同じ              → 受信共通経路（headend 増幅器）
"""
import subprocess, os, sys, array, math, re

FS = 1015873
BIN = "./target/release/examples/stream_decode"
PROBE = "./target/release/examples/tmcc_probe"
env = dict(os.environ, ISDBT_DEBUG="1")


def sweep_power(freq_hz, dur="0.4"):
    p = subprocess.run(
        ["timeout", "2", "rtl_sdr", "-f", str(freq_hz), "-s", str(FS), "-g", "0", "-"],
        capture_output=True, timeout=10)
    a = array.array("B", p.stdout[:400000])
    if not a:
        return 0.0
    n = len(a)
    s = 0.0
    for v in a:
        x = v - 127.5
        s += x * x
    return (s / n) ** 0.5


def measure_channel(freq_hz, sec=10):
    """1 チャネルの TMCC ロックと synd/blk を測る。"""
    iq = subprocess.run(
        ["timeout", str(sec), "rtl_sdr", "-f", str(freq_hz), "-s", str(FS), "-g", "0", "-"],
        capture_output=True, timeout=sec + 15).stdout
    if len(iq) < 1_000_000:
        return None
    path = f"/tmp/scan_{freq_hz}.iq"
    with open(path, "wb") as fh:
        fh.write(iq)
    # TMCC ロック確認
    pr = subprocess.run([PROBE, path, str(FS)], capture_output=True, text=True, timeout=300)
    locked = "真のロックか          : YES" in pr.stdout
    if not locked:
        os.unlink(path)
        return {"lock": False}
    # synd/blk
    r = subprocess.run([BIN, path, "/dev/null", "--live"],
                       capture_output=True, text=True, env=env, timeout=900)
    err = blk = drop = 0
    for ln in r.stderr.splitlines():
        m = re.search(r"rs_err=(\d+)/(\d+)blk rs_drop=(\d+)", ln)
        if m:
            err, blk, drop = int(m.group(1)), int(m.group(2)), int(m.group(3))
    os.unlink(path)
    tot = err + drop
    return {
        "lock": True,
        "blk": blk,
        "synd_per_blk": err / blk if blk else 0.0,
        "drop_pct": drop / tot * 100 if tot else 0.0,
    }


print("=== 1) 帯域スイープ ===")
rows = []
for f in range(470, 609):
    p = sweep_power(f * 1_000_000)
    rows.append((f, p))
    print(f"  {f} MHz  power={p:6.1f}")

vals = [p for _, p in rows]
thr = (max(vals) + min(vals)) / 2
print(f"\n=== 2) ピーク検出（閾値 {thr:.1f}）===")
peaks = []
for i, (f, p) in enumerate(rows):
    if p > thr and (i == 0 or rows[i - 1][1] <= thr):
        peaks.append(f)
print(f"  ピーク: {peaks}")

print("\n=== 3) 各チャネルの SINR 実測 ===")
print(f"{'freq':>8s} {'lock':>6s} {'synd/blk':>10s} {'drop%':>7s}")
for f in peaks:
    r = measure_channel(f * 1_000_000, 10)
    if r is None:
        print(f"{f:8d}  (キャプチャ失敗)")
    elif not r["lock"]:
        print(f"{f:8d} {'NG':>6s} {'-':>10s} {'-':>7s}")
    else:
        print(f"{f:8d} {'OK':>6s} {r['synd_per_blk']:10.3f} {r['drop_pct']:6.1f}%")
