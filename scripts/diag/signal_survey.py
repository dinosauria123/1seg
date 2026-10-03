#!/usr/bin/env python3
"""1seg 受信信号が本当に弱いかを、独立に検証する。

判定の論点: 「RS復号率が悪い」→「信号が弱い」は仮説であって証明ではない。
このスクリプトは仮説と区別するために、以下の独立な観測を行う。

1. 帯域電力: 470-608 MHz を 1 MHz 刻みで掃引し平均/最大電力を測る。
   → 放送チャネルごとのピーク検出と、ピーク間のノイズ床（送信インフラ）を得る。
2. 忙yming/飽和チェック: IQ の 0 と 255 の出現率。
   → 8bit ADC の飽和が起きていれば「信号は強いのに復調が悪い」另一种説明になる。
3. ゲイン特性: 固定ゲインを段階的に変えて平均電力を測る。
   → 自動ゲインが上限に張り付いているか（＝ scavenger が弱信号に反応して
   ゲインを最大まで上げているか）を推定する。
4. チャネル別 1seg 復調: 検出した各ピークで TMCC ロックと RS 復号率を測る。
   → 全チャネルで同じなら受信系（アンテナ/ノイズ）/全帯域の都市雑音。
     特定チャネルだけ悪いならそのチャネルの干渉。

注意: rtl_sdr は USB デバイスを排他占有するので、ライブ再生を停止してから実行する。
"""
import array
import os
import pathlib
import re
import subprocess
import sys

FS = 1015873
RATE = 1_015_873
SCAN_DIR = pathlib.Path(os.path.expanduser("~/.hermes/cache/scratch"))
PROBE = "./target/release/examples/tmcc_probe"
DEC = "./target/release/examples/stream_decode"


def capture(freq_hz, sec=0.4, gain=0):
    cmd = ["timeout", str(int(sec) + 2), "rtl_sdr", "-f", str(freq_hz),
           "-s", str(FS), "-g", str(gain), "-"]
    try:
        p = subprocess.run(cmd, capture_output=True, timeout=sec + 12)
    except subprocess.TimeoutExpired:
        return b""
    return p.stdout


def stats(raw):
    a = array.array("B", raw)
    n = len(a)
    if n < 1000:
        return None
    s = 0.0
    mx = 0
    lo = 255
    at_lo = at_hi = 0
    for v in a:
        x = v - 127.5
        s += x * x
        if x > mx:
            mx = x
        if x < lo:
            lo = x
    # 飽和の目安: 端の 2 値ISSN を数える
    for v in a:
        if v <= 2:
            at_lo += 1
        elif v >= 253:
            at_hi += 1
    rms = (s / n) ** 0.5
    return {
        "n": n,
        "rms": rms,
        "peak": max(mx, -lo),
        "clip_lo": at_lo / n * 100.0,
        "clip_hi": at_hi / n * 100.0,
        "crf": 20 * (10 ** ((max(mx, -lo)) / 20 / 1.0)) if rms > 0 else 0,
    }


def db(x):
    return 20 * __import__("math").log10(x) if x > 0 else -999.0


def step1_band():
    print("=== 1) 470-608 MHz 帯域電力（自動ゲイン, 0.4 s/ch）===")
    rows = []
    for f in range(470, 609):
        raw = capture(f * 1_000_000, 0.4)
        st = stats(raw)
        if st is None:
            continue
        rows.append((f, st))
        if f % 10 == 0 or f in (485,):
            print(f"  {f} MHz  rms={st['rms']:6.2f}  peak={st['peak']:6.1f}  "
                  f"clip_lo/hi={st['clip_lo']:.3f}/{st['clip_hi']:.3f}%")
    return rows


def step3_gain(freq_hz):
    print(f"\n=== 3) ゲイン特性（{freq_hz/1e6:.0f} MHz, 1.0 s ごと）===")
    print("    gain  rms    peak   dBFS(rms)")
    prev = None
    for g in (0, 10, 15, 20, 25, 28, 30, 32, 35, 40, 45, 49):
        raw = capture(freq_hz, 1.0, gain=g)
        st = stats(raw)
        if st is None:
            print(f"    {g:3d}   (取得失敗)")
            continue
        d = 20 * __import__("math").log10(st["rms"] / 127.5)
        delta = "" if prev is None else f"  (+{st['rms']/prev:.2f}x)"
        print(f"    {g:3d}  {st['rms']:6.2f} {st['peak']:6.1f} {d:7.1f}{delta}")
        prev = st["rms"]


def step4_demod(freq_hz, sec=8):
    raw = capture(freq_hz, sec)
    if len(raw) < 2_000_000:
        return None
    path = os.path.join(SCAN_DIR, f"probe_{int(freq_hz)}.iq")
    with open(path, "wb") as fh:
        fh.write(raw)
    pr = subprocess.run([PROBE, path, str(FS)], capture_output=True,
                        text=True, timeout=600)
    locked = "真のロックか          : YES" in pr.stdout
    known = None
    m = re.search(r"known[^\d]*([\d.]+)", pr.stdout)
    if m:
        known = float(m.group(1))
    if not locked:
        os.unlink(path)
        return {"lock": False, "known": known}
    env = dict(os.environ, ISDBT_DEBUG="1")
    r = subprocess.run([DEC, path, os.devnull, "--live"],
                       capture_output=True, text=True, env=env, timeout=1200)
    err = blk = drop = 0
    off = None
    for ln in r.stderr.splitlines():
        m2 = re.search(r"rs_err=(\d+)/(\d+)blk rs_drop=(\d+)", ln)
        if m2:
            err, blk, drop = (int(m2.group(1)), int(m2.group(2)),
                              int(m2.group(3)))
        m3 = re.search(r"bin offset=(\d+)", ln)
        if m3:
            off = int(m3.group(1))
    os.unlink(path)
    rate = 1.0 - (err / blk if blk else 1.0)
    return {"lock": True, "known": known, "rs": rate, "blk": blk,
            "drop": drop, "off": off}


if __name__ == "__main__":
    SCAN_DIR.mkdir(parents=True, exist_ok=True)
    rows = step1_band()
    if not rows:
        print("帯域掃引に失敗（rtl_sdr が他のプロセスにgrabされている?)")
        sys.exit(1)
    vals = [st["rms"] for _, st in rows]
    thr = (max(vals) + min(vals)) / 2
    peaks = [f for i, (f, st) in enumerate(rows)
             if st["rms"] > thr and (i == 0 or rows[i - 1][1]["rms"] <= thr)]
    floor = sorted(vals)[:max(1, len(vals) // 5)]
    noise = sum(floor) / len(floor)
    print(f"\n=== 2) ピーク検出 ===")
    print(f"  検出ピーク: {peaks} MHz")
    print(f"  ノイズ床（最小20%の平均）: rms={noise:.2f}  ({db(noise):.1f} dBFS)")
    for f in peaks:
        st = dict(rows)[f]
        print(f"  {f} MHz: rms={st['rms']:6.2f} ({db(st['rms']):5.1f} dBFS)  "
              f"ノイズ床比 +{20*__import__('math').log10(st['rms']/noise):4.1f} dB  "
              f"clip_lo/hi={st['clip_lo']:.3f}/{st['clip_hi']:.3f}%")

    step3_gain(485_142_857)

    print(f"\n=== 4) チャネル別 1seg 復調（{8} s キャプチャ）===")
    print(f"{'freq':>8s} {'lock':>6s} {'known':>7s} {'RS復号率':>9s} {'offset':>7s}")
    for f in peaks:
        r = step4_demod(f * 1_000_000)
        if r is None:
            print(f"{f:8d} {'失敗':>6s}")
        elif not r["lock"]:
            print(f"{f:8d} {'NG':>6s} {str(r['known']):>7s}")
        else:
            print(f"{f:8d} {'OK':>6s} {str(r['known']):>7s} {r['rs']:9.3f} "
                  f"{str(r['off']):>7s}")