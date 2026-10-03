#!/usr/bin/env python3
"""bin offset の再現性を確認する。

`ISDBT_SEGOFF` に保存する bin offset は、測定ごとに 1 bin ぶれることが
ある（実測 2026-10-04: ch14 TVh が 8 MB 測定で 308、20 MB 測定で 307）。
1 bin = 992 Hz なので、受信機の LO 周波数誤差か SNR による選択の摇動。

**この摇動がそのままロック失敗になる。** 固定した値が 1 bin ずれていれば
`demod_and_align` が None を返して TS 0 バイトになる（実測: STV に 308 を
渡して `ロック失敗: navail=4000 specs=4000 phase0=3`）。

survey で 1 回だけ測って保存する方式は、揺れる.bin offset を固定して
 legislativ 壊す。 Inclusive な修正案は 2 つ:

  A) 揺れても正しい bin に집 中する投票（複数回測して最頻値を取る）
  B) bin offset を固定せず、探索する（ただし 63 秒かかる）

votes の実装: 3 回測して sync=1.000 の offset のうち最頻値。全て異なれば
「固定しない」が安全なので、`live_play_direct.sh` が自動探索にフォールバック
する。

使い方:
  python3 scripts/diag/segoff_stability.py [チャンネル数]
"""
import os
import re
import subprocess
import sys
from collections import Counter

FS = 1015873
OUT_DIR = "/tmp"
NSAMP = 20_000_000
REPEATS = 3

CHANNELS = [
    (13, 473_142_857, "NHK教育"),
    (14, 479_142_857, "TVh"),
    (15, 485_142_857, "NHK総合"),
    (19, 509_142_857, "HBC"),
    (21, 521_142_857, "STV"),
    (23, 533_142_857, "HTB"),
    (25, 545_142_857, "UHB"),
]


def probe(ch, freq, nsamp=NSAMP):
    iq = f"/tmp/_st_{ch}.iq"
    try:
        subprocess.run(
            ["timeout", "60", "rtl_sdr", "-f", str(freq), "-s", str(FS),
             "-g", "5", "-n", str(nsamp), iq],
            capture_output=True, timeout=90)
        r = subprocess.run(
            ["./target/release/examples/tmcc_probe", iq, str(FS)],
            capture_output=True, text=True, timeout=300)
        blob = r.stdout + r.stderr
        m = re.search(r"offset=(\d+)\s*\(nominal[^)]*\)\s*sync=([\d.]+)",
                      blob)
        if not m:
            return None
        return int(m.group(1)), float(m.group(2))
    finally:
        try:
            os.unlink(iq)
        except OSError:
            pass


def main():
    nrep = int(sys.argv[1]) if len(sys.argv) > 1 else REPEATS
    print(f"{'ch':>4s} {'局名':<10s} {nrep} 回の測定            "
          f"{'最頻':>5s}  判定")
    for ch, freq, name in CHANNELS:
        results = []
        for i in range(nrep):
            r = probe(ch, freq)
            if r and r[1] >= 0.95:
                results.append(r[0])
            else:
                results.append(None)
        valid = [x for x in results if x is not None]
        if not valid:
            print(f"{ch:4d} {name:<10s} {results}  —  全部失敗")
            continue
        cnt = Counter(valid)
        mode, n = cnt.most_common(1)[0]
        stable = len(cnt) == 1
        verdict = "安定" if stable else f"★不安定（{len(cnt)} 種類）"
        print(f"{ch:4d} {name:<10s} {results}  {mode:5d}  {verdict}")
        if stable:
            path = f"{OUT_DIR}/isdbt_segoff_{freq}"
            with open(path, "w") as fh:
                fh.write(f"{mode}\n")


if __name__ == "__main__":
    main()