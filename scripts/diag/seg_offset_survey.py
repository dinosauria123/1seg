#!/usr/bin/env python3
"""各チャンネルの bin offset を実測する。

`ISDBT_SEGOFF` で 308 を固定すると STV（ch21）がロックしない。
`tmcc_probe` を見ると STV は **309** が正解だった（sync 1.000）。
bin offset はチャンネルごとに違う（1 bin = 992 Hz の量子化で、
受信機の LO 周波数誤差によって変わる）。

使い方:
  python3 scripts/diag/seg_offset_survey.py
  # 出力する SEGOFF を /tmp/isdbt_segoff_<freq>.iq ogos に保存する。
"""
import os
import re
import subprocess
import sys

FS = 1015873
CHANNELS = [
    (13, 473_142_857, "NHK教育"),
    (14, 479_142_857, "TVh"),
    (15, 485_142_857, "NHK総合"),
    (19, 509_142_857, "HBC"),
    (21, 521_142_857, "STV"),
    (23, 533_142_857, "HTB"),
    (25, 545_142_857, "UHB"),
]
# tmcc_probe の入力サンプル数。
#
# **20 MB 未満だと偽値が出る**（実測 2026-10-04 03:2x）:
#   ch19 HBC を  8 MB で測ると sync=0.000（偽値）
#   同じ ch19 を 20 MB / 40 MB で測ると sync=1.000 offset=308
# sync=0.000 は「bin offset を決められない」ことを意味するので、
# 値を保存するとそのチャンネルだけロックできなくなる。
# 20 MB ≒ 9.8 秒分。測定 1 チャンネルあたり約 60〜90 秒。
NSAMP = 20_000_000
OUT_DIR = "/tmp"


def probe(ch, freq, nsamp=NSAMP):
    iq = f"/tmp/_so_{ch}.iq"
    try:
        subprocess.run(
            ["timeout", "60", "rtl_sdr", "-f", str(freq), "-s", str(FS),
             "-g", "5", "-n", str(nsamp), iq],
            capture_output=True, timeout=90)
        r = subprocess.run(
            ["./target/release/examples/tmcc_probe", iq, str(FS)],
            capture_output=True, text=True, timeout=300)
        # tmcc_probe は日本語見出しを stderr に出すので両方を探す。
        blob = r.stdout + r.stderr
        # 実出力の書式: "② TMCC候補offset選択: offset=309 (nominal 296)
        #                sync=1.000 combined=1002.022"
        m = re.search(r"offset=(\d+)\s*\(nominal[^)]*\)\s*sync=([\d.]+)", blob)
        if not m:
            sys.stderr.write(f"ch{ch}: offset パターン不一致 "
                             f"(stdout {len(r.stdout)}B, stderr {len(r.stderr)}B)\n")
            sys.stderr.write((r.stdout + r.stderr)[-600:] + "\n")
            return None
        # known は「選択された offset の候補行」にある。選択行は
        # "offset=309 (nominal 296) sync=1.000 combined=1004.348" で
        # known= が無いので、候補行 `candidate offset 309: ... known=X.XXX`
        # から取る（実測 2026-10-04: 全チャンネルの known は 3〜4.3）。
        k = re.search(r"candidate offset " + m.group(1) +
                      r":[^\n]*?known=([\d.]+)", blob)
        known = float(k.group(1)) if k else 0.0
        return int(m.group(1)), float(m.group(2)), known
    finally:
        try:
            os.unlink(iq)
        except OSError:
            pass


def main():
    print(f"{'ch':>4s} {'局名':<10s} {'freq':>12s} {'offset':>7s} "
          f"{'sync':>6s}  {'known':>6s}  判定")
    for ch, freq, name in CHANNELS:
        r = probe(ch, freq)
        if r is None:
            print(f"{ch:4d} {name:<10s} {freq:12d} {'—':>7s} {'—':>6s}  "
                  f"{'—':>6s}  測定失敗")
            continue
        off, sync, known = r
        # sync が 1.000 未満なら偽値の可能性が高いので保存しない。
        # 20 MB 測っても出ないなら、そのチャンネルは再測定する。
        # （実測 2026-10-04: ch25 UHB が一度 sync=0.000 になったが、
        #  個別に 40/60 MB で測ると 309 / sync 1.000 / known 4.348 だった。
        #  一過的な受信状態の可能性があるので、survey の 1 回で
        #  落ちても自動リトライする。）
        if sync < 0.95:
            print(f"{ch:4d} {name:<10s} {freq:12d} {off:7d} {sync:6.3f}  "
                  f"{known:6.2f}  ★ sync 不足・再測定", flush=True)
            for attempt, nsamp in ((2, 40_000_000), (3, 60_000_000)):
                r2 = probe(ch, freq, nsamp)
                if r2 and r2[1] >= 0.95:
                    off, sync, known = r2
                    break
            if sync < 0.95:
                print(f"{ch:4d} {name:<10s} {freq:12d} {'—':>7s} {'—':>6s}  "
                      f"{'—':>6s}  ★ 3 回とも失敗・保存せず", flush=True)
                continue
        path = f"{OUT_DIR}/isdbt_segoff_{freq}"
        with open(path, "w") as fh:
            fh.write(f"{off}\n")
        print(f"{ch:4d} {name:<10s} {freq:12d} {off:7d} {sync:6.3f}  "
              f"{known:6.2f}  {path}", flush=True)


if __name__ == "__main__":
    sys.exit(main())