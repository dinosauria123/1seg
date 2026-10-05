#!/usr/bin/env python3
"""rtl_sdr の AGC が実際に効いているかを検査する。

何を見るか（指標の定義を先に固定する）:
  rtl_sdr の -g 0 はチューナー内部の AGC を ON にし、入力端子からの
  電力を見てゲインを自動調整する。開かれた「1seg が見えない」原因の
  第一候補は「AGC が振幅一杯まで上げて ADC を飽和させている」こと
  （docs/OPERATION.md §6 が -g 0 を唯一正しいとするが、実測は逆の_encoding）。

  判定に使う量（すべて「何 Udall 測っているか」を明示する）:
    [1] sd（標準偏差）      : 信号の広がり。AGC が効いていれば目标 sd に寄る
    [2] 端値率              : 0/255 に貼りつく割合。飽和の直接指標
    [3] Kurtosis (Fisher)   : 3.0=完全ガウス、<2.7=平坦化（clipping）
    [4] 過剰原子            : 観測端値率 − ガウス理論端値率。>0 で原子がある
    [5] 振幅饱和率          : |z| >= 127.5 の割合 を Rayleigh 理論値と比較

AGC が効いているかの判定:
  - ゲイン指示値を上げても sd が比例して増える = AGC OFF（固定ゲイン）
  - ゲイン指示値を上げても sd が頭打ちになる = AGC ON（天井に達した）
  - 同じ信号で -g 0 の sd が異常に大きい = AGC が過大ゲインQwindeている
"""
import math
import sys

import numpy as np


def load_iq(path, seconds=8.0, fs=1015873.0):
    raw = np.fromfile(path, dtype=np.uint8)
    raw = raw[: int(seconds * fs) * 2]
    raw = raw[: (len(raw) // 2) * 2]
    u8 = raw.view(np.uint8).reshape(-1, 2).astype(np.float64)
    i = u8[:, 0]
    q = u8[:, 1]
    return i, q


def stats(i_raw, q_raw, label):
    n = len(i_raw)
    i = i_raw - 127.5
    q = q_raw - 127.5

    i_sd = float(i_raw.std())
    q_sd = float(q_raw.std())

    edge_i = int(((i_raw == 0) | (i_raw == 255)).sum())
    edge_q = int(((q_raw == 0) | (q_raw == 255)).sum())
    edge_pct = 100.0 * (edge_i + edge_q) / (2 * n)

    # kurtosis
    def kurt(v):
        x = v - v.mean()
        m2 = float((x * x).mean())
        m4 = float((x ** 4).mean())
        return m4 / (m2 * m2) if m2 > 0 else 0.0

    ki = kurt(i_raw)
    kq = kurt(q_raw)

    # ガウス理論の端値率（片側、ゼロ点 127.5 から sd 外）
    def gauss_edge(sd):
        z = 127.5 / sd
        return 100.0 * 2.0 * (1.0 - 0.5 * (1.0 + math.erf(z / math.sqrt(2.0))))

    ge = gauss_edge(i_sd)
    excess = edge_pct - ge

    # 複素振幅と Rayleigh 理論
    amp = np.sqrt(i * i + q * q)
    sg = math.hypot(i_sd, q_sd) / math.sqrt(2.0)
    sat_pct = 100.0 * float((amp >= 127.5).mean())
    ray_pct = 100.0 * math.exp(-(127.5 ** 2) / (2.0 * sg * sg))

    print(f"\n=== {label} ===")
    print(f"  sd(I)/sd(Q)      : {i_sd:.2f} / {q_sd:.2f}")
    print(f"  端値率            : {edge_pct:.4f} %  ({edge_i + edge_q:,} / {2*n:,})")
    print(f"  Kurtosis (Fisher): I={ki:.4f}  Q={kq:.4f}")
    print(f"  ガウス理論端値率  : {ge:.4f} %")
    print(f"  過剰原子          : {excess:+.4f} %")
    print(f"  振幅>=127.5       : {sat_pct:.4f} %  (Rayleigh 理論 {ray_pct:.4f} %, "
          f"観測/理論 {sat_pct/ray_pct if ray_pct else 0:.2f} 倍)")

    if ki < 2.7 and kq < 2.7 and excess > 0.05:
        verdict = "**真の飽和**（flat-top）"
    elif excess <= 0.05 and edge_pct > 0.5:
        verdict = "ゲイン過大だが clipping なし"
    elif edge_pct < 0.1:
        verdict = "飽和なし"
    else:
        verdict = "判定保留"
    print(f"  → {verdict}")
    return dict(i_sd=i_sd, q_sd=q_sd, edge_pct=edge_pct, ki=ki, kq=kq,
                excess=excess, sat_pct=sat_pct, ratio=sat_pct / ray_pct if ray_pct else 0)


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    paths = sys.argv[2:] or [
        "/tmp/agc_auto.iq",
        "/tmp/agc_g20.iq",
    ]
    labels = {
        "/tmp/agc_auto.iq": "-g 0（AGC ON, 自動）",
        "/tmp/agc_g20.iq": "-g 20（固定ゲイン）",
    }
    results = {}
    for p in paths:
        try:
            i, q = load_iq(p, seconds)
        except OSError:
            print(f"skip: {p}")
            continue
        results[p] = stats(i, q, labels.get(p, p))

    print("\n=== AGC 判定 ===")
    if len(results) >= 2:
        a = list(results.values())[0]
        b = list(results.values())[1]
        print(f"  -g 0   sd = {a['i_sd']:.2f}")
        print(f"  -g 20  sd = {b['i_sd']:.2f}")
        ratio = a['i_sd'] / b['i_sd'] if b['i_sd'] else 0
        print(f"  比 = {ratio:.2f} 倍")
        if ratio > 3:
            print("  → AGC が必要以上にゲインを上げている（過大ゲイン）")
        elif ratio < 0.3:
            print("  → 固定ゲインのほうが大きい = AGC が効いていない疑い")
        else:
            print("  → 両者同程度 = AGC は動作している")


if __name__ == "__main__":
    main()