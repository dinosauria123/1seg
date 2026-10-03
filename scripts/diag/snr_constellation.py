#!/usr/bin/env python3
"""原子スナップショット（等化出力 soft）から 1seg の SNR を実測する。

母数の定義（最初にここを必ず確認すること）
------------------------------------------------
入力 `/tmp/atom_<blk>.softf32` は **depuncture 後** の soft を f32 で並べた
もので、長さ 60000 = 30000 ビット = 15000 組の (X, Y)。

**パンクチャの消去は X 軸にしか現れない。**
`PUNCTURE_2_3 = [1,1,0,1]` は母符号 (X0, Y0, X1, Y1) のうち **X1 を必ず抜く**。
そのため:

    X 側 30000 個中 15000 個が 0.0 = erasure  → **50.0%**
    Y 側 30000 個中    0 個が 0.0            →  0.0%

実測で確認済み（zero index は必ず `i % 4 == 2`）。

**このため「(x, y) の両方が 0 のときだけ除外」すると 0 組しか落ちず、
X 側の 50% を母数に入れたままになる。** 結果として:

- 四隅の分布が 12.5 / 37.3 / 37.8 / 12.4 % に崩れる（理想 25%）
- E[x²] = 0.331 と E[y²] = 0.664 がちょうど 2 倍の不整合になる

これは信号の問題ではなく**母数の取り方の誤り**。正しい除外条件は
「x == 0.0 or y == 0.0」で、解析母数は **15000 組**（ Ereasure を含む
(depuncture 後) 全 30000 組のうち 15000 組）。

判定基準（この整合でチェックする）
    E[x²] ≒ E[y²] ≒ A² + σ²  が成立すれば母数・重心・σ の全部が整合している。

SNR の定義: 等化出力は 4QAM なので X と Y が独立に ±A + 雑音。
      σ² = クアッドラントごとの重心まわりの分散（信号を平均で除去した分）
      A   = 各重心を反転して原点へleast-squares で寄せた振幅（回転/DC を吸収）
      SNR = A/σ  [dB] = 20·log10(A/σ)
    pe 推定 = Q(A/σ)（4QAM の hard decision 誤り率）

使い方:
  python3 scripts/diag/snr_constellation.py /tmp/atom_400.softf32
"""
import math
import sys


def load(path):
    vals = []
    with open(path) as fh:
        for tok in fh.read().split():
            try:
                vals.append(float(tok))
            except ValueError:
                pass
    return vals


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/atom_400.softf32"
    vals = load(path)
    n_all = len(vals)
    n_bits = n_all - (n_all % 2)
    vals = vals[:n_bits]
    n_pairs_all = n_bits // 2
    if n_pairs_all < 100:
        print(f"母数不足: {n_pairs_all} 組")
        return

    pairs = [(vals[2 * i], vals[2 * i + 1]) for i in range(n_pairs_all)]

    # --- 母数: erasure を除外する。X だけが 50% 欠ける（上記 docstring）。 ---
    er_x = sum(1 for x, _ in pairs if x == 0.0)
    er_y = sum(1 for _, y in pairs if y == 0.0)
    kept = [(x, y) for (x, y) in pairs if x != 0.0 and y != 0.0]
    n = len(kept)

    print(f"入力: {path}")
    print(f"母数: f32 {n_all} 個 → (X,Y) {n_pairs_all} 組（depuncture 後）")
    print(f"      erasure: X 側 {er_x}/{n_pairs_all} = {er_x / n_pairs_all * 100:.1f}%, "
          f"Y 側 {er_y}/{n_pairs_all} = {er_y / n_pairs_all * 100:.1f}%")
    print(f"      → **解析母数 {n} 組**（erasure を除く）")
    print(f"      検証: 理想は X 側 50.0% / Y 側 0.0%（PUNCTURE_2_3 が X1 を抜く）")
    if n < 100:
        print(f"母数不足: {n}")
        return

    # --- 四隅の分布 ---
    quads = {(1, 1): [], (-1, 1): [], (-1, -1): [], (1, -1): []}
    for (x, y) in kept:
        quads[(1 if x > 0 else -1, 1 if y > 0 else -1)].append((x, y))
    print("\n=== 4QAM 四隅の分布（理想 25.00%）===")
    for k in [(1, 1), (-1, 1), (-1, -1), (1, -1)]:
        c = quads[k]
        print(f"  ({k[0]:+d},{k[1]:+d})  n={len(c):6d}  {len(c) / n * 100:5.2f}%")
    spread = max(len(c) for c in quads.values()) / n * 100 - \
        min(len(c) for c in quads.values()) / n * 100
    print(f"  最大−最小 = {spread:.2f} ポイント"
          f"（{'健全' if spread < 3 else '★要調査: 変調/同期/等化を疑う'}）")

    # --- 重心と振幅 A ---
    cents = {}
    for k, c in quads.items():
        cents[k] = (sum(p[0] for p in c) / len(c), sum(p[1] for p in c) / len(c))
    print("\n=== クアッドラント重心（回転・DC バイアスを吸収する）===")
    for k in [(1, 1), (-1, 1), (-1, -1), (1, -1)]:
        cx, cy = cents[k]
        print(f"  ({k[0]:+d},{k[1]:+d}): X {cx:+.4f}  Y {cy:+.4f}  |c| {math.hypot(cx, cy):.4f}")

    # A: min Σ‖c_k − A·S_k·(1,1)‖² の解
    A = sum(cx * k[0] + cy * k[1] for k, (cx, cy) in cents.items()) / (4 * 2.0)

    # σ: クアッドラント重心まわりの分散
    ss = 0.0
    dof = 0
    for k, c in quads.items():
        cx, cy = cents[k]
        ss += sum((p[0] - cx) ** 2 + (p[1] - cy) ** 2 for p in c)
        dof += 2 * (len(c) - 1)
    sigma = math.sqrt(ss / dof)
    print(f"\n  推定振幅 A = {A:.4f}")
    print(f"  推定雑音 σ = {sigma:.4f}  (σ² = {ss / dof:.5f}, 自由度 {dof})")

    # --- 交差検証: E[x²] = A² + σ²（母数が正しければ両軸で一致する）---
    nx = [x for (x, _) in kept]
    ny = [y for (_, y) in kept]
    ex2 = sum(v * v for v in nx) / n
    ey2 = sum(v * v for v in ny) / n
    pred = A * A + ss / dof
    print(f"  交差検証: E[x²]={ex2:.5f}  E[y²]={ey2:.5f}  A²+σ²={pred:.5f}"
          f"  → {'一致 OK' if abs(ex2 - pred) < 0.01 and abs(ey2 - pred) < 0.01 else '★不一致: 母数か重心を疑う'}")

    snr = A / sigma
    snr_db = 20 * math.log10(snr)
    pe = 0.5 * math.erfc(snr / math.sqrt(2.0))
    print("\n=== 結果 ===")
    print(f"  SNR = A/σ = {snr:.4f} = **{snr_db:.2f} dB**")
    print(f"  推定 pe = Q(A/σ) = **{pe:.5f}**")
    print(f"  K=7 自由距離 10 の Viterbi が耐える pe ≲ 1e-4")
    print(f"  → 判定: {'復調可能域' if pe < 1e-3 else '不足（都市雑音・受信環境）'}")

    # --- 残留が白色雑音か ISI か ---
    m = sum(nx) / n
    d = [v - m for v in nx]
    var = sum(v * v for v in d) / n
    print("\n=== 残留自己相関（|r|<0.02 なら多径/ISI でなく白色雑音）===")
    for lag in (1, 2, 3, 4, 5, 8, 12, 16):
        if lag >= n:
            continue
        r = sum(d[i] * d[i + lag] for i in range(n - lag)) / (n * var)
        print(f"  lag {lag:2d}: r = {r:+.4f}")


if __name__ == "__main__":
    main()