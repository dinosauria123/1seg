#!/usr/bin/env python3
"""PRBS リセット周期（64 ブロック）と実際のフレーム構造の整合を検証する。

仮説: 1 OFDM フレームの kept bit 数が 64 ブロック分に**正確に一致しない**ため、
PRBS リセットの位置が 1 ブロックずつずれていき、累積する。
「冒頭は完璧、後半で崩壊」と整合する。
"""
import math

SYMS = 204
CARRIERS = 384
BITS_PER_CARRIER = 2
K = 188
TSP = 204
PUNCTURE_KEPT = 2 / 3   # PUNCTURE_2_3 = [1,1,0,1] で 3/4 残るが…
RESET_PERIOD = 64

raw_bits = SYMS * CARRIERS * BITS_PER_CARRIER
print(f"1 フレーム = {SYMS} シンボル × {CARRIERS} carrier × {BITS_PER_CARRIER} bit")
print(f"  = {raw_bits} bit（puncture 前）")
print()
print(f"PUNCTURE_2_3 = [1,1,0,1] → 4 中 3 保持 = 3/4")
kept34 = raw_bits * 3 / 4
print(f"  puncture 3/4 後 = {kept34:.0f} bit")
print()
blocks_needed = kept34 / (K * 8)
print(f"1 フレーム分 = {blocks_needed:.3f} ブロック")
print(f"RESET_PERIOD（実装値）= {RESET_PERIOD}")
print(f"  → 差 = {blocks_needed - RESET_PERIOD:+.3f} ブロック")
print()

# depuncture 側の実際の kept 数
PUNCTURE_2_3 = [1, 1, 0, 1]
print(f"PUNCTURE_2_3 = {PUNCTURE_2_3}")
kept_count = sum(PUNCTURE_2_3)
period = len(PUNCTURE_2_3)
print(f"  kept = {kept_count}/{period} = {kept_count/period:.4f}")
print()

# 実装: depu_pos は 1 bit ごとに増える
# kept bit 数 = floor(raw_bits * 3/4) だが、depu_pos は modulo で回す
exact_kept = int(raw_bits * kept_count / period)
print(f"実装の kept bit 数 = {raw_bits} × {kept_count}/{period} = {exact_kept}")
print(f"1 フレーム = {exact_kept} bit")
print(f"64 ブロック × 188 B × 8 bit = {RESET_PERIOD * K * 8} bit")
print(f"  差 = {exact_kept - RESET_PERIOD * K * 8:+d} bit")
print()

# depu_pos の mod 4 位置がフレーム境界でリセットされるか
r = exact_kept % period
print(f"exact_kept mod {period} = {r}")
print(f"→ フレーム末尾で depu_pos は {period - r} 個分「余る」")
print()

# 1 ブロックあたり kept bit
blk_kept = exact_kept / RESET_PERIOD
print(f"1 ブロックあたり kept bit = {exact_kept} / {RESET_PERIOD} = {blk_kept:.2f}")
print(f"（K×8 = {K*8} のはず）")
print()
if abs(blk_kept - K * 8) > 1:
    print("!!! 1 フレームの kept bit 数が 64 ブロック分に一致しない !!!")
    print(f"    ずれ = {blk_kept - K*8:+.2f} bit/ブロック")
    print(f"    累積（{RESET_PERIOD} ブロックで）= {(blk_kept - K*8) * RESET_PERIOD:+.1f} bit")
else:
    print("整合している")
