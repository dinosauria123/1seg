#!/usr/bin/env python3
"""gr-isdbt（参照実装）で IQ ファイルを復調して「正解の TS」を作る。

自前実装（~/oneseg-rs）の出力が正しいか比較するための ground truth。
`examples/rx_demo.grc` の受信チェーンを UHD ではなく**ファイル入力**で再現する。

    uhd_usrp_source
      -> throttle
      -> [isdbt] ofdm_synchronization
      -> [isdbt] frequency_deinterleaver
      -> [isdbt] time_deinterleaver
      -> [isdbt] symbol_demapper
      -> [isdbt] bit_deinterleaver
      -> [isdbt] viterbi_decoder
      -> [isdbt] byte_deinterleaver
      -> [isdbt] energy_descrambler
      -> [isdbt] reed_solomon_dec_isdbt   (blocks_file_sink)

1seg に合わせるため mode=3 / 1 segment を使う。
gr-isdbt の viterbi rate='2' は internal enum（2/3 に対応）。
"""
import sys, os
from gnuradio import gr, blocks, filter
from gnuradio import isdbt

IQ = sys.argv[1] if len(sys.argv) > 1 else "/home/dino/oneseg-rs/captures/atk3.iq"
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/gr_ref"
MODE = 3
GUARD = "1.0/16"
RATE_IDX = 2  # viterbi rate enum: 2/3

nseg = 1
total_carriers = 2 ** (10 + MODE)
data_carriers = 13 * 96 * 2 ** (MODE - 1)
active_carriers = 13 * 108 * 2 ** (MODE - 1) + 1

tb = gr.top_block("gr_isdbt_ref", catch_exceptions=True)

# 入力は rtl_sdr の u8 IQ（1 サンプル = 2 バイト、値域 -128..127）。
# gr-isdbt は complex float（1 サンプル = 8 バイト）を要求するので変換する。
src_raw = blocks.file_source(gr.sizeof_char * 2, IQ, False)
src = blocks.interleaved_char_to_complex(gr.sizeof_char)
sc  = blocks.multiply_const_cc(1.0 / 127.0)
# grc には low_pass_filter(cutoff=5.8e6/2, samp_rate=8e6*64/63) があるが、
# あれは**フルセグメント**前提の受信機用 ours は 1seg 帯域(約700kHz)だけ
# 録った IQ なので、帯域制限は不要 오히려 1seg を取り逃す。
# そのため LPF を入れず、そのまま同期器へ渡す。
tb.run_until_done = False

# 1seg: oneseg=True, segments_A=1, length_A=204, B/C は使わない
CP = 1.0 / 16          # GI 1/16 (1seg mode3 で使用)
ONESEG = True

# 重要: grc の実チェーンは
#   ofdm_synchronization -> tmcc_decoder -> frequency_deinterleaver
# で、**TMCC デコーダが同期と 周波数デインターリーブの間に入る**。
# TMCC から得た pilot 位相・星座サイズ・区切り位置が 周波数デインタ に
# 渡る。我々の実装に無い部品であり、眾の Pilot 位相取得に相当する。
sync  = isdbt.ofdm_synchronization(MODE, CP, False)
tmcc  = isdbt.tmcc_decoder(MODE, True)
fdint = isdbt.frequency_deinterleaver(ONESEG, MODE)
tdint = isdbt.time_deinterleaver(MODE, 1, 204, 0, 0, 0, 0)
demap = isdbt.symbol_demapper(MODE, 1, 4, 0, 0, 0, 0)   # 4 = QPSK
bint  = isdbt.bit_deinterleaver(MODE, 1, 4)
vit   = isdbt.viterbi_decoder(4, 2)                      # QPSK, rate 2/3
bdeint = isdbt.byte_deinterleaver()
desc  = isdbt.energy_descrambler()
rs    = isdbt.reed_solomon_dec_isdbt()

v2s = blocks.vector_to_stream(gr.sizeof_char, 188)
sink = blocks.file_sink(gr.sizeof_char, OUT + ".ts")

tb.connect(src_raw, src)
tb.connect(src, sc)
tb.connect(sc, sync)
tb.connect(sync, tmcc)
tb.connect(tmcc, fdint)
tb.connect(fdint, tdint)
tb.connect(tdint, demap)
tb.connect(demap, bint)
tb.connect(bint, vit)
tb.connect(vit, bdeint)
tb.connect(bdeint, desc)
tb.connect(desc, rs)
tb.connect(rs, v2s)
tb.connect(v2s, sink)

sz = os.path.getsize(IQ)
n = sz // 4  # complex float32
print(f"input : {IQ} ({sz/1e6:.1f} MB, {n} samples)")
print(f"params: mode={MODE} cp={CP} oneseg={ONESEG}")
print(f"output: {OUT}.ts")

tb.start()
import time
done = 0
t0 = time.time()
while done < n and time.time() - t0 < 900:
    done = tb.head_pos() // 4
    time.sleep(1.0)
tb.stop()
tb.wait()
print(f"consumed {done}/{n} samples, wrote {os.path.getsize(OUT + '.ts') if os.path.exists(OUT + '.ts') else 0} bytes")
