//! # isdbt-dsp
//!
//! ISDB-T **1セグ（ワンセグ）** 自作復調器のDSPコア。
//!
//! 段構成（信号の流れ）：
//! 1. `①` RF入力（rtl_sdr の生IQ） … [`iq`]
//! 2. `②` **OFDM同期**（CP自己相関でシンボル境界＋小数CFO） … [`sync`]  ← いまここ
//! 3. `③` **チャネル等化**（スキャッタードパイロット） … [`equalize`] / [`pilots`]
//! 4. `④` デマップ＋デインターリーブ（要TMCC） … [`tmcc`]（TMCC復号 ← いまここ）, デマップ未実装
//! 5. `⑤` FEC（Viterbi＋RS(204,188)）                          … 未実装
//! 6. `⑥` TS出力                                                … 未実装
//!
//! 物理層パラメータは [`params`]。一次資料は ARIB STD-B31、
//! 参照実装は gr-isdbt / gr-dvbt（GPL）と williamyang98/DAB-Radio。

pub mod deinterleave;
pub mod demap;
pub mod demod;
pub mod demod_block;
pub mod equalize;
pub mod iq;
pub mod latm;
pub mod params;
pub mod pilots;
pub mod psi;
pub mod stream;
pub mod sync;
pub mod viterbi;
pub mod tmcc;
pub mod rs;
pub mod trace;
pub mod ts;

pub use deinterleave::{
    data_carrier_indices, freq_deinterleave, TimeDeinterleaver, AC_LOCAL_CARRIERS, DATA_CARRIERS,
    FREQ_PERM_MODE3,
};
pub use demap::{qpsk_soft, BitDeinterleaverQpsk};
pub use demod::OfdmDemod;
pub use viterbi::{conv_encode, conv_encode_terminated, depuncture, hard_to_soft, puncture, Viterbi, ViterbiStreaming};
pub use equalize::{
    detect_symbol_phase, equalize, estimate_channel, extract_segment, phase_scores, phase_scores_gr_isdbt,
    sp_coherence, track_symbol_phases, SEGMENT_BIN_OFFSET,
};
pub use params::{GuardInterval, CARRIER_SPACING_HZ, FFT_LEN, SAMPLE_RATE_HZ};
pub use pilots::{SegmentPilots, CENTER_SEGMENT_OFFSET, SEGMENT_CARRIERS};
pub use stream::StreamingDecoder;
pub use sync::{
    estimate_best_guard, estimate_symbol_sync, estimate_sync, fold_metric, metric_curve,
    periodicity_score, SyncEstimate,
};
pub use tmcc::{
    coding_rate_str, dbpsk_bits, equalized_dbpsk_bits, estimate_integer_offset, find_frame_sync,
    integer_offset_score, interleaving_mode3, majority_frame, parse_tmcc, rank_segment_offsets,
    select_segment_offset,
    FrameSync, LayerInfo, Modulation, TmccInfo, SYMBOLS_PER_FRAME, TMCC_LOCAL_CARRIERS,
};
