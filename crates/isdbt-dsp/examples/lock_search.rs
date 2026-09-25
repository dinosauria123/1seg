//! TMCC 同期語一致率を通しで走査し、復調フリー 参数（symbol offset, search_radius,
//! 整数CFO, セグメント bin offset）で Live 放送がロックできる条件を数値的に探す。
//! 既存 stream_decode / tmcc_probe に 対して総当たりし、最適条件を報告する。
//!
//! ```bash
//! cargo run --release -p isdbt-dsp --example lock_search -- cap.iq 1015873
//! ```

use isdbt_dsp::demod::OfdmDemod;
use isdbt_dsp::equalize::SEGMENT_BIN_OFFSET;
use isdbt_dsp::iq::u8_iq_to_complex;
use isdbt_dsp::params::{FFT_LEN, GuardInterval};
use isdbt_dsp::tmcc::{equalized_dbpsk_bits, find_frame_sync_joint};
use num_complex::Complex32;
use std::env;

fn score_for(specs: &[Vec<Complex32>], off: usize) -> (usize, usize, bool, bool) {
    if off + 432 > FFT_LEN {
        return (0, 1, false, false);
    }
    let segs: Vec<Vec<Complex32>> = specs
        .iter()
        .map(|sp| sp[off..off + 432].to_vec())
        .collect();
    let bits = equalized_dbpsk_bits(&segs);
    match find_frame_sync_joint(&bits) {
        Some(s) => (s.matched, s.total, s.alternates, s.is_true_lock()),
        None => (0, 1, false, false),
    }
}

fn main() {
    let mut a = env::args().skip(1);
    let path = a.next().expect("usage: lock_search <cap.iq> <fs_hz> [nsym]");
    let fs: f64 = a.next().expect("fs").parse().unwrap();
    let nsym: usize = a.next().map(|s| s.parse().unwrap()).unwrap_or(2000);

    let bytes = std::fs::read(&path).expect("IQ");
    let mut s = u8_iq_to_complex(&bytes);
    let mean: Complex32 = s.iter().sum::<Complex32>() / s.len() as f32;
    for v in s.iter_mut() {
        *v -= mean;
    }
    let base = (s.len() / 10).min(50_000);
    let sig = &s[base..];

    let est =
        isdbt_dsp::sync::estimate_symbol_sync(sig, FFT_LEN, GuardInterval::G1_8).expect("sync");
    let demod = OfdmDemod::new(FFT_LEN);

    // 対象窓を 1seg マス内 scan
    let off_range = SEGMENT_BIN_OFFSET.saturating_sub(20)..SEGMENT_BIN_OFFSET + 21;

    let mut best = (0usize, 1usize, 0.0f32, false);
    let mut any_true_lock = false;
    for &rad in &[4usize, 8, 16] {
        let specs = demod.demod_stream_tracked(
            sig,
            est.symbol_start,
            est.guard,
            est.cfo_subcarriers,
            nsym,
            rad,
        );
        if specs.is_empty() {
            continue;
        }
        let mut local = (0usize, 1usize, 0.0f32, false);
        for off in off_range.clone() {
            if off + 432 > FFT_LEN {
                continue;
            }
            let (m, t, _alt, true_lock) = score_for(&specs, off);
            let r = m as f32 / t as f32;
            if r > local.2 {
                local = (m, t, r, true_lock);
            }
            if r > best.2 {
                best = (m, t, r, true_lock);
            }
            if true_lock {
                any_true_lock = true;
            }
        }
        let (m, t, r, tl) = local;
        println!(
            "radius={:2}  best_offset -> {}/{} ({:.1}%)  真ロック={}",
            rad, m, t, 100.0 * r, if tl { "YES" } else { "NO" }
        );
    }
    let _ = best;
    if any_true_lock {
        println!("=== 結果: 偽ロックではない本物の TMCC ロックあり");
    } else {
        println!(
            "=== 結果: ❌ 全パラメータで偽ロック（同期語16bitが一貫しない＝入力に1seg信号なし）"
        );
    }
}
