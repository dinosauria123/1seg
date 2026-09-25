//! TMCC の復調ビット列と、フレーム位相ごとの同期語一致ビット位置をダンプする。
//! 本物の TMCC なら同期語は固定なので、チャンネルを変えても「一致した16bit中の
//! どのビット位置が一致するか」が同じ（あるいは完全に一致/不一致）になる。
//! 偽ロック（ノイズ由来）なら一致位置がチャンネル間でばらつく。
//!
//! ```bash
//! cargo run --release -p isdbt-dsp --example tmcc_bitdump -- cap.iq 1015873 [nsym] [out]
//! ```

use isdbt_dsp::demod::OfdmDemod;
use isdbt_dsp::equalize::SEGMENT_BIN_OFFSET;
use isdbt_dsp::iq::u8_iq_to_complex;
use isdbt_dsp::params::{FFT_LEN, GuardInterval};
use isdbt_dsp::tmcc::{
    equalized_dbpsk_bits, find_frame_sync_joint, match_count_public, SYMBOLS_PER_FRAME,
    SYNC_EVEN, SYNC_ODD, SYNC_SIZE,
};
use num_complex::Complex32;
use std::env;

fn main() {
    let mut a = env::args().skip(1);
    let path = a.next().expect("usage: tmcc_bitdump <cap.iq> <fs_hz> [nsym]");
    let _fs: f64 = a.next().expect("fs").parse().unwrap();
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
    let specs = demod.demod_stream_tracked(
        sig,
        est.symbol_start,
        est.guard,
        est.cfo_subcarriers,
        nsym,
        8,
    );

    // 最佳 offset を探す（同期語一致が最大）
    let mut best_off = SEGMENT_BIN_OFFSET;
    let mut best_rate = -1.0f32;
    for off in (SEGMENT_BIN_OFFSET.saturating_sub(20)..SEGMENT_BIN_OFFSET + 21) {
        if off + 432 > FFT_LEN {
            continue;
        }
        let segs: Vec<Vec<Complex32>> = specs
            .iter()
            .map(|sp| sp[off..off + 432].to_vec())
            .collect();
        let bits = equalized_dbpsk_bits(&segs);
        let r = find_frame_sync_joint(&bits)
            .map(|s| s.matched as f32 / s.total as f32)
            .unwrap_or(0.0);
        if r > best_rate {
            best_rate = r;
            best_off = off;
        }
    }

    let segs: Vec<Vec<Complex32>> = specs
        .iter()
        .map(|sp| sp[best_off..best_off + 432].to_vec())
        .collect();
    let bits = equalized_dbpsk_bits(&segs);
    let fs = find_frame_sync_joint(&bits).expect("frame sync");

    println!("file={} offset={} sync={}/{} ({:.1}%) phase={} alternates={}", path, best_off, fs.matched, fs.total, 100.0*fs.matched as f32/fs.total as f32, fs.phase, fs.alternates);

    // 同期語一致ビット位置の集計：各フレーム・各ビット位置で何回一致したか
    let mut hit = vec![0usize; SYNC_SIZE];
    let mut n = 0usize;
    for f in 0..fs.n_frames {
        let start = fs.phase + f * SYMBOLS_PER_FRAME;
        if start + 1 + SYNC_SIZE > bits.len() {
            break;
        }
        let w = &bits[start + 1..start + 1 + SYNC_SIZE];
        let me = match_count_public(w, &SYNC_EVEN);
        let mo = match_count_public(w, &SYNC_ODD);
        let pat = if mo > me { &SYNC_ODD } else { &SYNC_EVEN };
        for i in 0..SYNC_SIZE {
            if w[i] == pat[i] {
                hit[i] += 1;
            }
        }
        n += 1;
    }
    print!("match_hist=");
    for h in &hit {
        print!("{:.2},", *h as f32 / n.max(1) as f32);
    }
    println!();
}
