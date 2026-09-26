//! demod_block_tracked と demod_one_tracked の等価性検証。
//!
//! block=1, window=32 で demod_block_tracked を動かしたとき、
//! demod_one_tracked と同じ spectrum が得られるか確認する。
//! 両者の CP 探索窓が違う（1シンボル vs 32シンボル）ため spectrum は
//! 多少違うが、ブロック境界が一致するかを見るのが目的。

use isdbt_dsp::params::{GuardInterval, FFT_LEN};
use isdbt_dsp::demod::OfdmDemod;
use num_complex::Complex32;

fn main() {
    let path = std::env::args().nth(1).expect("usage: equiv <iq>");
    let raw = std::fs::read(&path).expect("read iq");
    let xs: Vec<Complex32> = raw
        .chunks_exact(4)
        .map(|c| {
            let re = i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0;
            let im = i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0;
            Complex32::new(re, im)
        })
        .collect();
    let d = OfdmDemod::new(FFT_LEN);
    let gi = GuardInterval::G1_8;
    let sym = FFT_LEN + gi.cp_len(FFT_LEN);

    // try_lock と同じ手順で開始位置を決める
    let off = (xs.len() / 10).min(50_000);
    let hi = (off + 1_000_000).min(xs.len());
    let est = match isdbt_dsp::sync::estimate_symbol_sync(&xs[off..hi], FFT_LEN, gi) {
        Some(e) => e,
        None => { println!("lock 失敗"); return; }
    };
    let sym0 = off + est.symbol_start;
    println!("lock: d={} metric={:.4} sym0={}", est.symbol_start, est.metric, sym0);
    let cfo = est.cfo_subcarriers;

    // 対照: demod_one_tracked を 20 シンボル回す
    let mut cur = sym0;
    let mut one_ok = 0;
    for _ in 0..20 {
        match d.demod_one_tracked(&xs, cur, gi, cfo, 8) {
            Some((_, next)) => { cur = next; one_ok += 1; }
            None => break,
        }
    }
    println!("demod_one_tracked: {} シンボル, 最終cur={}", one_ok, cur);

    // 新方式: demod_block_tracked block=32 window=32
    match d.demod_block_tracked(&xs, sym0, gi, cfo, 32, 32, 16) {
        Some((specs, next)) => {
            let e0: f32 = specs[0].iter().map(|c| c.norm_sqr()).sum();
            println!("demod_block_tracked: {} specs, next-sym0={} (block*sym={}), spec0 エネルギー={:.3e}",
                specs.len(), next - sym0, 32 * sym, e0);
            // エネルギーを 32 個並べて、living な spectrum か確認
            let es: Vec<f32> = specs.iter().map(|s| s.iter().map(|c| c.norm_sqr()).sum()).collect();
            let mn = es.iter().cloned().fold(f32::MAX, f32::min);
            let mx = es.iter().cloned().fold(0.0f32, f32::max);
            println!("spec エネルギー min={:.3e} max={:.3e} ratio={:.2}", mn, mx, mx / mn.max(1e-30));
        }
        None => println!("demod_block_tracked: None (データ不足)"),
    }
}
