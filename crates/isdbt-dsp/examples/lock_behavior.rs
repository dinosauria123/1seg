//! ロック済み状態での境界追跡の真の挙動を測る。
//!
//! `stream.rs` の try_lock と同一手順（1M サンプル窓で estimate_symbol_sync）を
//! 実行し、そこから等間隔で追従Localisationしたときの CP peak のメトリクスを記録する。
//! これにより「ロック時は探索が機能し、放置すると劣化する」のか
//! 「そもそも探索が機能しない」のかを切り分ける。

use isdbt_dsp::params::{GuardInterval, FFT_LEN};
use isdbt_dsp::sync::estimate_symbol_sync;
use num_complex::Complex32;

fn main() {
    let path = std::env::args().nth(1).expect("usage: lock_behavior <iq>");
    let raw = std::fs::read(&path).expect("read iq");
    let n = FFT_LEN;
    let gi = GuardInterval::G1_8;
    let l = gi.cp_len(n);
    let sym = n + l;

    let xs: Vec<Complex32> = raw
        .chunks_exact(4)
        .map(|c| {
            let re = i16::from_le_bytes([c[0], c[1]]) as f32 / 32768.0;
            let im = i16::from_le_bytes([c[2], c[3]]) as f32 / 32768.0;
            Complex32::new(re, im)
        })
        .collect();
    println!("samples={} sym={}", xs.len(), sym);

    // --- try_lock と同一手順 ---
    let off = (xs.len() / 10).min(50_000);
    let hi = (off + 1_000_000).min(xs.len());
    let est = estimate_symbol_sync(&xs[off..hi], n, gi);
    match est {
        Some(e) => println!(
            "try_lock窓(1M): d={} metric={:.4} cfo_sub={:.3} guard={:?}",
            e.symbol_start, e.metric, e.cfo_subcarriers, e.guard
        ),
        None => { println!("try_lock窓で None"); return; }
    }

    let sym0 = off + est.as_ref().unwrap().symbol_start;
    // 以降は buf[0] = symbol 0 とした位置から等間隔で探索する（実際の process と等価）
    let mut expected = 0usize; // sym0 基準
    let radius = 8usize;
    let mut pass20 = 0;
    let mut total = 0;
    let mut reported = 0;
    while expected + sym + radius < xs.len() - sym0 {
        let lo = expected + sym0;
        let hi = (lo + sym + radius).min(xs.len());
        let e = estimate_symbol_sync(&xs[lo..hi], n, gi);
        if let Some(e) = e {
            if e.metric > 0.2 { pass20 += 1; }
            if total % 500 == 0 && reported < 20 {
                println!(
                    "  sym#{}: d={} metric={:.4} pass20={}",
                    total, e.symbol_start, e.metric, e.metric > 0.2
                );
                reported += 1;
            }
        }
        // 等間隔（実 process と同じ: start + sym、ただし start は expected になる）
        expected += sym;
        total += 1;
        if total > 6000 { break; }
    }
    println!("=== 結果 ===");
    println!("総 {} シンボル、metric>0.2 通過 {} 回 ({:.1}%)",
        total, pass20, pass20 as f32 / total.max(1) as f32 * 100.0);
}
