//! 隣接 OFDM シンボル間の相互相関（ISDB-T 標準の同期方式）で境界を推定する実験。
//!
//! 目的: CP 自己相関（`estimate_symbol_sync`）ではメトリクスが 0.05-0.21 に
//! 留まり 0.2 閾値を通らない（実測）ため、CP 方式は使えない。
//! ISDB-T では連続する pilot 構成が既知なので、隣接シンボル間の
//! 相互相関でガード区間の位置を推定するのが標準的。
//!
//! ここではまず、CP 自己相関が「ロック済み位置（既知の正しい境界）」を
//! 起点にした場合はど Policies の peak を出せるか確認し、
//! 併せて隣接シンボル相関のメトリクスを計算して比較する。

use isdbt_dsp::params::GuardInterval;
use isdbt_dsp::sync::{estimate_symbol_sync, metric_curve};
use num_complex::Complex32;

fn main() {
    let path = std::env::args().nth(1).expect("usage: cross_view <iq>");
    let raw = std::fs::read(&path).expect("read iq");
    let n = 1024usize;
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
    println!("sym={} samples={}", sym, xs.len());

    // 1) 大きな窓で CP 自己相関を取り、シンボル周期で peak が立つ位置を
    //    正しい境界の候補として列挙する。
    let big = 40 * sym;
    if big > xs.len() {
        return;
    }
    let curve = metric_curve(&xs[0..big], n, gi);
    let mut peaks: Vec<(usize, f32)> = curve
        .iter()
        .enumerate()
        .map(|(i, &v)| (i, v))
        .filter(|(_, v)| *v > 0.25)
        .collect();
    peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    println!("=== CP自己相関 peak (v>0.25), 上位20 ===");
    for (i, v) in peaks.iter().take(20) {
        let m = *i as i64 % sym as i64;
        println!("  d={:5} v={:.3}  (sym周期内 offset={})", i, v, m);
    }
    println!("  peak総数(>0.25) = {}", peaks.len());

    // 2) 隣接シンボル相互相関: シンボル d と d+1 の有効部（CP除く）の相関を
    //    全オフセットで計算し、ガード位置を推定する。
    //    有効部は時間-domain のシンボルなので、この相関は pilots ではなく
    //    データ依存で peak が弱いが、CP 自己相関の d の alternatives となる。
    println!("=== 隣接シンボル相互相関（d 走査） ===");
    let mut best = (0usize, 0.0f32);
    for d in 0..=(2 * sym) {
        // r[d..d+n] と r[d+sym..d+sym+n] の正規化相関
        if d + sym + n >= xs.len() {
            break;
        }
        let mut num = Complex32::new(0.0, 0.0);
        let mut den = 0.0f32;
        for j in 0..n {
            let a = xs[d + j];
            let b = xs[d + sym + j];
            num += a * b.conj();
            den += (a.norm_sqr() + b.norm_sqr()) * 0.5;
        }
        let m = if den > 0.0 { num.norm() / den } else { 0.0 };
        if m > best.1 {
            best = (d, m);
        }
        if d % 200 == 0 {
            println!("  d={:5} m={:.4}", d, m);
        }
    }
    println!("  adjacent best: d={} m={:.4}", best.0, best.1);

    // 3) 参考: 1シンボル窓の estimate_symbol_sync（現行方式）
    let e = estimate_symbol_sync(&xs[0..sym + 32], n, gi);
    println!("=== 1sym窓 estimate_symbol_sync: d={:?} v={:.4} ===",
        e.as_ref().map(|x| x.symbol_start),
        e.as_ref().map(|x| x.metric).unwrap_or(0.0));
}
