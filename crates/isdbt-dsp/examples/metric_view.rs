//! CP 相関メトリクス曲線から、正しい peak 選択規則を確定する診断ツール。
//!
//! ライブ入力で RS 出力か約 2000 ブロックで停止する原因を突き止める。
//! 仮説: `estimate_symbol_sync` は窓内の絶対最大値を取るため、
//! 窓に複数のシンボル周期が入ると無関係な peak を掴む。

use isdbt_dsp::params::GuardInterval;
use isdbt_dsp::sync::{estimate_symbol_sync, metric_curve};
use num_complex::Complex32;

fn main() {
    let path = std::env::args().nth(1).expect("usage: metric_view <iq>");
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

    // 既知の正しい位置（try_lock が見つけた場所 Gardens と仮定して 0 付近から走査）
    // 各シンボル位置で、そこを期待位置としたとき peak がどこに出るか見る。
    println!("sym={} (n={} l={})", sym, n, l);
    for k in 0..6usize {
        let base = k * sym;
        if base + 4 * sym + 32 > xs.len() {
            break;
        }
        // 窓幅 1 シンボル（現行実装）
        let w1 = &xs[base..base + sym + 32];
        let e1 = estimate_symbol_sync(w1, n, gi);
        // 窓幅 3 シンボル
        let w3 = &xs[base..(base + 3 * sym + 32).min(xs.len())];
        let e3 = estimate_symbol_sync(w3, n, gi);

        // 曲線を見て peak の位置を見る
        let curve = metric_curve(&xs[base..(base + 4 * sym).min(xs.len())], n, gi);
        let mut peaks: Vec<(usize, f32)> = curve
            .iter()
            .enumerate()
            .filter(|(_, &v)| v > 0.3)
            .map(|(i, &v)| (i, v))
            .collect();
        peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        let top: Vec<String> = peaks
            .iter()
            .take(6)
            .map(|(i, v)| format!("d={} ({:+} from sym) v={:.3}", i, *i as i64 - sym as i64, v))
            .collect();

        println!(
            "k={} base={} | 1sym窓→ d={:?} v={:.3} | 3sym窓→ d={:?} v={:.3} | peaks: {}",
            k, base,
            e1.as_ref().map(|e| e.symbol_start),
            e1.as_ref().map(|e| e.metric).unwrap_or(0.0),
            e3.as_ref().map(|e| e.symbol_start),
            e3.as_ref().map(|e| e.metric).unwrap_or(0.0),
            top.join(", ")
        );
    }
}
