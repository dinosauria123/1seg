//! シンボル境界の追跡挙動を診断する。
//!
//! ライブ入力で RS 出力が約 2000 ブロックで恒久停止する原因を突き止めるため、
//! 各シンボルでの CP 探索結果（探索窓内のオフセットとメトリクス）を記録する。
//!
//! 想定される破綻機構: 実 fs (1015873.002204) と指定 fs (1015873) の差により
//! 1 シンボルあたり約 0.0022 サンプルずつ境界がずれる。累積して探索窓
//! （±16 サンプル）を超えると peak を取り違え、境界が 1 ずつずれて復号が破綻する。

use num_complex::Complex32;
use isdbt_dsp::params::GuardInterval;
use isdbt_dsp::sync::estimate_symbol_sync;
use std::time::Instant;

fn main() {
    let path = std::env::args().nth(1).expect("usage: boundary_probe <iq> [fs]");
    let fs_req: f32 = std::env::args().nth(2).map_or(1015873.0, |s| s.parse().unwrap());
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
    eprintln!("サンプル数 {} ({:.1}秒 @ {})", xs.len(), xs.len() as f32 / fs_req, fs_req);

    // まず既知の位置（0）から等間隔で追従し、各シンボルの
    // 「期待位置からのオフセット」と「メトリクス」を記録する。
    let radius = 16usize;
    let mut cur: usize = 0;
    let mut k = 0usize;
    let mut prev_off: i64 = 0;
    let mut max_drift = 0i64;

    let t0 = Instant::now();

    while cur + sym + radius < xs.len() {
        let lo = cur.saturating_sub(radius);
        let hi = (cur + sym + radius).min(xs.len());
        let est = estimate_symbol_sync(&xs[lo..hi], n, gi);
        let (off, metric) = match est {
            Some(e) => ((lo + e.symbol_start) as i64 - cur as i64, e.metric),
            None => {
                eprintln!("k={} estimate=None", k);
                k += 1;
                cur += sym;
                continue;
            }
        };
        if k % 200 == 0 {
            if off.abs() > max_drift.abs() {
                max_drift = off;
            }
            eprintln!(
                "k={:6} cur={:9} off={:+3} metric={:.4} pass20={}",
                k, cur, off, metric,
                metric > 0.2
            );
        }
        prev_off = off;
        // 等間隔で進む（実 fs 差は意図的に無視して、ドリフトがどう蓄積するか見る）
        cur += sym;
        k += 1;
    }
    eprintln!(
        "完了: {} シンボル / {:.1}秒、max 累積ドリフト = {}",
        k,
        t0.elapsed().as_secs_f32(),
        max_drift
    );
}
