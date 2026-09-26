//! ブロック単位のシンボル復調（CP 探索を長区間で行う）。
//!
//! # なぜこの方式なのか
//!
//! `demod_one_tracked` は 1 シンボル（1152 サンプル）だけを CP 探索窓にして
//! いたが、1seg の CP 自己相関では peak が統計的に立たない。実測:
//!
//! ```text
//! try_lock の 1M サンプル窓:  metric = 0.3005   ← 通る
//! 1 シンボル窓（1152サンプル）: metric = 0.03〜0.17、閾値 0.2 通過率 1.0%
//! ```
//!
//! そのため `demod_one_tracked` はほぼ全シンボルで `.filter(|e| e.metric > 0.2)`
//! を通過できず、`unwrap_or(expected)` にフォールバックしていた。境界が
//! 固定 1152 ずつ進むため、実 fs（1015873.002204）と指定 fs（1015873）の差
//! （≈0.0022 サンプル/シンボル）が累積し、ライブ入力では約 2000 RS ブロック
//! （約 7 秒）で境界が破綻して出力が恒久停止した。
//!
//! # この方式
//!
//! `demod_block_tracked` は `block` 個（既定 64）シンボルをまとめて処理する。
//! 各シンボルで `window` 個（既定 32）シンボルを CP 探索窓にすることで
//! 統計的に peak が立つ（実測 0.30 程度）。peak 位置は block 内で共通なので
//! 一度探索して block 全体を等間隔で復調する。

use crate::demod::OfdmDemod;
use crate::params::GuardInterval;
use num_complex::Complex32;
use std::f32::consts::PI;

impl OfdmDemod {
    /// `block` 個のシンボルをまとめて復調する（CP 探索はblock 単位で1回だけ）。
    ///
    /// 返り値は `(block 個のスペクトル, 次の block の開始位置)`。
    ///
    /// - `r`: 入力 IQ
    /// - `expected`: 次のシンボルの期待開始位置
    /// - `block`: まとめて復調するシンボル数
    /// - `window_syms`: CP 探索窓の長さ（シンボル数。大きいほど統計的に強い）
    /// - `radius`: 探索する最大オフセット（サンプル）
    pub fn demod_block_tracked(
        &self,
        r: &[Complex32],
        expected: usize,
        gi: GuardInterval,
        cfo_subcarriers: f32,
        block: usize,
        window_syms: usize,
        radius: usize,
    ) -> Option<(Vec<Vec<Complex32>>, usize)> {
        let n = self.n;
        let l = gi.cp_len(n);
        let sym = n + l;
        let radius = radius.min(64);
        let window_len = window_syms.max(2) * sym + 2 * radius;
        let need = window_len + (block - 1) * sym;
        if expected + need > r.len() {
            return None;
        }

        // --- CP 探索（window_syms 個分をまとめて相関させる） ---
        let lo = expected.saturating_sub(radius);
        let hi = (expected + window_len).min(r.len());
        let start = crate::sync::estimate_symbol_sync(&r[lo..hi], n, gi)
            .map(|e| lo + e.symbol_start)
            .unwrap_or(expected);
        if start + block * sym > r.len() {
            return None;
        }

        // --- block 個分を等間隔で復調 ---
        let half = n / 2;
        let scale = 1.0 / (n as f32).sqrt();
        let w = Complex32::from_polar(1.0, -2.0 * PI * cfo_subcarriers / (n as f32));
        let mut out = Vec::with_capacity(block);
        let mut buf = vec![Complex32::new(0.0, 0.0); n];
        for s in 0..block {
            let base = start + l + s * sym;
            let mut ph = Complex32::new(1.0, 0.0);
            for k in 0..n {
                buf[k] = r[base + k] * ph;
                ph *= w;
            }
            self.fft.process(&mut buf);
            let mut spec = vec![Complex32::new(0.0, 0.0); n];
            for k in 0..n {
                spec[(k + half) % n] = buf[k] * scale;
            }
            out.push(spec);
        }
        Some((out, start + block * sym))
    }
}
