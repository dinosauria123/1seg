//! ②→③ の橋渡し：OFDM復調（小数CFO補正 → CPスキップ → FFT → fftshift）。
//!
//! 同期 [`crate::sync`] で得た `symbol_start` / `guard` / `cfo` を使い、
//! 各OFDMシンボルから `fft_len` 本の副搬送波（複素）を取り出す。
//! 出力は fftshift 済み（DC＝carrier0 が中央）。中央付近の約432本が
//! 1セグメントのキャリア。チャネル等化はこの後段（③）。

use crate::params::GuardInterval;
use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};
use std::f32::consts::PI;
use std::sync::Arc;

/// 固定FFT長のOFDM復調器（プランを使い回す）。
pub struct OfdmDemod {
    pub(crate) fft: Arc<dyn Fft<f32>>,
    pub(crate) n: usize,
}

impl OfdmDemod {
    pub fn new(fft_len: usize) -> Self {
        let mut planner = FftPlanner::<f32>::new();
        Self {
            fft: planner.plan_fft_forward(fft_len),
            n: fft_len,
        }
    }

    pub fn fft_len(&self) -> usize {
        self.n
    }

    /// `symbol_start` 以降の各シンボルを復調し、副搬送波スペクトル列を返す。
    ///
    /// - `cfo_subcarriers`：[`crate::sync`] の小数CFO（±0.5）。時間領域で
    ///   `exp(-j2π ε m / N)`（m は絶対サンプルindex）として連続補正する。
    /// - CPは捨て、有効シンボル `N` 点だけをFFTする。
    /// - 返り値の各 `Vec` は長さ `N`、**fftshift済み**（index N/2 が DC）。
    pub fn demod_stream(
        &self,
        r: &[Complex32],
        symbol_start: usize,
        gi: GuardInterval,
        cfo_subcarriers: f32,
        max_symbols: usize,
    ) -> Vec<Vec<Complex32>> {
        let n = self.n;
        let l = gi.cp_len(n);
        let sym = n + l;
        let half = n / 2;
        let scale = 1.0 / (n as f32).sqrt();

        let mut out = Vec::new();
        let mut buf = vec![Complex32::new(0.0, 0.0); n];
        let mut s = symbol_start;
        while out.len() < max_symbols && s + sym <= r.len() {
            let base = s + l; // CPをスキップした有効シンボル先頭
            for k in 0..n {
                let m = base + k;
                let ph = -2.0 * PI * cfo_subcarriers * (m as f32) / (n as f32);
                buf[k] = r[m] * Complex32::from_polar(1.0, ph);
            }
            self.fft.process(&mut buf);

            // fftshift（DCを中央 index N/2 へ）＋ 正規化
            let mut spec = vec![Complex32::new(0.0, 0.0); n];
            for k in 0..n {
                spec[(k + half) % n] = buf[k] * scale;
            }
            out.push(spec);
            s += sym;
        }
        out
    }

    /// シンボルごとにCP相関で境界を微調整する復調。
    /// サンプリング周波数誤差による長時間のtiming driftを追跡する。
    pub fn demod_stream_tracked(
        &self,
        r: &[Complex32],
        symbol_start: usize,
        gi: GuardInterval,
        cfo_subcarriers: f32,
        max_symbols: usize,
        search_radius: usize,
    ) -> Vec<Vec<Complex32>> {
        let n = self.n;
        let l = gi.cp_len(n);
        let sym = n + l;
        let radius = search_radius.min(16);
        let mut out = Vec::new();
        let mut expected = symbol_start;
        let mut buf = vec![Complex32::new(0.0, 0.0); n];
        while out.len() < max_symbols && expected + sym <= r.len() {
            let lo = expected.saturating_sub(radius);
            let hi = (expected + sym + radius).min(r.len());
            if hi <= lo + sym || hi - lo < n + l {
                break;
            }
            let window = &r[lo..hi];
            let refined = crate::sync::estimate_symbol_sync(window, n, gi);
            let start = refined
                .filter(|e| e.metric > 0.2)
                .map(|e| lo + e.symbol_start)
                .unwrap_or(expected);
            if start + sym > r.len() { break; }
            let base = start + l;
            for k in 0..n {
                let m = base + k;
                let ph = -2.0 * PI * cfo_subcarriers * (m as f32) / (n as f32);
                buf[k] = r[m] * Complex32::from_polar(1.0, ph);
            }
            self.fft.process(&mut buf);
            let half = n / 2;
            let scale = 1.0 / (n as f32).sqrt();
            let mut spec = vec![Complex32::new(0.0, 0.0); n];
            for k in 0..n { spec[(k + half) % n] = buf[k] * scale; }
            out.push(spec);
            expected = start + sym;
        }
        out
    }

    /// 長窓で CP 探索し、**期待位置に最も近い peak** を境界として返す。
    ///
    /// # なぜこれが正しいか（実測に基づく）
    ///
    /// 1seg の CP 自己相関は短窓では統計的に peak が立たない:
    ///
    /// | 探索窓 | メトリクス |
    /// |--------|-----------|
    /// | 1 シンボル (1152 サンプル) | 0.03-0.17（0.2 通過率 1.0%）|
    /// | 1M サンプル | 0.90 前後（**全シンボル境界に peak が立つ**）|
    ///
    /// 長窓は強いが、絶対最大値を取ると別のシンボル境界を掴む（実測で
    /// Δ = 16〜21 シンボル）。しかし漂移（実 fs 1015873.002204 と指定
    /// 1015873 の差 ≈0.0022 サンプル/シンボル）は 800 シンボルで 1.9
    /// サンプルにしかならないので、**期待位置の近傍にある peak が真の境界**。
    ///
    /// 返り値は `(境界位置, メトリクス)`。近傍に peak がなければ `None`
    /// （呼び出し側は直前の境界を維持して等間隔で進める）。
    pub fn reacquire_boundary(
        &self,
        r: &[Complex32],
        expected: usize,
        gi: GuardInterval,
        window: usize,
    ) -> Option<(usize, f32)> {
        let n = self.n;
        let l = gi.cp_len(n);
        let lo = expected;
        let hi = (expected + window).min(r.len());
        if hi - lo < n + l {
            return None;
        }
        let curve = crate::sync::metric_curve(&r[lo..hi], n, gi);
        if curve.len() < 3 {
            return None;
        }
        // 期待位置（offset 0）からの許容ドリフト。実測で 800 シンボルあたり
        // 1.9 サンプルなので 16 サンプルは十分余裕がある。
        //
        // ただし capture によって真の境界の落ち方が変わる。実測:
        //   ck.iq  metric 0.96 / MB 0.16/s
        //   cap2.iq metric 0.89 / MB 2.77/s
        // 同じ復調器・同じコードなのに capture で 17 倍悪化する。
        // 探索範囲を `ISDBT_TOL` で広げて検証できるようにしておく。
        const TOL_DEFAULT: usize = 16;
        let tol: usize = std::env::var("ISDBT_TOL")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(TOL_DEFAULT);
        // offset 0..tol の範囲で、局所最大値のうち metric が最も高い peak を選ぶ。
        // 単調増加区間（端）を選んでしまわないよう、両隣と比べて局所最大を要求する。
        let mut best: Option<(usize, f32)> = None;
        for d in 0..=tol.min(curve.len() - 1) {
            let v = curve[d];
            if v < 0.2 {
                continue;
            }
            let left = if d == 0 { v } else { curve[d - 1] };
            let right = if d + 1 < curve.len() { curve[d + 1] } else { v };
            if v >= left && v >= right {
                if best.map_or(true, |(_, bv)| v > bv) {
                    best = Some((lo + d, v));
                }
            }
        }
        best
    }

    /// 1シンボルを復調しつつ、次のシンボル境界をCP相関で更新する。
    /// 返り値は `(spectrum, 次のシンボル開始位置)`。
    ///
    /// # 境界追跡の設計（実測に基づく）
    ///
    /// 1seg の CP 自己相関は**短窓では統計的に peak が立たない**。実測:
    ///
    /// | 探索窓 | メトリクス |
    /// |--------|-----------|
    /// | 1 シンボル (1152 サンプル) | 0.03-0.17（0.2 通過率 1.0%） |
    /// | 32 シンボル (36,864) | 0.15-0.22（窓内で誤 peak が発生） |
    /// | 1M サンプル (868 シンボル) | **0.30（安定して peak が立つ）** |
    ///
    /// したがって 1 シンボル窓で探索すると `.filter(|e| e.metric > 0.2)` が
    /// ほぼ全シンボルで失敗し、`unwrap_or(expected)` にフォールバックしていた。
    /// その結果境界が固定 1152 ずつ進み、実 fs（1015873.002204）と指定 fs
    /// （1015873）の差 ≈0.0022 サンプル/シンボルが累積して、ライブ入力では
    /// 約 2000 RS ブロック（約 7 秒）で境界が破綻し出力が恒久停止した。
    ///
    /// 正しい設計は**周期的な長窓再同期**にすること。`reacquire_every` シンボル
    /// ごとに 1M サンプル窓で CP 探索し、間のシンボルはその結果の周期
    /// `LONG_WINDOW_SYMS * sym` で等間隔に詰める。長窓が実 fs を正しく反映した
    /// のでその間の累積ドリフトは 0.0022 × 868 ≈ 1.9 サンプルに収まり、
    /// 短窓の探索半径内で追従できる。
    pub fn demod_one_tracked(
        &self,
        r: &[Complex32],
        expected: usize,
        gi: GuardInterval,
        cfo_subcarriers: f32,
        radius: usize,
    ) -> Option<(Vec<Complex32>, usize)> {
        // 分数境界オフセットを外から上書きできるようにする。
        // 実 fs 1015873.002204 と指定 1015873 の差は 0.0022 サンプル/シンボル。
        // 整数サンプル境界に丸め続けると系統誤差が蓄積して等化器のタップが
        // 合わず、RS は 100% でも H.264 ビットが一部化ける
        // （実測 2026-09-26: 保存 IQ で MB エラー 1 件/10 秒、ライブで
        //  70 件/25 秒。境界 metric は 0.96 対 0.89 だった）。
        self.demod_one_tracked_frac(r, expected, gi, cfo_subcarriers, radius, 0.0)
    }

    /// `frac` は `expected` に加える分数サンプル位置（0.0〜1.0）。
    pub fn demod_one_tracked_frac(
        &self,
        r: &[Complex32],
        expected: usize,
        gi: GuardInterval,
        cfo_subcarriers: f32,
        radius: usize,
        frac: f32,
    ) -> Option<(Vec<Complex32>, usize)> {
        let n = self.n;
        let l = gi.cp_len(n);
        let sym = n + l;
        let radius = radius.min(16);
        if expected + sym + radius > r.len() {
            return None;
        }
        let lo = expected.saturating_sub(radius);
        let hi = (expected + sym + radius).min(r.len());
        let start = crate::sync::estimate_symbol_sync(&r[lo..hi], n, gi)
            .filter(|e| e.metric > 0.2)
            .map(|e| lo + e.symbol_start)
            .unwrap_or(expected);
        if start + sym > r.len() {
            return None;
        }
        // 分数境界: FFT 窓の起点を線形補間でずらす。整数位置の丸めより
        // サンプリング位相の誤差が半分以下になる。
        let base = start + l;
        let w0 = 1.0 - frac;
        let w1 = frac;
        let mut buf = vec![Complex32::new(0.0, 0.0); n];
        let w = Complex32::from_polar(1.0, -2.0 * PI * cfo_subcarriers / (n as f32));
        let mut ph = Complex32::new(1.0, 0.0);
        for k in 0..n {
            let a = r[base + k];
            let b = if base + k + 1 < r.len() { r[base + k + 1] } else { a };
            buf[k] = (a * w0 + b * w1) * ph;
            ph *= w;
        }
        self.fft.process(&mut buf);
        let half = n / 2;
        let scale = 1.0 / (n as f32).sqrt();
        let mut spec = vec![Complex32::new(0.0, 0.0); n];
        for k in 0..n {
            spec[(k + half) % n] = buf[k] * scale;
        }
        Some((spec, start + sym))
    }

    /// 1シンボルだけ復調（ストリーミング/逐次処理用）。`r[sym_start ..]` から
    /// CPを飛ばして有効N点をFFT・fftshift。CFO位相は**シンボルローカル**（m=0..N）で、
    /// シンボル間の定数位相差は後段の[`crate::equalize`]（SPで毎シンボルH推定）が吸収する。
    /// 返り値は長さ`fft_len`のfftshift済みスペクトル。
    pub fn demod_one(
        &self,
        r: &[Complex32],
        sym_start: usize,
        gi: GuardInterval,
        cfo_subcarriers: f32,
    ) -> Vec<Complex32> {
        let n = self.n;
        let l = gi.cp_len(n);
        let half = n / 2;
        let scale = 1.0 / (n as f32).sqrt();
        let base = sym_start + l;
        let mut buf = vec![Complex32::new(0.0, 0.0); n];
        // CFO補正：sin/cosを毎サンプル呼ばず、定数位相子を掛け続ける増分回転（大幅高速化）。
        let w = Complex32::from_polar(1.0, -2.0 * PI * cfo_subcarriers / (n as f32));
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
        spec
    }
}
