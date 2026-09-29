//! 単一 RS ブロックの全段トレース。
//!
//! # 目的
//!
//! 集計統計（平均・相関・ヒストグラム）では「どこで数値が理論値から外れ始めるか」が
//! 見えない。そこで **1 ブロックだけ** を選び、`estimate_symbol_sync` → FFT →
//! `estimate_channel` → 等化 → Viterbi → deinterleave → RS の各段の生値を全部
//! ダンプし、正常ブロックと 1 バイト単位で突き合わせる。
//!
//! # 有効化
//!
//! `ISDBT_TRACE=<blk>` を指定すると、そのブロック番号と近傍（±4）、および
//! **最初の訂正不能ブロック**を自動的にトレース対象にする。
//! 出力は `BlockTrace::dump()` がcalledSuggested
//! む時に stderr に出る。

use num_complex::Complex32;

/// 1 シンボルのトレース。
#[derive(Clone, Default)]
pub struct SymTrace {
    /// OFDM フレーム内でのシンボル番号。
    pub sym_idx: u64,
    ///  demod した FFT 前の CP 境界（サンプルインデックス）。
    pub sym_start: usize,
    /// 探索窓 `[lo, hi)`。
    pub win_lo: usize,
    pub win_hi: usize,
    /// `estimate_symbol_sync` が返した `symbol_start`（窓内相対）。
    pub est_symbol_start: usize,
    /// 同期の quality（`metric`）。
    pub est_metric: f32,
    /// `symbol_index_in_frame`（`sym_mod4`）。
    pub sym_mod4: usize,
    /// FFT 前の CP 境界（等化之前）。
    pub base: usize,
    /// SP 位置の `Y/P`。Carrier インデックス → 値。
    pub sp_hat: Vec<(usize, Complex32)>,
    /// 等化後のデータキャリア。`data_carrier_indices` の順。
    pub eq_data: Vec<Complex32>,
    /// 周波数デインターリーブ後。
    pub freq_deint: Vec<Complex32>,
    /// 時間デインターリーブ後。
    pub time_deint: Vec<Complex32>,
    /// QPSK ソフト-demap 後（[lsb, msb] の順で 2 値ずつ）。
    pub soft: Vec<f32>,
    /// depuncture 後の母符号。
    pub mother: Vec<f32>,
}

/// 1 RS ブロック分のトレース。
#[derive(Clone, Default)]
pub struct BlockTrace {
    /// トレース対象のブロック番号。
    pub target_blk: u64,
    /// 実際に完了したブロック番号。
    pub blk: u64,
    /// 対象ブロックのシンボルたち。
    pub syms: Vec<SymTrace>,
    /// 推定されたブロック境界（`block_phase`）。
    pub block_phase: usize,
    /// `depu_pos` 開始値。
    pub depu_pos_start: usize,
    /// depuncture された母符号から復元したバイト列。
    pub bytes: Vec<u8>,
    /// RS 訂正結果。
    pub rs: Option<Vec<u8>>,
    pub rs_uncorrectable: bool,
}

impl BlockTrace {
    /// 対象のブロック番号の近傍か、最初の訂正不能なら true。
    pub fn wants(&self, blk: u64, first_bad: Option<u64>) -> bool {
        let near = |b: u64| b.abs_diff(self.target_blk) <= 4;
        near(blk) || first_bad == Some(blk)
    }

    /// 1 TSD を 16 進ダンプ（1 バイト = 2 桁、8 バイトごとに区切り）。
    pub fn hexdump(label: &str, data: &[u8]) -> String {
        let mut s = String::new();
        for (i, b) in data.iter().enumerate() {
            if i > 0 && i % 8 == 0 {
                s.push(' ');
            }
            s.push_str(&format!("{:02X}", b));
        }
        format!("{label}: {s}\n")
    }

    /// 浮動小数点配列の統計。最小・最大・平均・二乗平均。
    pub fn stats(label: &str, v: &[f32]) -> String {
        if v.is_empty() {
            return format!("{label}: (空)\n");
        }
        let n = v.len() as f64;
        let min = v.iter().cloned().fold(f32::INFINITY, f32::min);
        let max = v.iter().cloned().fold(f32::NEG_INFINITY, f32::max);
        let mean = v.iter().map(|&x| x as f64).sum::<f64>() / n;
        let rms = (v.iter().map(|&x| (x as f64) * (x as f64)).sum::<f64>() / n).sqrt();
        format!(
            "{label}: n={} min={:+.4} max={:+.4} mean={:+.4} rms={:.4}\n",
            v.len(),
            min,
            max,
            mean,
            rms
        )
    }

    /// 複素数配列の統計（電力）。
    pub fn cstats(label: &str, v: &[Complex32]) -> String {
        if v.is_empty() {
            return format!("{label}: (空)\n");
        }
        let n = v.len() as f64;
        let p: Vec<f64> = v.iter().map(|c| (c.re as f64) * (c.re as f64) + (c.im as f64) * (c.im as f64)).collect();
        let mean = p.iter().sum::<f64>() / n;
        let rms = mean.sqrt();
        let min = p.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = p.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        format!(
            "{label}: n={} |H|^2 min={:.2} max={:.2} mean={:.4} rms={:.4}\n",
            v.len(),
            min,
            max,
            mean,
            rms
        )
    }

    /// 全段を 1 つの文字列にまとめる。
    pub fn render(&self) -> String {
        let mut s = String::new();
        s.push_str(&format!(
            "\n======== TRACE blk={} (対象 {}) ========\n",
            self.blk, self.target_blk
        ));
        s.push_str(&format!(
            "block_phase={} depu_pos_start={} シンボル数={}\n",
            self.block_phase,
            self.depu_pos_start,
            self.syms.len()
        ));
        for st in &self.syms {
            s.push_str(&format!(
                "  sym#{} start={} win=[{},{}) est_start={} metric={:.4} mod4={} base={}\n",
                st.sym_idx,
                st.sym_start,
                st.win_lo,
                st.win_hi,
                st.est_symbol_start,
                st.est_metric,
                st.sym_mod4,
                st.base
            ));
            let spv: Vec<Complex32> = st.sp_hat.iter().map(|&(_, c)| c).collect();
            s.push_str(&Self::cstats("    SP  Y/P", &spv));
            s.push_str(&Self::cstats("    EQ  data", &st.eq_data));
            s.push_str(&Self::stats("    soft", &st.soft));
            // ゼロの割合（erasure の疑い）
            let z = st.mother.iter().filter(|&&x| x == 0.0).count();
            s.push_str(&format!(
                "    mother: n={} zero={} ({:.2}%)\n",
                st.mother.len(),
                z,
                z as f64 / st.mother.len().max(1) as f64 * 100.0
            ));
        }
        // 全シンボル横断の集計。個別シンボルではなく、この 1 ブロック全体の
        // 「理論値からの逸脱」を見る。
        let all_sp: Vec<Complex32> = self.syms.iter().flat_map(|x| x.sp_hat.iter().map(|&(_, c)| c)).collect();
        let all_eq: Vec<Complex32> = self.syms.iter().flat_map(|x| x.eq_data.iter().copied()).collect();
        let all_soft: Vec<f32> = self.syms.iter().flat_map(|x| x.soft.iter().copied()).collect();
        let all_moth: Vec<f32> = self.syms.iter().flat_map(|x| x.mother.iter().copied()).collect();
        s.push_str("  ---- 全体集計 ----\n");
        s.push_str(&Self::cstats("    ALL SP  Y/P", &all_sp));
        s.push_str(&Self::cstats("    ALL EQ  data", &all_eq));
        // 等化後キャリアを「SP 間補間」と「両端外挿」に分けて比較する。
        // `estimate_channel` は SP 間は線形補間、両端は `slope` による線形外挿。
        // 外挿部分が発散していれば、そこから soft が ±8 に叩き込まれる。
        // mode 3: SP は k = 12p + 3(mod4)、n_carriers = 432。
        let (mut interp, mut extrap) = (Vec::new(), Vec::new());
        for st in &self.syms {
            let phase = st.sym_mod4;
            let sp: Vec<usize> = (0..)
                .map(|p| 12 * p + 3 * phase)
                .take_while(|&k| k < st.eq_data.len())
                .collect();
            if sp.is_empty() {
                continue;
            }
            let first = sp[0];
            let last = *sp.last().unwrap();
            for (i, &v) in st.eq_data.iter().enumerate() {
                if i < first || i > last {
                    extrap.push(v);
                } else {
                    interp.push(v);
                }
            }
        }
        s.push_str(&Self::cstats("    EQ  SP間補間", &interp));
        s.push_str(&Self::cstats("    EQ  両端外挿", &extrap));
        s.push_str(&Self::stats("    ALL soft", &all_soft));
        // ソフト値の絶対値が 0.5 未満 = 復調確信度が低い = BER リスク
        let weak = all_soft.iter().filter(|&&x| x.abs() < 0.5).count();
        s.push_str(&format!(
            "    ALL soft |x|<0.5: {}/{} ({:.2}%)\n",
            weak,
            all_soft.len(),
            weak as f64 / all_soft.len().max(1) as f64 * 100.0
        ));
        let z = all_moth.iter().filter(|&&x| x == 0.0).count();
        s.push_str(&format!(
            "    ALL mother zero: {}/{} ({:.2}%)\n",
            z,
            all_moth.len(),
            z as f64 / all_moth.len().max(1) as f64 * 100.0
        ));
        s.push_str(&Self::hexdump("  BYTES(末尾64)", &self.bytes));
        s.push_str(&format!(
            "  RS: {} (syndromes {:?})\n",
            if self.rs_uncorrectable {
                "訂正不能"
            } else {
                "訂正済み"
            },
            self.rs.as_ref().map(|_| ())
        ));
        s.push_str("======== TRACE END ========\n");
        s
    }
}
