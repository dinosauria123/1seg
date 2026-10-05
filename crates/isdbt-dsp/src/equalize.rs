//! ③ チャネル等化：スキャッタードパイロット(SP)によるチャネル推定と等化。
//!
//! 流れ（gr-isdbt `ofdm_synchronization_impl.cc` の RX 側に倣う）：
//! 1. SPキャリアで `H[l] = Y[l] / pilot[l]`（既知パイロットで割る）
//! 2. SP間（12本間隔）を周波数方向に線形補間
//! 3. 両端は最寄りSP対の傾きで外挿
//! 4. 全キャリアを `X[l] = Y[l] / H[l]` で等化
//!
//! SPの位相（`symbol % 4`）は [`detect_symbol_phase`] が、隣接SPチャネル推定の
//! コヒーレンスを最大化する候補として求める（PRBS符号が正しいと隣接SP同士が
//! 揃い、コヒーレンス≈1になる性質を使う）。

use crate::params::FFT_LEN;
use crate::pilots::{SegmentPilots, SEGMENT_CARRIERS, SP_SPACING};
use num_complex::Complex32;

/// fftshift済み1024スペクトルの中で、中央セグメント432本が始まるbin。
/// `(1024 - 432)/2 = 296`（DCはbin512＝ローカルキャリア216）。
pub const SEGMENT_BIN_OFFSET: usize = (FFT_LEN - SEGMENT_CARRIERS) / 2;

/// fftshift済みスペクトル（長さ`FFT_LEN`）から中央セグメント432本を取り出す。
///
/// `bin_offset` は通常 [`SEGMENT_BIN_OFFSET`]。DCオフセットや半キャリアずれを
/// 経験的に詰めたいとき用に可変にしてある。
pub fn extract_segment(spectrum: &[Complex32], bin_offset: usize) -> Vec<Complex32> {
    spectrum[bin_offset..bin_offset + SEGMENT_CARRIERS].to_vec()
}

/// 残留サンプルタイミング誤差を推定する。
///
/// **方式 (b) Moose / Classen**（隣接パイロット間の相関）を、
/// 周波数方向（間隔 `D`）と時間方向（4 シンボル周期、3 carrier シフト）の
/// **両方向**で合算して S/N を上げる。
///
/// ```text
///   H_hat(k) = Y(k) / P(k)
///   ε_hat    = -(N / (2π·D_eff)) · angle( Σ H_hat(k)·conj(H_hat(k+D)) )
/// ```
///
/// 時間方向（4 シンボル周期、3 carrier シフト）でも積算して S/N を上げる。
///
/// SP パターン周期が 4 なので `symbol%4` だけが 3 carrier ずつずれる。
/// 時間方向の「隣接」は `symbol%4` を跨いだ同じ論理周波数の carrier
/// どうし（`D_time = 3`）に対応する。`D_eff = 3` だと
/// エイリアシングフリー範囲は `N/(2·3) ≈ 170` サンプルに広がり、
/// 周波数方向（`D=12`）よりチャネル位相勾配の影響が小さくなる。
///
/// 返り値は**サンプルの小数部**。エイリアシングフリー範囲は
/// `N/(2·D_eff)` なので、呼び出し側が範囲を確認する。
pub fn sp_timing_offset(seg: &[Complex32], sym_mod4: usize, pilots: &SegmentPilots) -> f32 {
    sp_timing_offset_2d(seg, None, sym_mod4, 0.0, pilots)
}

/// 1 シンボルのセグメントと `symbol%4`（SP 履歴バッファ用）。
pub struct SegSample<'a> {
    pub seg: &'a [Complex32],
    pub mod4: usize,
}

/// 複数フレームをまたいで SP チャネル推定を coherent 平均するアキュムレータ。
///
/// **なぜ必要か**: 4 シンボル分の履歴だけで `angle()` を取ると ε が
/// `-18 〜 +28` サンプルで行き来する（実測 2026-09-26）。各 carrier の
/// 観測が 1 回しかなくノイズがそのまま ε になるため。
/// 同じ `(phase, carrier)` の観測を複素数のまま平均すれば S/N は
/// 観測数に比例して改善する（準静的フェージングなので coherent 平均が効く）。
///
/// **4 位相が揃うまで [`Self::finalize`] を使ってはいけない**: 欠けた位相の
/// ところだけ間隔が 6 に飛び、`D=3` 前提の相関が局所的に誤推定を出して
/// 振動の原因になる。`is_complete()` で確認してから使う。
pub struct SpGridAccumulator {
    /// carrier インデックス → (複素数の総和, 観測数)
    acc: Vec<(Complex32, u32)>,
    /// 見た `symbol%4` のビット集合
    seen: [bool; 4],
}

impl SpGridAccumulator {
    pub fn new() -> Self {
        Self { acc: vec![(Complex32::new(0.0, 0.0), 0); SEGMENT_CARRIERS], seen: [false; 4] }
    }

    /// 1 シンボルの `(k, H_hat)` を投入する。`mod4` は `0..4`。
    pub fn push(&mut self, mod4: usize, seg: &[Complex32], pilots: &SegmentPilots) {
        if mod4 >= 4 {
            return;
        }
        self.seen[mod4] = true;
        for l in pilots.sp_carriers(mod4) {
            if let Some(h) = h_hat(seg, l, pilots) {
                let e = &mut self.acc[l];
                e.0 += h;
                e.1 += 1;
            }
        }
    }

    /// 4 位相すべて投入済みか。
    pub fn is_complete(&self) -> bool {
        self.seen.iter().all(|&b| b)
    }

    /// 位相 0 の SP だけを昇順で返す（`D=12` 推定用）。
    pub fn phase0_grid(&self) -> Vec<(usize, Complex32)> {
        (0..SEGMENT_CARRIERS)
            .filter(|&k| k % SP_SPACING == 0 && self.acc[k].1 > 0)
            .map(|k| (k, self.acc[k].0 / self.acc[k].1 as f32))
            .collect()
    }

    /// 位相 1 のみ見る（仮説A: 位相1の P(k) が反転しているか）。
    pub fn phase1_grid(&self) -> Vec<(usize, Complex32)> {
        (0..SEGMENT_CARRIERS)
            .filter(|&k| k % SP_SPACING == 3 && self.acc[k].1 > 0)
            .map(|k| (k, self.acc[k].0 / self.acc[k].1 as f32))
            .collect()
    }

    /// 4 位相の `H_hat` の平均位相。仮説A の符号列不整合検出用。
    /// 位相 0 の平均位相を基準に、相位 1,2,3 が π ずれているかを見る。
    pub fn phase_mean_angle(&self) -> [Option<f32>; 4] {
        let mut out = [None; 4];
        for (i, &m) in [0usize, 3, 6, 9].iter().enumerate() {
            let mut acc = Complex32::new(0.0, 0.0);
            let mut n = 0u32;
            for k in 0..SEGMENT_CARRIERS {
                if k % SP_SPACING == m && self.acc[k].1 > 0 {
                    acc += self.acc[k].0 / self.acc[k].1 as f32;
                    n += 1;
                }
            }
            out[i] = if n > 0 { Some(acc.arg()) } else { None };
        }
        out
    }

    /// 各 carrier の観測回数（位相 0 の SP のみ対象）。
    pub fn observations(&self) -> u32 {
        (0..SEGMENT_CARRIERS)
            .filter(|&k| k % SP_SPACING == 0)
            .map(|k| self.acc[k].1)
            .min()
            .unwrap_or(0)
    }

    pub fn reset(&mut self) {
        self.acc = vec![(Complex32::new(0.0, 0.0), 0); SEGMENT_CARRIERS];
        self.seen = [false; 4];
    }
}

impl Default for SpGridAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

/// 位相 0 単体（`D=12`）で ε を求める。
///
/// Classen の相殺が成り立つのは**同一シンボル内の 2 キャリア比較**のときだけ:
///
/// ```text
///   H(k)   = A(l)·exp(+j2πkε/N)·C(k,l)
///   H(k+D) = A(l)·exp(+j2π(k+D)ε/N)·C(k+D,l)
///   → H(k+D)·conj(H(k)) = exp(+j2πDε/N)·C(k+D,l)·conj(C(k,l))
/// ```
///
/// `A(l)` はシンボル `l` 共通の位相（残留 CFO・位相雑音・伝搬遅延）。
/// 同一シンボル内なら両方に同じ `A(l)` が乗るので積で完全に消える。
/// エイリアシングフリー範囲は `n_fft/(2D) = 1024/24 ≈ 43` サンプル。
///
/// **4 位相統合で `D=3` にしてはいけない**: 統合グリッドの隣接 `(k, k+3)` は
/// 位相 0 のシンボル `s` と位相 1 のシンボル `s+1` 由来なので `A(s+1)·conj(A(s))`
/// が残り打ち消されない。その残留を小さなスケール `N/(2π·3)` で割ると見かけの
/// サンプル誤差が大きく増幅される。実測でも `D=3` は `+65〜+167` サンプル
/// （範囲端）に飽和し、観測数を 128→1792 に増やしても改善しなかった。
/// **採用しない**。
///
/// 将来の範囲拡大が要るなら、空間方向（4 位相合成）ではなく**時間方向の
/// unwrap/積算**（毎シンボル `D=12` で得た ε を前回値からの連続性で
/// アンラップして累積）の方が正しい。
/// 実測値: `-0.10 〜 -1.54` サンプル（sub-sample、妥当）。
pub fn timing_offset_phase0_d12(grid: &[(usize, Complex32)]) -> Option<f32> {
    const D: usize = 12;
    let mut acc = Complex32::new(0.0, 0.0);
    let mut count = 0usize;
    for w in grid.windows(2) {
        let ((ka, ha), (kb, hb)) = (w[0], w[1]);
        if kb - ka != D {
            continue;
        }
        acc += hb * ha.conj();
        count += 1;
    }
    if count == 0 || acc.norm() < 1e-9 {
        return None;
    }
    Some(-(FFT_LEN as f32) / (2.0 * std::f32::consts::PI * D as f32) * acc.arg())
}

/// 4 位相揃った統合グリッドから Classen/Moose 型 ε を求める。
///
/// `grid` は [`SpGridAccumulator::finalize`] の出力（間隔 3）。
/// エイリアシングフリー範囲は `n_fft/(2*3) ≈ 170` サンプル。
///
/// **未解決**: 実測で ε が `+109 〜 +167` サンプル（範囲の端）に張り付く。
/// 観測数を 128 → 1792 に増やしても改善しない。統計ノイズなら観測数に
/// 比例して必ず小さくなるので、これは**決定論的バグ**の証拠。
/// investigación 仮説A: 位相 1,2,3 の P(k) 符号列が送信側と不一致
/// （PRBS 状態遷移の取り違え）で、残りの H_hat だけ符号が反転している。
/// 仮説B: 粗タイミングの基準点（GI 先頭 vs FFT 窓の開始）の規約ズレで、
/// 100 サンプル超の整数オフセットが実際に存在する。
/// 切り分けは [`timing_offset_phase0_d12`] との比較で行う。
/// 当面は既定無効（`ISDBT_SPTRACK=0`）で、診断用途に限る。


/// 残留サンプルタイミング誤差を推定する。
///
/// **4 シンボル統合 + 周波数方向 Classen/Moose 相関**。
///
/// SP パターン周期は 4 シンボルで位置が 3 carrier ずつシフトするので、
/// 4 シンボルの `H_hat` を 1 つのグリッドに統合すると実効間隔が
/// `D = 12 → 3` に縮まり、エイリアシングフリー範囲 `n_fft/(2D)` が 4 倍に
/// 広がる（`1024/24 ≈ 43` → `1024/6 ≈ 170` サンプル）。
///
/// ```text
///   H_hat(k) = Y(k) / P(k)
///   merged   = { H_hat(k) : k = 0,3,6,9,...,429 }      （4 シンボル統合後）
///   ε_hat    = -(n_fft / (2π·D)) · angle( Σ H_hat(k+D)·conj(H_hat(k)) )
/// ```
///
/// **準静的フェージングの仮定**が要る（4 シンボル内でチャネルがほぼ一定）。
/// 4 観測は同一 carrier の 4 回観測なので CFO は共通項として打ち消される。
pub fn sp_timing_offset_2d(
    seg: &[Complex32],
    prev_hist: Option<&[SegSample<'_>]>,
    sym_mod4: usize,
    _cfo_subcarriers: f32,
    pilots: &SegmentPilots,
) -> f32 {
    let (eps, _f, _t) = sp_timing_offset_2d_diag(
        seg, prev_hist, sym_mod4, _cfo_subcarriers, pilots,
    );
    eps
}

/// [`sp_timing_offset_2d`] の診断版。将来 2 方向推定に拡張したときの拡張点。
pub fn sp_timing_offset_2d_diag(
    seg: &[Complex32],
    prev_hist: Option<&[SegSample<'_>]>,
    sym_mod4: usize,
    _cfo_subcarriers: f32,
    pilots: &SegmentPilots,
) -> (f32, f32, f32) {
    let n = FFT_LEN as f32;
    const D_EFF: usize = 3;   // 4 シンボル統合後の実効 SP 間隔

    // --- 4 シンボルの SP を 1 つのグリッドに統合 ---
    //
    // SP パターン周期は 4 シンボルで、位置が 3 carrier ずつシフトする:
    //   symbol%4 = 0 → k = 0, 12, 24, ...
    //   symbol%4 = 1 → k = 3, 15, 27, ...
    //   symbol%4 = 2 → k = 6, 18, 30, ...
    //   symbol%4 = 3 → k = 9, 21, 33, ...
    //
    // 4 つを統合すると **k = 0, 3, 6, 9, ...** となり、実効間隔が 12 → 3 に
    // 縮まる。エイリアシングフリー範囲 `n_fft/(2D)` は 4 倍（`1024/6 ≈ 170`）に
    // 広がる。
    //
    // **重要**: これは「別シンボルの carrier を跨いで相関する」のではない。
    // 4 シンボルを**同一 carrier の 4 回の観測**として 1 グリッドにまとめ、
    // そこから**周波数方向**の相関を取る。こうすると CFO は 4 観測に共通なので
    // 打ち消され、準静的フェージング（4 シンボル内でチャネルがほぼ一定）だけで
    // よい。素朴に「前のシンボルの p と現在の p+3」を相関すると CFO が残り、
    // 実測で +27〜+37 サンプルという無意味な値になる。
    let mut grid: Vec<(usize, Complex32)> = Vec::with_capacity(SEGMENT_CARRIERS);
    {
        // 4 シンボルの `H_hat` を `k` ごとに coherent 平均して 1 グリッドにまとめる。
        //
        // **Simple 平均では S/N が足りない**（実測 2026-09-26: 3 シンボル履歴
        // だけだと ε が -18〜+28 サンプルで行き来する）。理由は各 carrier の
        // 観測が 1 回しかなく、`angle()` のノイズがそのまま ε になること。
        // 複数フレームをまたいで coherent 平均すれば S/N は観測数に比例して
        // 改善する（フェージングは準静的なので coherent 平均が効く）。
        let mut acc_v = vec![Complex32::new(0.0, 0.0); SEGMENT_CARRIERS];
        let mut cnt = vec![0u32; SEGMENT_CARRIERS];
        {
            let mut add = |s: &[Complex32], m: usize| {
                let sp: Vec<usize> = pilots.sp_carriers(m).collect();
                for &l in &sp {
                    if let Some(h) = h_hat(s, l, pilots) {
                        acc_v[l] += h;
                        cnt[l] += 1;
                    }
                }
            };
            add(seg, sym_mod4);
            if let Some(hist) = prev_hist {
                for seg_p in hist.iter().take(3) {
                    add(seg_p.seg, seg_p.mod4);
                }
            }
        }
        for k in 0..SEGMENT_CARRIERS {
            if cnt[k] > 0 {
                grid.push((k, acc_v[k] / cnt[k] as f32));
            }
        }
    }
    if grid.len() < 8 {
        return (0.0, f32::NAN, f32::NAN);
    }

    // --- 周波数方向の相関（実効 D = 3） ---
    //
    // 符号: `H_hat(k+D)·conj(H_hat(k))`（paste 4 と同じ向き）。逆だと
    // ε の符号が反転する。
    let mut acc = Complex32::new(0.0, 0.0);
    let mut count = 0usize;
    for w in grid.windows(2) {
        let ((ka, ha), (kb, _hb)) = (w[0], w[1]);
        if kb - ka != D_EFF {
            continue;
        }
        acc += _hb * ha.conj();
        count += 1;
    }
    if count == 0 || acc.norm() < 1e-9 {
        return (0.0, f32::NAN, f32::NAN);
    }
    let eps = -(n / (2.0 * std::f32::consts::PI * D_EFF as f32)) * acc.arg();
    (eps, eps, f32::NAN)
}

/// SP 位置 `a` と `b` の `H_hat(a)·conj(H_hat(b))` を返す。
/// 振幅が小さすぎる場合は `None`。
#[allow(dead_code)] // 診断用の未被呼び出しヘルパ
fn sp_pair(seg: &[Complex32], a: usize, b: usize, pilots: &SegmentPilots) -> Option<Complex32> {
    let (ha, hb) = (h_hat(seg, a, pilots)?, h_hat(seg, b, pilots)?);
    Some(ha * hb.conj())
}

/// 別シンボルの SP 位置 `a`（`seg_a` 側）と `b`（`seg_b` 側）の相関。
#[allow(dead_code)] // 診断用の未被呼び出しヘルパ
fn sp_pair_across(
    seg_a: &[Complex32],
    seg_b: &[Complex32],
    a: usize,
    b: usize,
    pilots: &SegmentPilots,
) -> Option<Complex32> {
    let (ha, hb) = (h_hat(seg_a, a, pilots)?, h_hat(seg_b, b, pilots)?);
    Some(ha * hb.conj())
}

/// `H_hat(k) = Y(k) / P(k)`。既知 pilot 値が 0 か、受信値が小さすぎる場合は `None`。
fn h_hat(seg: &[Complex32], l: usize, pilots: &SegmentPilots) -> Option<Complex32> {
    let k = Complex32::new(pilots.values[l], 0.0);
    if k.norm() < 1e-9 {
        return None;
    }
    if seg[l].norm() < 1e-3 {
        return None;      // 振幅が小さすぎる SP はノイズが支配的
    }
    Some(seg[l] / k)
}

/// SPから周波数方向チャネル `H`（長さ432）を推定する。
///
/// `seg` は中央セグメント432本、`sym_mod4` はそのシンボルの `symbol%4`。
pub fn estimate_channel(
    seg: &[Complex32],
    sym_mod4: usize,
    pilots: &SegmentPilots,
) -> Vec<Complex32> {
    let n = SEGMENT_CARRIERS;
    let mut h = vec![Complex32::new(0.0, 0.0); n];
    let sp: Vec<usize> = pilots.sp_carriers(sym_mod4).collect();

    // 1) SP位置で生チャネル推定（パイロットは実数 ±4/3）
    for &l in &sp {
        h[l] = seg[l] / pilots.values[l];
    }

    // 2) 隣接SP間（12本間隔）を線形補間
    for w in sp.windows(2) {
        let (a, b) = (w[0], w[1]);
        let span = (b - a) as f32;
        let (ha, hb) = (h[a], h[b]);
        for l in (a + 1)..b {
            let t = (l - a) as f32 / span;
            h[l] = ha * (1.0 - t) + hb * t;
        }
    }

    // 3) 両端を傾きで外挿
    let first = sp[0];
    if first > 0 && sp.len() >= 2 {
        let slope = (h[sp[1]] - h[sp[0]]) / Complex32::new((sp[1] - sp[0]) as f32, 0.0);
        for l in 0..first {
            let d = l as f32 - first as f32;
            h[l] = h[first] + slope * Complex32::new(d, 0.0);
        }
    }
    let last = *sp.last().unwrap();
    if last < n - 1 && sp.len() >= 2 {
        let m = sp.len();
        let slope =
            (h[sp[m - 1]] - h[sp[m - 2]]) / Complex32::new((sp[m - 1] - sp[m - 2]) as f32, 0.0);
        for l in (last + 1)..n {
            let d = l as f32 - last as f32;
            h[l] = h[last] + slope * Complex32::new(d, 0.0);
        }
    }

    h
}

/// 推定チャネル `h` でセグメントを等化（`X = Y / H`）。
/// `|H|` が極小のbinは0にする（端の外挿が破綻した場合の保険）。
pub fn equalize(seg: &[Complex32], h: &[Complex32]) -> Vec<Complex32> {
    // 診断: `ISDBT_EQOFF=1` で ZF を切り、identity（`h` で割らない）に
    // する。
    //
    // 88% の hot が「`|H|` を分母にした除算」の副作用なら、これを OFF に
    // した瞬間に drop が 0.1% 付近へ落ちる。落ちれば犯人は等化器の分母
    // 構造に確定し、`l=204/205` 周辺で `|H|` を過大/過小評価している箇所を
    // 追える。落ちなければ等化器は無罪で、soft 以降（demap / depuncture /
    // Viterbi）に犯人の範囲が移る。
    //
    // これ��� ZF（ゼロ强迫）なので identity だと**ほぼ復調できない**のは
    // 予想通り。判定は「drop が 0.1% 付近まで**改善**するか」で、
    // 「等化器が消えて RS が全部落ちた」だけでは何も言えない。
    if eqoff_enabled() {
        return seg.to_vec();
    }
    seg.iter()
        .zip(h)
        .map(|(&y, &hh)| {
            if hh.norm_sqr() < 1e-12 {
                Complex32::new(0.0, 0.0)
            } else {
                y / hh
            }
        })
        .collect()
}

/// 診断: `ISDBT_EQOFF` を 1 度だけ読んでキャッシュする。
pub fn eqoff_enabled() -> bool {
    use std::sync::OnceLock;
    static FLAG: OnceLock<bool> = OnceLock::new();
    *FLAG.get_or_init(|| std::env::var("ISDBT_EQOFF").is_ok_and(|v| v != "0"))
}

/// 隣接SPチャネル推定のコヒーレンス（0..1）。
///
/// `|Σ H[i+1]·conj(H[i])| / Σ |H[i+1]||H[i]|`。チャネルが周波数方向に滑らか
/// かつ PRBS符号（＝`sym_mod4`）が正しいと隣接SPが揃い1に近づく。位相が違うと
/// SP位置・符号がずれてランダム化し小さくなる。
pub fn sp_coherence(seg: &[Complex32], sym_mod4: usize, pilots: &SegmentPilots) -> f32 {
    let mut num = Complex32::new(0.0, 0.0);
    let mut den = 0.0f32;
    let mut prev: Option<Complex32> = None;
    for l in pilots.sp_carriers(sym_mod4) {
        let h = seg[l] / pilots.values[l];
        if let Some(p) = prev {
            num += h * p.conj();
            den += h.norm() * p.norm();
        }
        prev = Some(h);
    }
    if den < 1e-12 {
        0.0
    } else {
        num.norm() / den
    }
}

/// gr-isdbtの方式に寄ったSP位相スコア。
/// 候補位相の各SP隣接対について、受信値の積に既知PRBSの符号関係を掛ける。
pub fn phase_scores_gr_isdbt(seg: &[Complex32], pilots: &SegmentPilots) -> [f32; 4] {
    let mut out = [0.0f32; 4];
    for p in 0..4 {
        let sp: Vec<usize> = pilots.sp_carriers(p).collect();
        let mut num = Complex32::new(0.0, 0.0);
        for pair in sp.windows(2) {
            let a = seg[pair[0]];
            let b = seg[pair[1]];
            let sign = if pilots.values[pair[0]] == pilots.values[pair[1]] { 1.0 } else { -1.0 };
            num += Complex32::new(sign, 0.0) * (b * a.conj());
        }
        out[p] = num.norm();
    }
    out
}

pub fn phase_scores(seg: &[Complex32], pilots: &SegmentPilots) -> [f32; 4] {
    [
        sp_coherence(seg, 0, pilots),
        sp_coherence(seg, 1, pilots),
        sp_coherence(seg, 2, pilots),
        sp_coherence(seg, 3, pilots),
    ]
}

/// 追従推定: SP位相は `symbol % 4` なので、前記号から次記号への
/// 遷移 `(p+1)%4` を弱い拘束として使い、SPスコアが曖昧な記号を安定させる。
pub fn track_symbol_phases(scores: &[[f32; 4]]) -> Vec<usize> {
    if scores.is_empty() { return Vec::new(); }
    let mut out: Vec<usize> = scores.iter().map(|r| {
        r.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0
    }).collect();
    for i in 1..out.len() {
        let expected = (out[i - 1] + 1) % 4;
        if out[i] != expected && scores[i][expected] + 1e-4 >= scores[i][out[i]] {
            out[i] = expected;
        }
    }
    out
}


pub fn detect_symbol_phase(seg: &[Complex32], pilots: &SegmentPilots) -> (usize, f32) {
    (0..4)
        .map(|p| (p, sp_coherence(seg, p, pilots)))
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tracked_phase_follows_periodic_transition_under_ambiguous_scores() {
        let scores = vec![[0.30, 0.29, 0.10, 0.05], [0.29, 0.30, 0.05, 0.04], [0.05, 0.04, 0.30, 0.29], [0.04, 0.05, 0.29, 0.30]];
        assert_eq!(track_symbol_phases(&scores), vec![0, 1, 2, 3]);
    }

    #[test]
    fn gr_isdbt_phase_score_uses_unnormalized_correlation_magnitude() {
        let pilots = SegmentPilots::center_1seg();
        let mut weak = vec![Complex32::new(0.0, 0.0); SEGMENT_CARRIERS];
        let mut strong = weak.clone();
        for p in 0..4 {
            for &l in pilots.sp_carriers(p).collect::<Vec<_>>().iter().skip(1) {
                let a = l - 12;
                weak[l] = Complex32::new(0.1 * pilots.values[l], 0.0);
                weak[a] = Complex32::new(0.1 * pilots.values[a], 0.0);
                strong[l] = Complex32::new(pilots.values[l], 0.0);
                strong[a] = Complex32::new(pilots.values[a], 0.0);
            }
        }
        let ws = phase_scores_gr_isdbt(&weak, &pilots);
        let ss = phase_scores_gr_isdbt(&strong, &pilots);
        assert!(ss[0] > ws[0] * 10.0);
    }

    #[test]
    fn gr_isdbt_style_phase_score_uses_known_prbs_relation() {
        let pilots = SegmentPilots::center_1seg();
        let mut seg = vec![Complex32::new(0.0, 0.0); SEGMENT_CARRIERS];
        for p in 0..1 {
            for &l in pilots.sp_carriers(p).collect::<Vec<_>>().iter().skip(1) {
                let prev = l - 12;
                seg[l] = Complex32::new(pilots.values[l], 0.0);
                seg[prev] = Complex32::new(pilots.values[prev], 0.0);
            }
        }
        let scores = phase_scores_gr_isdbt(&seg, &pilots);
        assert!(scores[0] > 0.99);
    }

    #[test]
    fn phase_scores_rank_the_injected_sp_phase_first() {
        let pilots = SegmentPilots::center_1seg();
        let mut seg = vec![Complex32::new(0.2, 0.1); SEGMENT_CARRIERS];
        for l in pilots.sp_carriers(2) {
            seg[l] = Complex32::new(pilots.values[l], 0.0);
        }
        let scores = phase_scores(&seg, &pilots);
        assert_eq!(detect_symbol_phase(&seg, &pilots).0, 2);
        assert!(scores[2] > scores[0]);
        assert!(scores[2] > scores[1]);
        assert!(scores[2] > scores[3]);
    }

    #[test]
    fn segment_bin_offset_is_296() {
        assert_eq!(SEGMENT_BIN_OFFSET, 296);
    }

    #[test]
    fn flat_channel_recovers_pilots() {
        // 平坦チャネル(=1)で、SP位置に正しいパイロット値を置けば H≈1、等化後も一致
        let pilots = SegmentPilots::center_1seg();
        let mut seg = vec![Complex32::new(0.3, -0.2); SEGMENT_CARRIERS]; // ダミーデータ
        for l in pilots.sp_carriers(0) {
            seg[l] = Complex32::new(pilots.values[l], 0.0); // チャネル1のSP
        }
        let h = estimate_channel(&seg, 0, &pilots);
        // SP位置のHは厳密に1
        for l in pilots.sp_carriers(0) {
            assert!(
                (h[l] - Complex32::new(1.0, 0.0)).norm() < 1e-5,
                "H[{l}] != 1"
            );
        }
        // コヒーレンスはほぼ1
        assert!(sp_coherence(&seg, 0, &pilots) > 0.999);
    }
}
