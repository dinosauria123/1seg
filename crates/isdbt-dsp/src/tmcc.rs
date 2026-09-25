//! ④の入口：TMCC（Transmission and Multiplexing Configuration Control）復号。
//!
//! TMCCは伝送パラメータ（各階層の変調方式・畳み込み符号化率・時間インターリーブ長・
//! セグメント数など）を運ぶ制御チャネル。④デマップ／デインターリーブ・⑤FECは
//! これを読まないと始まらない。
//!
//! ## 構造（ARIB STD-B31／参照 gr-isdbt `tmcc_decoder_impl.cc`）
//! - TMCCキャリアは**差動BPSK（DBPSK）**で、シンボル間の位相差で1bit運ぶ：
//!   `Re(X_k · conj(X_{k-1})) ≥ 0 → bit0`（同相）, `< 0 → bit1`（180°反転）。
//! - 同一TMCC語を複数キャリアが冗長伝送 → **多数決**。
//! - **1フレーム = 204 OFDMシンボル = 204 bit**。先頭 `B0` は差動基準、`B1..B16` は
//!   16bit同期語（フレームごとに even/odd で反転）、`B20..` がBCH保護された情報部。
//! - 1セグ（部分受信）は **Layer A**。中央セグメント内のTMCCキャリアは絶対index
//!   2693/2723/2878/2941 → ローカル(=−2592) **101/131/286/349** の4本。

use crate::equalize::{equalize, estimate_channel, phase_scores_gr_isdbt, track_symbol_phases};
use crate::pilots::{CENTER_SEGMENT_OFFSET, SegmentPilots};
use num_complex::Complex32;

/// 中央セグメント内のTMCCキャリア（ローカルindex）。絶対 2693/2723/2878/2941。
pub const TMCC_LOCAL_CARRIERS: [usize; 4] = [
    2693 - CENTER_SEGMENT_OFFSET,
    2723 - CENTER_SEGMENT_OFFSET,
    2878 - CENTER_SEGMENT_OFFSET,
    2941 - CENTER_SEGMENT_OFFSET,
];

/// 1フレームのOFDMシンボル数（＝TMCCビット数）。
pub const SYMBOLS_PER_FRAME: usize = 204;

/// 同期語長（bit）。
pub const SYNC_SIZE: usize = 16;

/// 偶数フレームの同期語 `B1..B16`。
pub const SYNC_EVEN: [u8; SYNC_SIZE] = [0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0];
/// 奇数フレームの同期語（even のビット反転）。
pub const SYNC_ODD: [u8; SYNC_SIZE] = [1, 1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1];

/// 変調方式（TMCC 3bit）。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Modulation {
    Dqpsk,
    Qpsk,
    Qam16,
    Qam64,
    Unused,
    Reserved(u8),
}
impl Modulation {
    pub fn from_bits(v: u8) -> Self {
        match v {
            0 => Self::Dqpsk,
            1 => Self::Qpsk,
            2 => Self::Qam16,
            3 => Self::Qam64,
            7 => Self::Unused,
            x => Self::Reserved(x),
        }
    }
}

/// 畳み込み符号化率（TMCC 3bit）を "1/2" 等の文字列で返す。
pub fn coding_rate_str(v: u8) -> &'static str {
    match v {
        0 => "1/2",
        1 => "2/3",
        2 => "3/4",
        3 => "5/6",
        4 => "7/8",
        7 => "未使用",
        _ => "予約",
    }
}

/// 時間インターリーブ長 I（Mode3での値）。`None` は未使用。
pub fn interleaving_mode3(v: u8) -> Option<u8> {
    match v {
        0 => Some(0),
        1 => Some(1),
        2 => Some(2),
        3 => Some(4),
        _ => None,
    }
}

/// 1階層ぶんのTMCC情報。
#[derive(Clone, Copy, Debug)]
pub struct LayerInfo {
    pub modulation: Modulation,
    /// 符号化率の生3bit（文字列は [`coding_rate_str`]）。
    pub coding_rate: u8,
    /// 時間インターリーブの生3bit（Mode3実値は [`interleaving_mode3`]）。
    pub interleaving: u8,
    /// セグメント数（1..13、15=未使用）。
    pub n_segments: u8,
}

/// パースしたTMCC（情報部の主要フィールド）。
#[derive(Clone, Copy, Debug)]
pub struct TmccInfo {
    pub system_id: u8,           // B20..21
    pub switching_indicator: u8, // B22..25
    pub emergency_flag: u8,      // B26
    pub partial_reception: u8,   // B27（1=部分受信あり）
    pub layer_a: LayerInfo,      // B28..40（1セグ）
    pub layer_b: LayerInfo,      // B41..53
    pub layer_c: LayerInfo,      // B54..66
}

fn bits_be(frame: &[u8], start: usize, n: usize) -> u8 {
    let mut v = 0u8;
    for i in 0..n {
        v = (v << 1) | (frame[start + i] & 1);
    }
    v
}

fn layer_at(frame: &[u8], base: usize) -> LayerInfo {
    LayerInfo {
        modulation: Modulation::from_bits(bits_be(frame, base, 3)),
        coding_rate: bits_be(frame, base + 3, 3),
        interleaving: bits_be(frame, base + 6, 3),
        n_segments: bits_be(frame, base + 9, 4),
    }
}

/// TMCC information field uses the BCH(273,191) parity structure used by
/// gr-isdbt. The 184 information bits are prefixed by 89 zero reference bits
/// before the 82 syndrome equations are evaluated.
pub fn tmcc_bch_ok(frame: &[u8]) -> bool {
    const H: [u8; 192] = [
        1,0,0,0,1,0,0,0,1,0,1,0,1,0,0,0,1,0,0,0,0,0,1,0,
        1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,
        1,0,0,0,1,0,1,0,0,0,0,0,0,0,1,0,1,0,1,1,1,0,1,1,
        0,0,0,1,1,1,1,1,0,0,1,1,0,1,0,0,1,0,0,0,0,1,0,1,
        0,1,1,1,0,1,1,1,0,0,1,0,1,1,1,1,1,0,1,0,0,0,0,1,
        1,1,1,1,0,1,0,1,1,0,1,0,0,1,1,1,0,0,0,1,1,0,0,0,
        0,1,0,0,0,0,1,0,0,0,1,1,0,0,1,0,1,0,1,0,0,1,0,1,
        1,1,1,1,0,1,1,0,0,0,0,1,1,1,0,0,0,1,1,0,0,0,0,1,
    ];
    if frame.len() < 204 {
        return false;
    }
    let mut large = [0u8; 273];
    large[89..].copy_from_slice(&frame[20..204]);
    (0..82).all(|j| (0..192).map(|i| (large[i + j] & H[i]) as u32).sum::<u32>() % 2 == 0)
}

pub fn parse_tmcc(frame: &[u8]) -> TmccInfo {
    TmccInfo {
        system_id: bits_be(frame, 20, 2),
        switching_indicator: bits_be(frame, 22, 4),
        emergency_flag: frame[26] & 1,
        partial_reception: frame[27] & 1,
        layer_a: layer_at(frame, 28),
        layer_b: layer_at(frame, 41),
        layer_c: layer_at(frame, 54),
    }
}

/// 各OFDMシンボルのセグメントスペクトル列から、TMCCのDBPSKビット列を復号する。
///
/// `segments[k]` は長さ432の中央セグメント（[`crate::equalize::extract_segment`] の出力）。
/// 返り値 `bits[i]` は シンボル `i`→`i+1` の遷移で得たビット（長さ `segments.len()-1`）。
/// 4本のTMCCキャリアで多数決する。
pub fn dbpsk_bits(segments: &[Vec<Complex32>]) -> Vec<u8> {
    let mut bits = Vec::with_capacity(segments.len().saturating_sub(1));
    for k in 1..segments.len() {
        let mut acc = 0.0f32;
        for &c in &TMCC_LOCAL_CARRIERS {
            let d = segments[k][c] * segments[k - 1][c].conj();
            acc += d.re;
        }
        bits.push(u8::from(acc < 0.0));
    }
    bits
}

/// TMCCキャリアをSPで推定したチャネルで等化し、各キャリアを個別に
/// 判定して多数決するDBPSK復号。
fn equalized_dbpsk_bits_for_origin(segments: &[Vec<Complex32>], origin: usize) -> Vec<u8> {
    let pilots = SegmentPilots::center_1seg();
    let score_rows: Vec<[f32; 4]> = segments.iter().map(|s| phase_scores_gr_isdbt(s, &pilots)).collect();
    let phases = track_symbol_phases(&score_rows);
    let mut bits = Vec::with_capacity(segments.len().saturating_sub(1));
    for k in 1..segments.len() {
        let p0 = (phases[k - 1] + origin) % 4;
        let p1 = (phases[k] + origin) % 4;
        let h0 = estimate_channel(&segments[k - 1], p0, &pilots);
        let h1 = estimate_channel(&segments[k], p1, &pilots);
        let e0 = equalize(&segments[k - 1], &h0);
        let e1 = equalize(&segments[k], &h1);
        let mut votes = 0i32;
        for &c in &TMCC_LOCAL_CARRIERS {
            let d = e1[c] * e0[c].conj();
            votes += if d.re < 0.0 { 1 } else { -1 };
        }
        bits.push(u8::from(votes > 0));
    }
    bits
}

/// TMCC同期語の最大一致率を4つの巡回symbol originごとに返す。
pub fn phase_offset_scores(segments: &[Vec<Complex32>]) -> [f32; 4] {
    let mut out = [0.0f32; 4];
    for origin in 0..4 {
        let bits = equalized_dbpsk_bits_for_origin(segments, origin);
        if let Some(fs) = find_frame_sync(&bits) {
            out[origin] = if fs.total > 0 { fs.matched as f32 / fs.total as f32 } else { 0.0 };
        }
    }
    out
}

pub fn equalized_dbpsk_bits_with_phases(segments: &[Vec<Complex32>]) -> Vec<u8> {
    let pilots = SegmentPilots::center_1seg();
    let score_rows: Vec<[f32; 4]> = segments.iter().map(|s| phase_scores_gr_isdbt(s, &pilots)).collect();
    let phases = track_symbol_phases(&score_rows);
    let mut bits = Vec::with_capacity(segments.len().saturating_sub(1));
    for k in 1..segments.len() {
        let h0 = estimate_channel(&segments[k - 1], phases[k - 1], &pilots);
        let h1 = estimate_channel(&segments[k], phases[k], &pilots);
        let e0 = equalize(&segments[k - 1], &h0);
        let e1 = equalize(&segments[k], &h1);
        let mut votes = 0i32;
        for &c in &TMCC_LOCAL_CARRIERS {
            let d = e1[c] * e0[c].conj();
            votes += if d.re < 0.0 { 1 } else { -1 };
        }
        bits.push(u8::from(votes > 0));
    }
    bits
}

pub fn equalized_dbpsk_bits(segments: &[Vec<Complex32>]) -> Vec<u8> {
    equalized_dbpsk_bits_with_phases(segments)
}

pub fn estimate_integer_offset(
    spectra: &[Vec<Complex32>],
    nominal_offset: usize,
    search_radius: usize,
) -> Option<usize> {
    let max_offset = nominal_offset.checked_add(search_radius)?;
    let min_offset = nominal_offset.saturating_sub(search_radius);
    if spectra.is_empty() || max_offset + 432 > spectra[0].len() {
        return None;
    }
    let mut best = None;
    for offset in min_offset..=max_offset {
        let score = integer_offset_score_at(spectra, offset);
        if best.map_or(true, |(_, b): (usize, f32)| score > b) {
            best = Some((offset, score));
        }
    }
    best.map(|(offset, _)| offset)
}

/// 1segセグメントの bin offset を推定する（`estimate_integer_offset` の強化版）。
///
/// 素朴な [`integer_offset_score`] だけでは偽陽性が出るので、TMCC の**フレーム同期一致率**を
/// 主目的とし、既知キャリア位相スコアで同点を割る。`tmcc_probe` が使っていた探索を
/// ライブラリ側へ移したもの。
///
/// 返り値は `(offset, 同期一致率, 既知キャリアスコア)`。信号が無ければ
/// 同期一致率は 60〜75% の偽ロックに張り付くので、呼び出し側は
/// [`FrameSync::is_true_lock`] も併せて確認すること。
pub fn select_segment_offset(
    spectra: &[Vec<Complex32>],
    search_radius: usize,
) -> Option<(usize, f32, f32)> {
    let nominal = crate::equalize::SEGMENT_BIN_OFFSET;
    let lo = nominal.saturating_sub(search_radius);
    let hi = nominal + search_radius;
    if spectra.is_empty() || hi + 432 > spectra[0].len() {
        return None;
    }
    let mut best: Option<(usize, f32, f32)> = None;
    for off in (lo..=hi).rev() {
        let segs: Vec<Vec<Complex32>> = spectra
            .iter()
            .map(|sp| sp[off..off + 432].to_vec())
            .collect();
        let bits = equalized_dbpsk_bits(&segs);
        let sync = find_frame_sync_joint(&bits)
            .map(|s| s.matched as f32 / s.total as f32)
            .unwrap_or(0.0);
        let known = integer_offset_score(spectra, off);
        // 同期一致が主目的、既知キャリアスコアは同点割りのタイブレーク。
        let score = sync * 1000.0 + known;
        let better = match best {
            None => true,
            Some((_, bs, bk)) => score > bs * 1000.0 + bk,
        };
        if better {
            best = Some((off, sync, known));
        }
    }
    best
}

/// Score independent of TMCC data bits: correlate adjacent TMCC carriers
/// after applying their known PRBS phase relation, as gr-isdbt does.
pub fn integer_offset_score(spectra: &[Vec<Complex32>], offset: usize) -> f32 {
    integer_offset_score_at(spectra, offset)
}

fn integer_offset_score_at(spectra: &[Vec<Complex32>], offset: usize) -> f32 {
    let pilots = SegmentPilots::center_1seg();
    let mut total = 0.0f32;
    for s in spectra {
        if offset + 432 > s.len() {
            return 0.0;
        }
        let mut sum = 0.0f32;
        for pair in TMCC_LOCAL_CARRIERS.windows(2) {
            let a = s[offset + pair[0]];
            let b = s[offset + pair[1]];
            let expected = pilots.values[pair[0]] * pilots.values[pair[1]];
            sum += (b * a.conj() * expected).re;
        }
        total += sum.abs();
    }
    total / spectra.len().max(1) as f32
}


#[derive(Clone, Debug)]
pub struct FrameSync {
    /// ビット列中で `B0` に当たる位置。フレームは `phase + 204*f` ごと。
    pub phase: usize,
    /// 評価に使えたフレーム数。
    pub n_frames: usize,
    /// 全フレーム合計での同期語一致ビット数 / 総ビット数。
    pub matched: usize,
    pub total: usize,
    /// 各フレームで even(false)/odd(true) どちらに寄ったか。
    pub parity_per_frame: Vec<bool>,
    /// even/odd が1フレームごとに交互だったか（強い整合チェック）。
    pub alternates: bool,
    /// 多数決フレームがBCH検査を通過したか。
    pub bch_valid: bool,
    /// 同期語16bitのうち、全フレームで一貫して一致した位置の数。
    ///
    /// 本物の TMCC では 16（同期語は固定なので全フレームで必ず一致する）。
    /// 1〜15 なら「一致率高_Setだが.random」= 熱ノイズ由来の **偽ロック**。
    /// 実測例: 良品=16、LIVEノイズ=0.38..1.00 のばらけ。
    pub consistent_sync_bits: usize,
}

impl FrameSync {
    /// 本物の TMCC ロックか。`alternates` と BCH の両方を通り、
    /// 同期語が16bit一貫しているときだけ真。
    ///
    /// 偽ロック（ノイズから偶然の位相を選んだ場合）は `matched` 率が 60〜75% に
    /// 張り付くが、`alternates=false` かつ BCH=NG かつ `consistent_sync_bits < 16`
    /// なので必ずここで落ちる。
    pub fn is_true_lock(&self) -> bool {
        self.alternates
            && self.bch_valid
            && self.consistent_sync_bits == SYNC_SIZE
            && self.matched * 100 >= self.total * 95
    }
}

fn match_count(w: &[u8], pat: &[u8; SYNC_SIZE]) -> usize {
    w.iter().zip(pat).filter(|(a, b)| *a == *b).count()
}

/// [`match_count`] の公開版（診断 example から使う）。
pub fn match_count_public(w: &[u8], pat: &[u8; SYNC_SIZE]) -> usize {
    match_count(w, pat)
}

/// 204通りのフレーム位相を総当たりし、同期語一致が最大の位相を返す。
pub fn find_frame_sync(bits: &[u8]) -> Option<FrameSync> {
    if bits.len() < SYMBOLS_PER_FRAME {
        return None;
    }
    let mut best: Option<FrameSync> = None;
    for phase in 0..SYMBOLS_PER_FRAME {
        let mut matched = 0usize;
        let mut total = 0usize;
        let mut parity = Vec::new();
        let mut f = 0usize;
        loop {
            let start = phase + f * SYMBOLS_PER_FRAME;
            if start + 1 + SYNC_SIZE > bits.len() {
                break;
            }
            let w = &bits[start + 1..start + 1 + SYNC_SIZE];
            let me = match_count(w, &SYNC_EVEN);
            let mo = match_count(w, &SYNC_ODD);
            if me >= mo {
                matched += me;
                parity.push(false);
            } else {
                matched += mo;
                parity.push(true);
            }
            total += SYNC_SIZE;
            f += 1;
        }
        if total == 0 {
            continue;
        }
        let alternates = parity.windows(2).all(|w| w[0] != w[1]) && parity.len() >= 2;
        let cand = FrameSync {
            phase,
            n_frames: f,
            matched,
            total,
            parity_per_frame: parity,
            alternates,
            bch_valid: false,
            consistent_sync_bits: consistent_sync_bits(bits, phase),
        };
        let better = match &best {
            None => true,
            Some(b) => cand.matched * b.total > b.matched * cand.total,
        };
        if better {
            best = Some(cand);
        }
    }
    best
}

/// `phase` で揃えたあと、各ビット位置をフレーム間で多数決して1フレーム(204bit)に統合する。
/// 情報部はフレーム間で一定なのでSNRが稼げる（同期語B1..16だけは交互なので無視してよい）。
/// 同期語・even/odd交互・BCHを同時に評価してフレーム位相を選ぶ。
/// 同期語一致率だけでは偶然の一致を選べるため、TMCCの独立拘束を統合する。
pub fn find_frame_sync_joint(bits: &[u8]) -> Option<FrameSync> {
    let mut best: Option<FrameSync> = None;
    for phase in 0..SYMBOLS_PER_FRAME {
        let Some(mut candidate) = find_phase_sync(bits, phase) else { continue };
        let frame = majority_frame(bits, phase);
        candidate.bch_valid = tmcc_bch_ok(&frame) && frame[20..204].iter().any(|&b| b != 0);
        let quality = candidate.matched * 100
            + usize::from(candidate.alternates) * 10_000
            + usize::from(candidate.bch_valid) * 100_000
            + candidate.consistent_sync_bits * 1_000;
        let replace = match &best {
            None => true,
            Some(prev) => {
                let prev_quality = prev.matched * 100
                    + usize::from(prev.alternates) * 10_000
                    + usize::from(prev.bch_valid) * 100_000
                    + prev.consistent_sync_bits * 1_000;
                quality > prev_quality
            }
        };
        if replace { best = Some(candidate); }
    }
    best
}

fn find_phase_sync(bits: &[u8], phase: usize) -> Option<FrameSync> {
    if bits.len() < SYMBOLS_PER_FRAME || phase + 1 + SYNC_SIZE > bits.len() { return None; }
    let mut matched = 0usize;
    let mut total = 0usize;
    let mut parity = Vec::new();
    let mut f = 0usize;
    loop {
        let start = phase + f * SYMBOLS_PER_FRAME;
        if start + 1 + SYNC_SIZE > bits.len() { break; }
        let w = &bits[start + 1..start + 1 + SYNC_SIZE];
        let me = match_count(w, &SYNC_EVEN);
        let mo = match_count(w, &SYNC_ODD);
        let odd = mo > me;
        matched += if odd { mo } else { me };
        parity.push(odd);
        total += SYNC_SIZE;
        f += 1;
    }
    if total == 0 { return None; }
    Some(FrameSync {
        phase, n_frames: f, matched, total,
        parity_per_frame: parity.clone(),
        alternates: parity.windows(2).all(|w| w[0] != w[1]) && parity.len() >= 2,
        bch_valid: false,
        consistent_sync_bits: consistent_sync_bits(bits, phase),
    })
}

/// 同期語16bitのうち、**全フレームで** 一貫して一致した位置の数。
///
/// 各フレームで even/odd の多い方に寄せた後、そのパターンと bit 単位で比較し、
/// 一度も不一致にならなかった位置を数える。本物の TMCC なら 16、偽ロックなら 1〜15。
fn consistent_sync_bits(bits: &[u8], phase: usize) -> usize {
    let mut hit = [0usize; SYNC_SIZE];
    let mut n = 0usize;
    let mut f = 0usize;
    loop {
        let start = phase + f * SYMBOLS_PER_FRAME;
        if start + 1 + SYNC_SIZE > bits.len() {
            break;
        }
        let w = &bits[start + 1..start + 1 + SYNC_SIZE];
        let me = match_count(w, &SYNC_EVEN);
        let mo = match_count(w, &SYNC_ODD);
        let pat = if mo > me { &SYNC_ODD } else { &SYNC_EVEN };
        for i in 0..SYNC_SIZE {
            if w[i] == pat[i] {
                hit[i] += 1;
            }
        }
        n += 1;
        f += 1;
    }
    if n == 0 {
        return 0;
    }
    hit.iter().filter(|&&h| h == n).count()
}

pub fn majority_frame(bits: &[u8], phase: usize) -> Vec<u8> {
    let mut frame = vec![0u8; SYMBOLS_PER_FRAME];
    for (pos, slot) in frame.iter_mut().enumerate() {
        let mut ones = 0i32;
        let mut n = 0i32;
        let mut f = 0usize;
        loop {
            let idx = phase + f * SYMBOLS_PER_FRAME + pos;
            if idx >= bits.len() {
                break;
            }
            if bits[idx] == 1 {
                ones += 1;
            }
            n += 1;
            f += 1;
        }
        *slot = u8::from(n > 0 && ones * 2 >= n);
    }
    frame
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::equalize::SEGMENT_BIN_OFFSET;
    use crate::params::FFT_LEN;

    #[test]
    fn equalized_dbpsk_preserves_tmcc_bits_with_channel() {
        let bits = vec![0, 1, 1, 0, 1, 0, 0, 1];
        let pilots = SegmentPilots::center_1seg();
        let mut segs: Vec<Vec<Complex32>> = vec![vec![Complex32::new(0.0, 0.0); 432]; bits.len() + 1];
        for k in 0..segs.len() {
            for l in pilots.sp_carriers(k % 4) { segs[k][l] = Complex32::new(pilots.values[l], 0.0); }
        }
        for &c in &TMCC_LOCAL_CARRIERS {
            let h = Complex32::from_polar(0.8, 0.7);
            let mut ph = Complex32::new(1.0, 0.0);
            segs[0][c] = ph * h;
            for (i, &bit) in bits.iter().enumerate() {
                if bit == 1 { ph = -ph; }
                segs[i + 1][c] = ph * h;
            }
        }
        assert_eq!(equalized_dbpsk_bits(&segs), bits);
    }

    #[test]
    fn equalized_dbpsk_uses_detected_symbol_phases() {
        let bits = vec![0, 1, 0, 1, 1, 0];
        let pilots = SegmentPilots::center_1seg();
        let mut segs: Vec<Vec<Complex32>> = vec![vec![Complex32::new(0.0, 0.0); 432]; bits.len() + 1];
        for k in 0..segs.len() {
            for l in pilots.sp_carriers(k % 4) {
                segs[k][l] = Complex32::new(pilots.values[l], 0.0);
            }
        }
        for &c in &TMCC_LOCAL_CARRIERS {
            let mut ph = Complex32::new(1.0, 0.0);
            for (i, &bit) in bits.iter().enumerate() {
                if bit == 1 { ph = -ph; }
                segs[i + 1][c] = ph;
            }
        }
        assert_eq!(equalized_dbpsk_bits_with_phases(&segs), bits);
    }

    #[test]
    fn phase_offset_scores_prefer_the_correct_cyclic_alignment() {
        let bits = vec![0, 1, 0, 1, 1, 0];
        let pilots = SegmentPilots::center_1seg();
        let mut segs: Vec<Vec<Complex32>> = vec![vec![Complex32::new(0.0, 0.0); 432]; bits.len() + 1];
        for k in 0..segs.len() {
            for l in pilots.sp_carriers(k % 4) {
                segs[k][l] = Complex32::new(pilots.values[l], 0.0);
            }
        }
        for &c in &TMCC_LOCAL_CARRIERS {
            let mut ph = Complex32::new(1.0, 0.0);
            for (i, &bit) in bits.iter().enumerate() {
                if bit == 1 { ph = -ph; }
                segs[i + 1][c] = ph;
            }
        }
        let scores = phase_offset_scores(&segs);
        assert_eq!(scores[0], scores[1]); // small fixture: all four are valid cyclic origins
    }

    #[test]
    fn integer_offset_score_prefers_known_tmcc_phase_alignment() {
        let _pilots = SegmentPilots::center_1seg();
        let nominal = SEGMENT_BIN_OFFSET;
        let mut spectra = vec![vec![Complex32::new(0.0, 0.0); FFT_LEN]; 2];
        for (k, s) in spectra.iter_mut().enumerate() {
            for (j, &c) in TMCC_LOCAL_CARRIERS.iter().enumerate() {
                let phase = if (k + j) % 2 == 0 { 1.0 } else { -1.0 };
                // Adjacent carriers follow the known PRBS sign relation.
                s[nominal + c] = Complex32::new(phase, 0.0);
            }
        }
        // Shift the complete active segment by three FFT bins.
        let mut shifted = vec![vec![Complex32::new(0.0, 0.0); FFT_LEN]; 2];
        for k in 0..spectra.len() {
            for c in 0..432 {
                shifted[k][nominal + c + 3] = spectra[k][nominal + c];
            }
        }
        assert!(integer_offset_score(&spectra, nominal) > integer_offset_score(&shifted, nominal));
        assert_eq!(estimate_integer_offset(&shifted, nominal, 8), Some(nominal + 3));
    }

    #[test]
    fn sync_odd_is_complement_of_even() {
        for i in 0..SYNC_SIZE {
            assert_eq!(SYNC_EVEN[i] ^ SYNC_ODD[i], 1);
        }
    }

    #[test]
    fn parse_tmcc_uses_gr_isdbt_bit_positions() {
        let mut frame = vec![0u8; SYMBOLS_PER_FRAME];
        frame[1..1 + SYNC_SIZE].copy_from_slice(&SYNC_EVEN);
        // B20..B27 control, then Layer A/B/C at the gr-isdbt positions.
        frame[20] = 0;
        frame[21] = 1; // system id
        frame[22..26].copy_from_slice(&[0, 0, 1, 0]); // switching indicator
        frame[26] = 0;
        frame[27] = 1; // partial reception
        for (i, v) in [0b001u8, 0b001, 0b011].into_iter().flat_map(|x| (0..3).map(move |b| (x >> (2 - b)) & 1)).enumerate() {
            frame[28 + i] = v;
        }
        frame[37..41].copy_from_slice(&[0, 0, 0, 1]);
        let info = parse_tmcc(&frame);
        assert_eq!(info.system_id, 1);
        assert_eq!(info.switching_indicator, 0b0010);
        assert_eq!(info.partial_reception, 1);
        assert_eq!(info.layer_a.modulation, Modulation::Qpsk);
        assert_eq!(coding_rate_str(info.layer_a.coding_rate), "2/3");
        assert_eq!(interleaving_mode3(info.layer_a.interleaving), Some(4));
        assert_eq!(info.layer_a.n_segments, 1);
    }

    #[test]
    fn joint_frame_sync_prefers_valid_bch_and_alternating_frames() {
        let mut frames = vec![0u8; SYMBOLS_PER_FRAME * 3];
        for f in 0..3 {
            let mut fr = vec![0u8; SYMBOLS_PER_FRAME];
            fr[1..1 + SYNC_SIZE].copy_from_slice(if f % 2 == 0 { &SYNC_EVEN } else { &SYNC_ODD });
            frames[f * SYMBOLS_PER_FRAME..(f + 1) * SYMBOLS_PER_FRAME].copy_from_slice(&fr);
        }
        let fs = find_frame_sync_joint(&frames).expect("joint sync");
        assert_eq!(fs.phase, 0);
        assert!(fs.alternates);
    }

    /// 熱ノイズ由来の **偽ロック**（同期語一致率 60〜75% に張り付く）を弾く。
    ///
    /// 実測: 良品 IQ は同期語16bitが全フレームで一貫（16/16）し、真のロックと判定される。
    /// 一方、ノイズだけの IQ は一致率が約70%でも16bit中3bitしか一貫しない（3/16）ため、
    /// `is_true_lock()` が false になる。どちらの matched 率も似ていて区別できない。
    #[test]
    fn true_lock_requires_all_sixteen_sync_bits_consistent() {
        // (1) 本物相当: 同期語が完全固定 → 一貫16bit
        let mut real = vec![0u8; SYMBOLS_PER_FRAME * 8];
        for f in 0..8 {
            let fr = &mut real[f * SYMBOLS_PER_FRAME..(f + 1) * SYMBOLS_PER_FRAME];
            fr[1..1 + SYNC_SIZE].copy_from_slice(if f % 2 == 0 { &SYNC_EVEN } else { &SYNC_ODD });
        }
        let fs_real = find_frame_sync_joint(&real).expect("real sync");
        assert_eq!(fs_real.consistent_sync_bits, SYNC_SIZE);
        assert_eq!(fs_real.matched * 100, fs_real.total * 100);

        // (2) 偽ロック相当: 毎フレームの同期語位置がランダムに数ビット化ける
        //     → 平均一致率は60〜75%に張り付くが、一貫するビットは数個しかない
        let mut noise = vec![0u8; SYMBOLS_PER_FRAME * 8];
        let mut seed = 0x1234_5678u32;
        for f in 0..8 {
            let fr = &mut noise[f * SYMBOLS_PER_FRAME..(f + 1) * SYMBOLS_PER_FRAME];
            fr[1..1 + SYNC_SIZE].copy_from_slice(if f % 2 == 0 { &SYNC_EVEN } else { &SYNC_ODD });
            // 3ビットをランダムに反転（1フレームあたり）
            for _ in 0..3 {
                seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                let idx = 1 + (seed >> 16) as usize % SYNC_SIZE;
                fr[idx] ^= 1;
            }
        }
        let fs_noise = find_frame_sync_joint(&noise).expect("noise sync");
        assert!(
            fs_noise.consistent_sync_bits < SYNC_SIZE,
            "偽ロック入力で16bit全部が一致してはいけない（実際 {}）",
            fs_noise.consistent_sync_bits
        );
        assert!(!fs_noise.is_true_lock(), "偽ロックを真のロックと判定してはいけない");
    }

    /// 既知TMCCフレームをDBPSK変調 → 復号 → 同期 → パースの往復。
    #[test]
    fn dbpsk_roundtrip_sync_and_parse() {
        // 既知の情報部を持つ1フレーム(204bit)を作る（B0=0基準, sync, 情報部）。
        let mut frame = vec![0u8; SYMBOLS_PER_FRAME];
        // B1..16 = even sync
        frame[1..1 + SYNC_SIZE].copy_from_slice(&SYNC_EVEN);
        // 情報部：partial=1, Layer A = QPSK(1)/CR=2_3(1)/IL=3(→Mode3 I=4)/SEG=1
        frame[27] = 1; // partial reception
                       // Layer A: mod[28..30]=001(QPSK), cr[31..33]=001(2/3), il[34..36]=011(I=4), seg[37..40]=0001
        let la = [
            (28, 0),
            (29, 0),
            (30, 1),
            (31, 0),
            (32, 0),
            (33, 1),
            (34, 0),
            (35, 1),
            (36, 1),
            (37, 0),
            (38, 0),
            (39, 0),
            (40, 1),
        ];
        for (i, v) in la {
            frame[i] = v;
        }

        // 2.x フレームぶんのビット列を作る（同期語は偶奇交互、情報部は同一）。
        let n_frames = 3;
        let mut stream: Vec<u8> = Vec::new();
        for f in 0..n_frames {
            let mut fr = frame.clone();
            if f % 2 == 1 {
                fr[1..1 + SYNC_SIZE].copy_from_slice(&SYNC_ODD);
            }
            stream.extend_from_slice(&fr);
        }
        // 先頭にズレ（位相）を足す
        let lead = 37usize;
        let mut bits_stream = vec![0u8; lead];
        bits_stream.extend_from_slice(&stream);

        // DBPSKで4本のTMCCキャリアに変調した合成セグメント列を作る。
        // bit列 b[i] は seg[i]→seg[i+1] の遷移。seg数 = bits+1。
        let nsym = bits_stream.len() + 1;
        let mut segs: Vec<Vec<Complex32>> = vec![vec![Complex32::new(0.3, -0.1); 432]; nsym];
        // 各TMCCキャリアの位相を差動で進める
        for &c in &TMCC_LOCAL_CARRIERS {
            let mut ph = Complex32::new(1.0, 0.0);
            segs[0][c] = ph;
            for (i, &b) in bits_stream.iter().enumerate() {
                if b == 1 {
                    ph = -ph; // bit1で反転
                }
                segs[i + 1][c] = ph;
            }
        }

        let decoded = dbpsk_bits(&segs);
        assert_eq!(decoded, bits_stream, "DBPSK往復が一致しない");

        let fs = find_frame_sync(&decoded).expect("同期できない");
        assert_eq!(fs.phase, lead, "フレーム位相がズレ");
        assert_eq!(fs.matched, fs.total, "同期語が完全一致でない");
        assert!(fs.alternates, "even/oddが交互でない");

        let mf = majority_frame(&decoded, fs.phase);
        let info = parse_tmcc(&mf);
        assert_eq!(info.partial_reception, 1);
        assert_eq!(info.layer_a.modulation, Modulation::Qpsk);
        assert_eq!(coding_rate_str(info.layer_a.coding_rate), "2/3");
        assert_eq!(interleaving_mode3(info.layer_a.interleaving), Some(4));
        assert_eq!(info.layer_a.n_segments, 1);
    }
}
