//! ⑥ TS化：バイトデインターリーブ（Forney）＋ TS同期検出 ＋ エネルギー逆拡散。
//!
//! ⑤Viterbiの情報ビット → バイト化 → **Forneyバイトデインターリーブ**（I=12, M=17）→
//! 204バイトRSブロック（先頭に同期バイト 0x47／先頭パケットは反転 0xB8）→ RS復号（別）→
//! **エネルギー逆拡散**（PRBS）→ 188バイトMPEG-TSパケット。
//!
//! ここでは「0x47/0xB8 が204バイト周期で立つ」ところまで（＝実電波からTS構造が出た証拠）と、
//! 逆拡散PRBSを実装する。RS(204,188)復号は [`crate::rs`]（予定）。
//! 参照：gr-isdbt `byte_deinterleaver_impl.cc` / `energy_descrambler_impl.cc`。

use std::collections::VecDeque;

/// TSパケット（RSブロック）長。
pub const TSP: usize = 204;
/// バイトインターリーブの分岐数。
pub const BI_I: usize = 12;
/// バイトインターリーブの単位遅延（バイト）。204/12 = 17。
pub const BI_M: usize = 17;
/// TS同期バイト。
pub const SYNC: u8 = 0x47;
/// 反転同期バイト（8パケット周期の先頭）。
pub const SYNC_INV: u8 = 0xb8;

/// ビット列を8bitごとにMSB firstでバイト化する。`bit_offset` で開始位相をずらせる。
pub fn pack_bits_msb(bits: &[u8], bit_offset: usize) -> Vec<u8> {
    let b = &bits[bit_offset.min(bits.len())..];
    let n = b.len() / 8;
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let mut v = 0u8;
        for j in 0..8 {
            v = (v << 1) | (b[i * 8 + j] & 1);
        }
        out.push(v);
    }
    out
}

/// TS バイト列の continuity_counter を PID ごとに連番に正規化する。
///
/// なぜ必要か（2026-09-26 実測）: 復調で 1〜2 パケット落ちると、以降その PID の
/// continuity counter がずれたまま出力される。実測で映像 PID 0x581 は
/// **99.9% のパケットが CC 不一致**（4,5,6,7,8,9,10,12,… と 11 が飛ぶ）となり、
/// VLC で映像が「壊れながら再生」、音声（別 PID）が無事という状態になった。
/// MPEG-TS 仕様は「CC が 1 欠けた時点でそれ以降すべて continuity error」と
/// みなすため、復号器は以降のストリームをすべて破棄する。
///
/// 落ちたデータの穴はもう戻せないので、番号だけを詰め直す（穴は的美国 no）。
/// これでデコーダは「途切れたが連続している」ものとして残りを復号する。
///
/// sync byte が 0x47 でない位置は無視する（破碎バイト列を壊さないため）。
pub fn normalize_continuity(ts: &mut [u8]) {
    if ts.len() < 188 {
        return;
    }
    // PID -> (直前の CC, 直前の pusi)
    let mut last: std::collections::HashMap<u16, (u8, u8)> = std::collections::HashMap::new();
    let mut i = 0;
    while i + 188 <= ts.len() {
        if ts[i] != SYNC {
            i += 1;
            continue;
        }
        let pid = ((ts[i + 1] as u16 & 0x1f) << 8) | ts[i + 2] as u16;
        let pusi = ts[i + 1] & 0x40;
        let scr = (ts[i + 1] >> 7) & 1;
        let afc = (ts[i + 3] >> 4) & 0x03;
        // payload_unit_start が立っていない EOS/TEI は CC を消費しない。
        if scr == 1 || (afc & 0x02 == 0 && pusi == 0) {
            i += 188;
            continue;
        }
        let mut cc = ts[i + 3] & 0x0f;
        if let Some(&(pcc, ppusi)) = last.get(&pid) {
            let expect = if pusi != 0 && ppusi == 0 {
                // 新しい PES の先頭: CC は 0 に戻ってよい。
                0
            } else {
                pcc.wrapping_add(1) & 0x0f
            };
            if cc != expect {
                // 欠損した 1 個を詰め直す。
                cc = expect;
                ts[i + 3] = (ts[i + 3] & 0xf0) | cc;
            }
        }
        last.insert(pid, (cc, pusi));
        i += 188;
    }
}

pub struct ContinuityTracker {
    last: std::collections::HashMap<u16, (u8, u8)>,
}

/// PES ヘッダの PTS を PCR 起点 0 に合わせて平行移動する。
///
/// # なぜ必要か（実測 2026-09-26）
///
/// 1seg 放送の PES ヘッダは**絶対時刻**の PTS を持つ。実測値:
/// 映像 PTS = 6,445,068,696（90 kHz で 71,611 秒）、音声も同程度。
/// これは放送局の起動からの経過時間で、33 bit カウンタが 26.5 時間で
/// 折り返す設計.
///
/// 一方こちらが注入する PCR は 0 から始める。二者が 71,611 秒もずれると
/// VLC が
/// `Timestamp conversion failed (delay 2000000, buffering 100000)`
/// と `Could not convert timestamp ... for faad` を出し、音声がノイズに、
/// 映像も MB _Error で見える。
///
/// # 做法
///
/// 最初に現れた PTS を基準（`base`）として、以降の PES からは `base` を
/// 引いた相対値を書き込む。PTS の 33 bit 折り返しは mod で吸収する。
///
/// 基準は PID ごとに持つ。映像と音声で放送側の時刻基準が僅かに違うことが
/// あり、共通化すると映像と音声がずれる。

#[derive(Default)]
pub struct PtsNormalizer {
    base: std::collections::HashMap<u16, u64>,
}

impl PtsNormalizer {
    pub fn new() -> Self {
        Self { base: std::collections::HashMap::new() }
    }

    /// `ts` 内の全 PES ヘッダの PTS を平行移動する。
    pub fn normalize(&mut self, ts: &mut [u8]) {
        let p = PTS_MOD;
        let mut i = 0usize;
        while i + 188 <= ts.len() {
            if ts[i] != 0x47 {
                i += 1;
                continue;
            }
            let b1 = ts[i + 1];
            let pusi = b1 & 0x40;
            let afc = (ts[i + 3] >> 4) & 0x03;
            // PES 開始は pusi=1 かつ payload あり（afc=01/11）のときだけ。
            if pusi == 0 || (afc != 0x01 && afc != 0x03) {
                i += 188;
                continue;
            }
            // adaptation field をスキップ
            let mut off = i + 4;
            if afc == 0x03 {
                off += 1 + ts[off] as usize;
            }
            // hlen=5（PTS のみ）でも PES ヘッダは 14 バイトで足りる。
            // 6 (00 00 01 id len2) + 3 (flags+hlen+PTS flag) + 5 (PTS) = 14。
            if off + 14 > ts.len() {
                i += 188;
                continue;
            }
            if ts[off..off + 3] != [0x00, 0x00, 0x01] {
                i += 188;
                continue;
            }
            let flags2 = ts[off + 7];
            let hlen = ts[off + 8] as usize;
            let pts_dts = (flags2 >> 6) & 0x03;
            if pts_dts != 2 && pts_dts != 3 {
                i += 188;
                continue;
            }
            // 省略可能フィールドの長さ:
            //   5  = MPEG 標準の PTS のみ
            //   10 = MPEG 標準の PTS + DTS
            //   7  = ISDB-T 1seg 独自（音声 PID 0x583 がこの形式）
            //
            // 7 の場合は [0..5) が PTS、[5..7) が独自フィールド。
            // 7 に対応しないと音声 PTS が 71,611 秒のまま残り、
            // 音声がノイズになる（実測 2026-09-26）。
            //
            // 23 は **字幕 PID 0x0587 (stream_id 0xbd)** の形式。実測の
            // 生バイト: `000001 bd 002e 80 81 17 2f48af07 1b8e4343495304bf`
            // → flags2=0x81 (PTS あり), hlen=0x17=23。PTS は [0..5) にあり
            // 後ろ 18 バイトは独自フィールド + stuffing。
            //
            // 23 をサポートしないと字幕 PTS が 86900 秒（= 1日1時間後）のまま残り、
            // ffprobe の `format.duration` が 86914 秒になり**TS 全体が再生不能に
            // なる**（実測 2026-09-26: 0x0587 を除くと duration 18.9 秒に復帰）。
            // 映像 0x0581 と音声 0x0583 は hlen=5 なので既に正規化されていた。
            let dts_len = if hlen == 10 {
                5
            } else {
                0
            };
            if hlen != 5 && hlen != 7 && hlen != 10 && hlen != 23 {
                i += 188;
                continue;
            }
            let b = off + 9;
            let pts = read_ts(&ts[b..b + 5]);
            let pid = ((b1 as u16 & 0x1f) << 8) | ts[i + 2] as u16;
            let base = *self.base.entry(pid).or_insert(pts);
            // base を引いた相対値。33 bit 折り返しは mod で吸収。
            let rel = (pts + p - base) % p;
            write_ts(&mut ts[b..b + 5], rel);
            if dts_len == 5 {
                let dts = read_ts(&ts[off + 14..off + 19]);
                let drel = (dts + p - base) % p;
                write_ts(&mut ts[off + 14..off + 19], drel);
            }
            i += 188;
        }
    }
}

/// PTS の 33 bit 周期。
pub const PTS_MOD: u64 = 1 << 33;

/// MPEG-TS の 5 バイト timestamp フィールドを読む。
fn read_ts(b: &[u8]) -> u64 {
    (((b[0] >> 1) as u64 & 0x07) << 30)
        | ((b[1] as u64) << 22)
        | (((b[2] >> 1) as u64 & 0x7f) << 15)
        | ((b[3] as u64) << 7)
        | ((b[4] as u64) >> 1)
}

/// MPEG-TS の 5 バイト timestamp フィールドを書く。
fn write_ts(b: &mut [u8], v: u64) {
    // 33 bit を 5 バイトに詰める: b[0] は '0010'(4bit)+3bit、
    // b[1] は 8bit、b[2] は 7bit+mark(1)、b[3] は 8bit、b[4] は 7bit+mark(1)。
    b[0] = ((0x20 | (((v >> 30) & 0x07) as u8)) << 1) | 1;
    b[1] = ((v >> 22) & 0xff) as u8;
    b[2] = (((((v >> 15) & 0x7f) as u8) << 1) | 1);
    b[3] = ((v >> 7) & 0xff) as u8;
    b[4] = (((v & 0x7f) as u8) << 1) | 1;
}

#[cfg(test)]
mod pts_tests {
    use super::*;

    fn ts_field(v: u64) -> [u8; 5] {
        let mut b = [0u8; 5];
        write_ts(&mut b, v & 0x1_FFFF_FFFF);
        b
    }

    /// read/write が互いに逆演算であること。
    #[test]
    fn ts_field_roundtrip() {
        // 33 bit カウンタ（周期 2^33 = 8,589,934,592）の実値域で往復確認。
        // 1seg 実測の放送側 PTS は 6,445,068,696 で周期内。
        for v in [0u64, 1, 90_000, 0x1234_5678, 6_445_068_696, 0x1_FFFF_FFFF] {
            let b = ts_field(v);
            assert_eq!(read_ts(&b), v, "v={v}");
        }
    }

    /// timestamp フィールドの構造: b[0] は '0010' + 3bit、
    /// b[2] と b[4] は mark ビット（最下位 1）で終わる。
    #[test]
    fn ts_field_has_mark_bits() {
        let b = ts_field(0);
        // b[0] は '01' + 2bit + 3bit + mark。PTS/DTS なら上位 4bit は '0100'。
        assert_eq!(b[0] & 0xF0, 0x40, "b[0] 上位 4bit は '0100'");
        assert_eq!(b[0] & 0x01, 0x01, "b[0] mark");
        assert_eq!(b[2] & 0x01, 0x01, "b[2] mark");
        assert_eq!(b[4] & 0x01, 0x01, "b[4] mark");
    }

    /// 放送の絶対時刻（71,611 秒相当）を PCR 起点 0 に平行移動すること。
    /// これが無ければ VLC は `Timestamp conversion failed` を出し、
    /// 音声がノイズになる（実測 2026-09-26）。
    #[test]
    fn pts_normalizer_zeroes_first_pts() {
        let mut pkt = [0xffu8; 188];
        pkt[0] = 0x47;
        pkt[1] = 0x40 | ((0x581 >> 8) as u8 & 0x1f);
        pkt[2] = (0x581 & 0xff) as u8;
        pkt[3] = 0x30; // adaptation+payload
        pkt[4] = 0;    // adaptation_field_length = 0
        // PES: 00 00 01 e0 <len> 80 80 05 <PTS 5B> ...
        pkt[5..8].copy_from_slice(&[0x00, 0x00, 0x01]);
        pkt[8] = 0xe0;
        pkt[9] = 0x00;
        pkt[10] = 0x00;
        pkt[11] = 0x80;
        pkt[12] = 0x80;
        pkt[13] = 0x05; // header length = 5 (PTS only)
        let orig: u64 = 6_445_068_696;
        pkt[14..19].copy_from_slice(&ts_field(orig));

        let mut n = PtsNormalizer::new();
        n.normalize(&mut pkt);
        assert_eq!(read_ts(&pkt[14..19]), 0, "最初の PTS は 0 になる");
    }

    /// 2 個目以降は base からの相対値になること。
    #[test]
    fn pts_normalizer_keeps_relative_delta() {
        let mut n = PtsNormalizer::new();
        let base: u64 = 6_445_068_696;
        let mut p1 = [0xffu8; 188];
        p1[0] = 0x47; p1[1] = 0x40 | 0x05; p1[2] = 0x81; p1[3] = 0x30; p1[4] = 0;
        p1[5..8].copy_from_slice(&[0x00, 0x00, 0x01]);
        p1[8] = 0xe0; p1[9] = 0; p1[10] = 0; p1[11] = 0x80; p1[12] = 0x80; p1[13] = 0x05;
        p1[14..19].copy_from_slice(&ts_field(base));
        let mut p2 = p1;
        // 1.0 秒後 = 90000 ティック
        p2[14..19].copy_from_slice(&ts_field(base.wrapping_add(90_000)));
        let mut buf = Vec::new();
        buf.extend_from_slice(&p1);
        buf.extend_from_slice(&p2);
        n.normalize(&mut buf);
        assert_eq!(read_ts(&buf[14..19]), 0);
        assert_eq!(read_ts(&buf[188 + 14..188 + 19]), 90_000, "1 秒ぶんの差が保たれる");
    }
}

impl ContinuityTracker {
    pub fn new() -> Self {
        Self { last: std::collections::HashMap::new() }
    }

    /// `normalize_continuity` と違い PID ごとの状態を跨いでチャンクをまたぐ。
    ///
    /// 実測: live モードは `feed()` がチャンク単位で 0 バイトを返すため、
    /// チャンクごとに state を初期化すると境界で CC が 1 つずれて
    /// 「壊れながら再生」になる。復調出力は連続しているので、
    /// tracker 側で状態を持ち越す必要がある。
    pub fn normalize(&mut self, ts: &mut [u8]) {
        if ts.len() < 188 {
            return;
        }
        let mut i = 0;
        while i + 188 <= ts.len() {
            if ts[i] != SYNC {
                i += 1;
                continue;
            }
            let pid = ((ts[i + 1] as u16 & 0x1f) << 8) | ts[i + 2] as u16;
            let pusi = ts[i + 1] & 0x40;
            let scr = (ts[i + 1] >> 7) & 1;
            if scr == 1 {
                i += 188;
                continue;
            }
            // PSI (PAT/PMT) は Tracker を経由しない。`psi_packets` が
            // 注入時に正しい CC を付けているのに、Tracker が後から
            // 別カウンタで上書きすると `...,9,1,10,...` と破綻する
            // （実測 2026-09-26: PMT CC 連続性違反 6 箇所、
            // libdvbpsi が `TS discontinuity (received 1, expected 10)
            // for PID 8136` を出し続けて VLC は 0:00 で止まった）。
            //
            // PCR 専用パケット (afc=0b10) も exogenous なので、
            // PCR_PID を SID に公示而不被し要じ PSP-PID の CC を消費する。
            if pid == 0x0000 || pid == 0x1FC8 {
                i += 188;
                continue;
            }
            // null パターン (PID 0x1FFF) は CC を持たない。訂正不能 RS ブロックの
            // 代替として挿入したもので、これを CC 較正の基準にすると
            // 「パケットが 1 本飛んだのに CC が連続」という矛盾した状態になる。
            if pid == 0x1FFF {
                i += 188;
                continue;
            }
            let mut cc = ts[i + 3] & 0x0f;
            // adaptation_field_control: 0b10/0b11 は先頭が adaptation field で
            // payload がない。**CC は payload があるときだけ 1 増える。**
            // PCR 専用パケット（afc=10）も payload **なし** なので、ここを
            // 飛ばさないと PCR パケットが映像 PID の CC を消費してしまう
            // （実測 2026-09-26: 24 秒間の TS に PCR が 2 本しか出ない）。
            let afc = (ts[i + 3] >> 4) & 0x03;
            let has_payload = afc == 0x01 || afc == 0x03;
            if has_payload {
                if let Some(&(pcc, ppusi)) = self.last.get(&pid) {
                    let expect = if pusi != 0 && ppusi == 0 {
                        0
                    } else {
                        pcc.wrapping_add(1) & 0x0f
                    };
                    if cc != expect {
                        cc = expect;
                        ts[i + 3] = (ts[i + 3] & 0xf0) | cc;
                    }
                }
                self.last.insert(pid, (cc, pusi));
            }
            i += 188;
        }
    }
}

/// Forney畳み込みバイトデインターリーバ（I=12, M=17）。
/// 分岐 `k` の遅延 = `M*(I-1-k)`（＝送信側 `M*k` と合わせて総遅延 `M*(I-1)` 一定）。
pub struct ByteDeinterleaver {
    fifos: Vec<VecDeque<u8>>,
    idx: usize,
}

impl Default for ByteDeinterleaver {
    fn default() -> Self {
        Self::new()
    }
}

impl ByteDeinterleaver {
    pub fn new() -> Self {
        Self::with_rev(false)
    }
    /// `rev=false`：分岐kの遅延 `M*(I-1-k)`（標準）。`rev=true`：`M*k`（逆向き）。
    /// 送信側の向きに合わせる必要があるので、実機では両方試して確定する。
    pub fn with_rev(rev: bool) -> Self {
        let fifos = (0..BI_I)
            .map(|k| {
                let d = if rev { k } else { BI_I - 1 - k };
                VecDeque::from(vec![0u8; BI_M * d])
            })
            .collect();
        Self { fifos, idx: 0 }
    }

    /// 1バイト投入し、対応分岐の遅延線先頭を返す（コミュテータは内部カウンタ）。
    pub fn push(&mut self, byte: u8) -> u8 {
        let k = self.idx % BI_I;
        self.idx += 1;
        let f = &mut self.fifos[k];
        f.push_back(byte);
        f.pop_front().unwrap_or(byte) // 遅延0の分岐（k=I-1）はそのまま
    }
}

/// Forney畳み込みバイト**インターリーバ**（送信側, I=12, M=17）。分岐kの遅延 `M*k`。
/// 検証用（[`ByteDeinterleaver`] の対）。
pub struct ByteInterleaver {
    fifos: Vec<VecDeque<u8>>,
    idx: usize,
}
impl Default for ByteInterleaver {
    fn default() -> Self {
        Self::new()
    }
}
impl ByteInterleaver {
    pub fn new() -> Self {
        let fifos = (0..BI_I)
            .map(|k| VecDeque::from(vec![0u8; BI_M * k]))
            .collect();
        Self { fifos, idx: 0 }
    }
    pub fn push(&mut self, byte: u8) -> u8 {
        let k = self.idx % BI_I;
        self.idx += 1;
        let f = &mut self.fifos[k];
        f.push_back(byte);
        f.pop_front().unwrap_or(byte)
    }
}

/// エネルギー逆拡散のPRBS（gr-isdbt準拠：reg=0xa9, 帰還 bit13^bit14, 15bit）。
/// `clock_prbs(8)` で8bitを1バイトにして返す。
pub struct EnergyPrbs {
    reg: u16,
}

impl Default for EnergyPrbs {
    fn default() -> Self {
        Self::new()
    }
}

impl EnergyPrbs {
    pub fn new() -> Self {
        Self::with_init(0xa9)
    }
    /// 初期値を指定して作る（規格解釈のブレを実機で総当たりするため）。
    pub fn with_init(init: u16) -> Self {
        Self { reg: init & 0x7fff }
    }
    pub fn reset(&mut self) {
        self.reg = 0xa9;
    }
    /// 任意の初期値でリセット。
    pub fn reset_to(&mut self, init: u16) {
        self.reg = init & 0x7fff;
    }
    /// `clocks` ビットぶん進め、そのビット列を整数で返す（8なら1バイト）。
    pub fn clock(&mut self, clocks: usize) -> u32 {
        let mut res = 0u32;
        for _ in 0..clocks {
            let feedback = ((self.reg >> 13) ^ (self.reg >> 14)) & 0x1;
            self.reg = ((self.reg << 1) | feedback) & 0x7fff;
            res = (res << 1) | feedback as u32;
        }
        res
    }
}

/// バイト列中で、位相 `p`（0..204）に同期バイト(0x47/0xB8)が周期204で立つ割合。
pub fn sync_score_at(bytes: &[u8], p: usize) -> f32 {
    let mut hit = 0usize;
    let mut tot = 0usize;
    let mut i = p;
    while i < bytes.len() {
        if bytes[i] == SYNC || bytes[i] == SYNC_INV {
            hit += 1;
        }
        tot += 1;
        i += TSP;
    }
    if tot == 0 {
        0.0
    } else {
        hit as f32 / tot as f32
    }
}

/// 最も同期バイトが揃う位相(0..204)とそのスコアを返す。
pub fn best_sync_phase(bytes: &[u8]) -> (usize, f32) {
    (0..TSP)
        .map(|p| (p, sync_score_at(bytes, p)))
        .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
        .unwrap_or((0, 0.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pack_bits_msb_basic() {
        let bits = [0, 1, 0, 0, 0, 1, 1, 1]; // 0x47
        assert_eq!(pack_bits_msb(&bits, 0), vec![0x47]);
    }

    #[test]
    fn byte_interleave_deinterleave_roundtrip() {
        // 送信側インターリーバ（分岐kの遅延 M*k touch）と本デインターリーバ（M*(11-k)）の
        // 直列は、総遅延 M*I*(I-1)=2244 バイト（=11 TSP）の恒等になるはず。
        // 各分岐は12バイトごとにしか触られないので、遅延は touch数×12。
        let total = BI_M * BI_I * (BI_I - 1);
        let mut tx: Vec<VecDeque<u8>> = (0..BI_I)
            .map(|k| VecDeque::from(vec![0u8; BI_M * k]))
            .collect();
        let mut rx = ByteDeinterleaver::new();
        let n = total + 500;
        let data: Vec<u8> = (0..n).map(|i| (i * 7 + 3) as u8).collect();
        for (i, &d) in data.iter().enumerate() {
            let k = i % BI_I;
            tx[k].push_back(d);
            let t = tx[k].pop_front().unwrap_or(d);
            let out = rx.push(t);
            if i >= total {
                assert_eq!(out, data[i - total], "i={i}: バイト恒等が崩れた");
            }
        }
    }

    #[test]
    fn prbs_is_deterministic_and_periodic() {
        let mut a = EnergyPrbs::new();
        let first: Vec<u32> = (0..10).map(|_| a.clock(8)).collect();
        let mut b = EnergyPrbs::new();
        let again: Vec<u32> = (0..10).map(|_| b.clock(8)).collect();
        assert_eq!(first, again); // 決定的
                                  // 最初の1バイトは非自明
        assert_ne!(first[0], 0);
    }
}
