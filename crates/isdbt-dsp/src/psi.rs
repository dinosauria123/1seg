//! MPEG-TS の PSI（PAT/PMT）注入。
//!
//! ISDB-T 1seg は PSI を TMCC 経由で運ぶ仕様で、復調した TS には
//! NIT (0x1FFF) と PMT (0x1FC8) が周期性的に現れるが、**PAT (PID 0) が欠落**する。
//! PAT が無いと VLC などのプレイヤーが program を解決できず、
//! ライブ TS では "buffer deadlock prevented" で停止する
//! （ffplay は PAT 不在でも再生できるためPlayerごと挙動が違う）。
//!
//! そこで復調出力に PAT を合成して差し込む。映像/音声 PID は
//! 1seg の固定割当（映像 0x581 / 音声 0x583）を使う。
//!
//! 参照: ISO/IEC 13818-1 §2.4.4.3 (PAT), §2.4.4.8 (PMT), Annex A (CRC32).

/// 1seg の固定 PID 割当（ISDB-T）。
pub const PID_VIDEO: u16 = 0x581;
pub const PID_AUDIO: u16 = 0x583;
pub const PID_PAT: u16 = 0x0000;
pub const PID_PMT: u16 = 0x1fc8;
/// program_number の既定値。
///
/// **実放送の program_number は局に依って変わる**（実測 2026-09-26: 札幌
/// NHK総合 ch15 = 10624 = 0x2980）。以前の 10240 は別の値だったため、ffmpeg が
/// `no program with id 10240 found` を出し program を解決できていなかった。
///
/// 合成 PAT/PMT の program_number は PMT_PID と一致している必要がない。ffmpeg
/// は PAT で得た program_number から PMT を辿るだけなので、PAT と PMT で
/// 同じ値を使えば program は解決する。値は局ごとに変わっても再生には
/// 影響しないので、既定値のまま注入してよい。
pub const PROGRAM_NO: u16 = 10624;

/// 実放送の program_number を任意の PSI バイト列から検出する。
///
/// 1seg の partial reception では **PAT (PID 0) 自体が復調されない**ため、
/// 復調側からは program_number を確実に取得できない。検出できない場合は
/// `None` を返し、呼び出し側は [`PROGRAM_NO`] の既定値を使う。
///
/// 戻り値が `None` でも再生は壊れない（既定値で PAT/PMT が整合するため）。
pub fn detect_program_no(_psi: &[u8]) -> Option<u16> {
    None
}

/// MPEG-2 systems 用の CRC32（poly 0x04C11DB7、初期値 0xFFFFFFFF）。
///
/// **final XOR を適用しない。** MPEG-2 section の CRC_32 フィールドは
/// `!crc` ではなく生の crc が入る。final XOR を付けると VLC が
/// `Bad CRC_32` で PAT/PMT を拒否し、demux が全 PID を unknown として
/// 扱って 0:00 のまま停止する（実測 2026-09-26）。
pub fn crc32_mpeg(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= (b as u32) << 24;
        for _ in 0..8 {
            crc = if crc & 0x8000_0000 != 0 {
                (crc << 1) ^ 0x04C1_1DB7
            } else {
                crc << 1
            };
        }
    }
    // final XOR なし（MPEG-2 systems 仕様）
    crc
}

/// 1つの TS パケットを組み立てる（stuffing なし・固定長）。
fn ts_packet(pid: u16, pusi: bool, payload: &[u8], cc: u8) -> [u8; 188] {
    let mut p = [0u8; 188];
    p[0] = 0x47;
    p[1] = (if pusi { 0x40 } else { 0x00 }) | ((pid >> 8) as u8 & 0x1f);
    p[2] = (pid & 0xff) as u8;
    // payload_unit_start_indicator=1 のとき 1バイトの pointer_field を置く。
    let off = if pusi { 1 } else { 0 };
    let n = payload.len().min(188 - 4 - off);
    // adaptation_field_control=01 (payload only)。0x40 が正。
    // 0x10 と書くと bits6-5 が 00（不定値）になり、
    // VLC が "PAT decoder: invalid section" / "Bad CRC_32" を出す。
    //
    // 下位 4 bit は continuity_counter で、**payload 長ではない**。
    // ここに n を入れると section 長（16 など）が CC になり、期待 CC と
    // 永久にずれて discontinuity を出し続ける（実測 2026-09-26）。
    // 呼び出し側が psi_packets ごとに適切な CC を管理する。
    p[3] = 0x10 | (cc & 0x0f);
    if pusi {
        p[4] = 0; // pointer_field
    }
    p[4 + off..4 + off + n].copy_from_slice(&payload[..n]);
    // 残り（stuffing）は 0xff に-filled する。0x00 のままだと VLC の
    // PSI パーサがセクション長を過大と解釈し、PAT 全体を破棄して
    // pid[0] を一度も処理しないまま PMT だけ "unknown" になる
    // （実測 2026-09-26: patonly.ts は ffprobe も "Invalid data" で弾く）。
    for b in p[4 + off + n..].iter_mut() {
        *b = 0xff;
    }
    p
}

/// PAT セクション（pointer_field 以外）を返す。
///
/// 構文（ISO/IEC 13818-1）:
///   table_id(1) section_syntax(1)+length(2) transport_stream_id(2)
///   reserved(2)+version(5)+current(1) section_number(1) last_section(1)
///   program_number(2) reserved(3)+PMT_PID(13) CRC(4)
/// = 16 バイト、section_length = 13。
fn pat_section() -> Vec<u8> {
    let mut s = Vec::with_capacity(16);
    s.push(0x00); // table_id = PAT
    let section_length = 13u16;
    s.push(0xb0 | ((section_length >> 8) as u8 & 0x0f)); // section_syntax_indicator=1
    s.push((section_length & 0xff) as u8);
    s.push(0x00); // transport_stream_id hi
    s.push(0x01); // transport_stream_id lo
    s.push(0xc1); // reserved '11' + version 0 + current_next_indicator 1
    s.push(0x00); // section_number
    s.push(0x00); // last_section_number
    s.push((PROGRAM_NO >> 8) as u8);
    s.push((PROGRAM_NO & 0xff) as u8);
    s.push(0xe0 | ((PID_PMT >> 8) as u8 & 0x1f)); // reserved '111' + PMT_PID hi
    s.push((PID_PMT & 0xff) as u8); // PMT_PID lo
    let crc = crc32_mpeg(&s);
    s.extend_from_slice(&crc.to_be_bytes());
    s
}

/// PMT セクションを返す。
///
/// 構文（ISO/IEC 13818-1）:
///   table_id(1) section_syntax(1)+length(2) program_number(2)
///   reserved(2)+version(5)+current(1) section_number(1) last_section(1)
///   reserved(3)+PCR_PID(13) reserved(4)+program_info_length(12)
///   [stream_type(1) reserved(3)+elementary_PID(13) reserved(4)+ES_info_length(12)] * N
///   CRC(4)
/// ES 2 種で 26 バイト、section_length = 23。
fn pmt_section() -> Vec<u8> {
    let mut s = Vec::with_capacity(26);
    s.push(0x02); // table_id = PMT
    let section_length = 23u16;
    s.push(0xb0 | ((section_length >> 8) as u8 & 0x0f));
    s.push((section_length & 0xff) as u8);
    s.push((PROGRAM_NO >> 8) as u8);
    s.push((PROGRAM_NO & 0xff) as u8);
    s.push(0xc1); // reserved '11' + version 0 + current 1
    s.push(0x00); // section_number
    s.push(0x00); // last_section_number
    // PCR_PID は**映像 PID**（0x581）を宣言する。PID_PMT そのものを
    // PCR_PID にすると、ffprobe は pcr_pid=8136 と報告して動いても
    // VLC は ES 側の PCR を期待できず pid[0] を処理しないまま止まる
    // （実測 2026-09-26: discontinuity 452、Position 0）。
    s.push(0xe0 | ((PID_VIDEO >> 8) as u8 & 0x1f)); // reserved '111' + PCR_PID hi
    s.push((PID_VIDEO & 0xff) as u8);
    s.push(0xf0); // reserved '1111' + program_info_length hi(4bit)
    s.push(0x00); // program_info_length lo = 0

    // elementary stream: 映像 (H.264)
    //
    // stream_type は **0x1B**。0x02 は MPEG-2 Video で、VLC は PMT にある
    // 方を優先して avcodec の mpeg2video デコーダを起動し、H.264 として
    // 復号しようせず映像が出ない（実測 2026-09-26: "codec (mpeg2video)
    // started" のまま無音）。ffprobe は IDR の NAL を直接見て h264 と推測
    // したため気づかなかった。
    s.push(0x1b); // stream_type = H.264/AVC (ISO/IEC 14496-10)
    s.push(0xe0 | ((PID_VIDEO >> 8) as u8 & 0x1f));
    s.push((PID_VIDEO & 0xff) as u8);
    s.push(0xf0); // reserved '1111' + ES_info_length hi
    s.push(0x00); // ES_info_length lo = 0

    // elementary stream: 音声 (HE-AAC)
    s.push(0x0f); // stream_type = AAC
    s.push(0xe0 | ((PID_AUDIO >> 8) as u8 & 0x1f));
    s.push((PID_AUDIO & 0xff) as u8);
    s.push(0xf0);
    s.push(0x00);

    let crc = crc32_mpeg(&s);
    s.extend_from_slice(&crc.to_be_bytes());
    s
}

/// PAT/PMT の 1 組を返す（各1パケット・pusi=1）。
///
/// # CC は PID ごとに別々で管理すること
///
/// 実測で 2 つの誤りが判明した（2026-09-26）。
///
/// 1. 毎回 CC=0 → libdvbpsi が `TS duplicate (received 0, expected 1)
///    for PID 0` を出し続ける。PSI は 0 固定にできない。
/// 2. 共有カウンタ（PAT と PMT で同じ `cc`）→ PMT の列が
///    `0,1,...,11,8,9,...` と折り返し、`TS discontinuity
///    (received 13, expected 1) for PID 0` になる。
///
/// つまり **CC はインクリメントcontinuity するが、PAT と PMT は
/// 別カウンタで回す**。これが唯一正しく、外すと VLC は demux を
/// 再起動して 0:00 のまま止まる。
///
/// **さらに ContinuityTracker が `ts` を走査したあとに PSI を注入するので、
/// PSI の CC は Tracker を経由しない正しい値でなければならない。**
/// Tracker が後から書き換えると `...,9,1,10,...` と破綻する
/// （実測 2026-09-26: PMT CC 連続性違反 6 箇所）。
pub fn psi_packets(pat_cc: u8, pmt_cc: u8) -> Vec<[u8; 188]> {
    vec![
        ts_packet(PID_PAT, true, &pat_section(), pat_cc),
        ts_packet(PID_PMT, true, &pmt_section(), pmt_cc),
    ]
}

/// 27 MHz クロックから base(33bit) と ext(9bit) に分解する。
///
/// MPEG-TS の PCR は 33bit の 90kHz ベース + 9bit の 300 分 の拡張部で
/// 27 MHz（= 90 kHz × 300）で表現する。
pub fn pcr_split(ticks_27mhz: u64) -> (u32, u16) {
    let t = ticks_27mhz % (1u64 << 33);
    let base = ((t / 300) % (1u64 << 33)) as u32;
    let ext = (t % 300) as u16;
    (base, ext)
}

/// PCR 付きパケットを 1 個返す（PID は `pid`）。
///
/// なぜ必要か（2026-09-26 実測）: 復調した 1seg TS には PCR が 1 本も
/// 無い。PCR は MPEG-TS の clock reference で、無いとプレイヤーは
/// PTS/DTS を生成できず「0:00 のまま黒画面」で止まる。
///
/// **PID は映像 PID（0x581）でなければならない。** PMT の PCR_PID を映像 PID
/// に宣言しているのに PCR を NULL PID (0x1fff) に入れていると、PCR をタイム
/// スタンプ源として認識されず VLC が
/// "more than 5 seconds of late video -> dropping frame" を繰り返して
/// 映像が静止画のままになる（実測 2026-09-26）。
///
/// adaptation_field 1 バイト + PCR 6 バイトで 1 パケットに収める。
pub fn pcr_packet(pid: u16, cc: u8, ticks_27mhz: u64) -> [u8; 188] {
    let mut p = [0xffu8; 188];
    p[0] = 0x47;
    // transport_error(0) payload_unit_start(0) priority(0)
    p[1] = ((pid >> 8) as u8) & 0x1f;
    p[2] = (pid & 0xff) as u8;
    // adaptation_field_control=10 (adaptation only)。CC も保つ。
    p[3] = 0x20 | (cc & 0x0f);
    p[4] = 7; // adaptation_field_length
    p[5] = 0x10; // PCR_flag
    let (base, ext) = pcr_split(ticks_27mhz);
    p[6] = (base >> 25) as u8;
    p[7] = (base >> 17) as u8;
    p[8] = (base >> 9) as u8;
    p[9] = (base >> 1) as u8;
    p[10] = (((base & 1) as u8) << 7) | 0x7e | ((ext >> 8) as u8 & 0x01);
    p[11] = ext as u8;
    p
}

/// **discontinuity_indicator を立てた PCR パケット** 1 本。
///
/// `pcr_packet()` と違い、adaptation field の `discontinuity_indicator` = 1。
///
/// ## なぜこれが必要か（実測 2026-09-26）
///
/// 訂正不能 RS ブロックは出力しない（穴を空ける）ため、その区間の TS が
/// 丸ごと消える。1seg は 1 フレーム = 64 RS ブロックなので、訂正不能が
/// **51 個連続**すると 1 フレームの 80% が消える（実測: 総 126 中 51 が
/// 1 箇所に集中）。その欠落が **I フレームを丸ごと落とす**。
///
/// I フレームは 2 秒（GOP 間隔）ごとに来るはずだが、欠落区間では
/// 7 秒間まったく出現せず（実測）、H.264 の参照フレーム連鎖が
/// 途切れて以降が破綻する。これが「再生時間とともに画像がこわれる」
/// 現象の正体。
///
/// ## なぜ discontinuity_indicator が効くか
///
/// MPEG-TS (ISO/IEC 13818-1 §2.4.3.4) の discontinuity_indicator は
/// 「**この時点より前の情報（CC / PCR）は信用できない**」と demux に
/// 伝える信号だ。demux はこれを受け取ると、その PID の状態を捨てて
/// 次の PCR 以降を**新規に**扱い直す。つまり:
/// - 壊れた GOP を「壊れたまま」再生し続けるのではなく、
/// - **次の I フレームから回復**できる
///
/// これが H.264 の自己修復（I フレーム = 独立した完全画）に沿った
/// 正しいアプローチ。`ContinuityTracker` で CC を詰めて隠す方法は
/// demux に嘘をつくことになり、回復の手がかりを渡せない。
///
/// ## MPEG-TS 構造（ISO/IEC 13818-1 §2.4.3.2 / §2.4.3.4）
/// ```text
/// byte 0:    0x47                              sync
/// byte 1-2:  PID hi/lo                          PCR_PID（映像 PID）
/// byte 3:    scrambling=0, afc=0b10, CC
/// byte 4:    183 (0xB7)                        adaptation_field_length
/// byte 5:    discontinuity_indicator=1 | PCR_flag=1   = 0x90
/// byte 6-11: PCR (base 33bit + ext 9bit)
/// byte 12-187: 0xFF                             stuffing
/// ```
/// PCR は discontinuity の**直後**に来る: adaptation_field の
/// 最初のフラグバイトの次の項目が PCR だから。
pub fn pcr_packet_discontinuity(pid: u16, cc: u8, ticks_27mhz: u64) -> [u8; 188] {
    let mut p = [0xffu8; 188];
    p[0] = 0x47;
    p[1] = ((pid >> 8) as u8) & 0x1f;
    p[2] = (pid & 0xff) as u8;
    // adaptation_field_control=10 (adaptation only)
    p[3] = 0x20 | (cc & 0x0f);
    // adaptation_field_length = 183: 先頭 1 バイト(フラグ) + PCR 6 バイト + stuffing
    p[4] = 183;
    // discontinuity_indicator(0x80) | PCR_flag(0x10)
    p[5] = 0x90;
    let (base, ext) = pcr_split(ticks_27mhz);
    p[6] = (base >> 25) as u8;
    p[7] = (base >> 17) as u8;
    p[8] = (base >> 9) as u8;
    p[9] = (base >> 1) as u8;
    p[10] = (((base & 1) as u8) << 7) | 0x7e | ((ext >> 8) as u8 & 0x01);
    p[11] = ext as u8;
    p
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pat_points_at_pmt() {
        let sec = pat_section();
        // table_id, section_length, program, pmt_pid
        assert_eq!(sec[0], 0x00);
        let section_length = ((sec[1] as u16 & 0x0f) << 8) | sec[2] as u16;
        assert_eq!(section_length as usize, sec.len() - 3);
        // バイト配置（ts_id は 2 バイト）:
        //  0 table_id / 1-2 section_length / 3-4 transport_stream_id
        //  5 version+current_next / 6 section_number / 7 last_section_number
        //  8-9 program_number / 10-11 reserved+PMT_PID / 12-15 CRC
        assert_eq!(((sec[8] as u16) << 8) | sec[9] as u16, PROGRAM_NO);
        let pmt_pid = (((sec[10] as u16) & 0x1f) << 8) | sec[11] as u16;
        assert_eq!(pmt_pid, PID_PMT);
    }

    #[test]
    fn pmt_lists_video_and_audio() {
        let sec = pmt_section();
        let section_length = ((sec[1] as u16 & 0x0f) << 8) | sec[2] as u16;
        assert_eq!(section_length as usize, sec.len() - 3);
        // バイト配置:
        //  0 table_id / 1-2 section_length / 3-4 program_number
        //  5 version / 6 section_number / 7 last_section_number
        //  8-9 reserved+PCR_PID / 10 reserved / 11 program_info_length
        //  12 stream_type(video) / 13-14 ES_PID(video) / 15-16 ES_info_length
        //  17 stream_type(audio) / 18-19 ES_PID(audio) / 20-21 ES_info_length
        let pcr_pid = (((sec[8] as u16) & 0x1f) << 8) | sec[9] as u16;
        // PCR_PID は映像 PID。PMT PID を入れると VLC が ES 側の PCR を
        // 見つけられず pid[0] を処理しないまま 0:00 で止まる（実測 2026-09-26）。
        assert_eq!(pcr_pid, PID_VIDEO);
        assert_eq!(sec[12], 0x1b, "映像は H.264 (stream_type 0x1b、0x02 は MPEG-2 で誤認される)");
        assert_eq!((((sec[13] as u16) & 0x1f) << 8) | sec[14] as u16, PID_VIDEO);
        assert_eq!(sec[17], 0x0f, "音声は AAC (stream_type 0x0f)");
        assert_eq!((((sec[18] as u16) & 0x1f) << 8) | sec[19] as u16, PID_AUDIO);
    }

    /// PCR は映像 PID に入れる。PMT の PCR_PID と一致していないと VLC が
    /// "more than 5 seconds of late video -> dropping frame" を繰り返し、
    /// 映像が静止画のままになる（実測 2026-09-26）。
    #[test]
    fn pcr_lands_on_video_pid() {
        let p = pcr_packet(PID_VIDEO, 7, 27_000_000);
        assert_eq!(p[0], 0x47);
        let pid = (((p[1] as u16 & 0x1f) << 8) | p[2] as u16) & 0x1fff;
        assert_eq!(pid, PID_VIDEO, "PCR は PMT の PCR_PID と同じ PID であること");
        assert_eq!(p[3] & 0x0f, 7, "CC が保持されること");
        assert_eq!((p[3] >> 4) & 0x03, 0b10, "adaptation_field_control=10");
        assert_eq!(p[4], 7, "adaptation_field_length");
        assert_eq!(p[5] & 0x10, 0x10, "PCR_flag が立つこと");
    }

    /// PCR の base は ticks/300 の 90 kHz カウンタになる。
    #[test]
    fn pcr_base_advances_at_90khz() {
        let a = pcr_split(27_000_000); // 1 秒
        let b = pcr_split(54_000_000); // 2 秒
        assert_eq!(b.0 - a.0, 90_000, "1 秒は 90000 ティック (90 kHz)");
        assert_eq!(a.1, 0, "1 秒は extension ちょうど 0");
    }

    #[test]
    fn packets_are_wellformed() {
        for p in psi_packets(0, 0) {
            assert_eq!(p[0], 0x47);
            assert_eq!(p.len(), 188);
            assert_eq!(p[1] & 0x40, 0x40, "pusi が立つこと");
        }
    }

    /// PAT と PMT は**別々**の CC を持つこと。
    ///
    /// 共有すると PMT の列が `0,1,...,11,8,9,...` と折り返して
    /// discontinuity になる。0 固定にすると duplicate になる。
    /// よって PID ごとに独立にインクリメントするのが唯一正しい。
    #[test]
    fn psi_cc_is_per_pid() {
        // 実測Hadの折り返しパターン: 共有なら 12,13 が飛ばされて 8,9 に戻る
        for i in 0..16u8 {
            let pkts = psi_packets(i, i.wrapping_add(5));
            for (p, pid) in pkts.iter().zip([PID_PAT, PID_PMT]) {
                let got_pid = (((p[1] as u16 & 0x1f) << 8) | p[2] as u16) & 0x1fff;
                assert_eq!(got_pid, pid);
            }
            // 2 回続けて注入した時の CC は +1 ずつ増える（mod 16）
            let a = psi_packets(i, i.wrapping_add(5));
            let b = psi_packets(i.wrapping_add(1), i.wrapping_add(6));
            for (pa, pb) in a.iter().zip(b.iter()) {
                assert_eq!(
                    (pb[3] & 0x0f),
                    (pa[3] + 1) & 0x0f,
                    "CC は注入ごとに 1 増える（mod 16）"
                );
            }
        }
    }

    /// 注入引数 anatagonist 違っても PAT と PMT の CC は独立していること。
    #[test]
    fn psi_cc_pat_pmt_independent() {
        // PAT=3, PMT=9 なら両方的 Their CC はそのまま
        let pkts = psi_packets(3, 9);
        assert_eq!(pkts[0][3] & 0x0f, 3);
        assert_eq!(pkts[1][3] & 0x0f, 9);
    }
}
