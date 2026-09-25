//! 実IQから ④の入口・TMCC を復号する診断ツール。
//!
//! ②同期→復調→中央セグメント抽出→TMCCキャリアのDBPSK→フレーム同期(204)→
//! 多数決でフレーム統合→伝送パラメータをパースして表示する。
//!
//! ```bash
//! cargo run -p isdbt-dsp --example tmcc_probe -- cap.iq 1015873 [nsym]
//! ```

use isdbt_dsp::demod::OfdmDemod;
use isdbt_dsp::equalize::SEGMENT_BIN_OFFSET;
use isdbt_dsp::iq::u8_iq_to_complex;
use isdbt_dsp::params::{FFT_LEN, GuardInterval};
use isdbt_dsp::tmcc::{
    coding_rate_str, equalized_dbpsk_bits, integer_offset_score,
    interleaving_mode3, majority_frame, parse_tmcc, phase_offset_scores, find_frame_sync_joint, tmcc_bch_ok,
    LayerInfo, SYMBOLS_PER_FRAME,
};
use num_complex::Complex32;
use std::{env, fs};

fn layer_line(tag: &str, l: &LayerInfo) {
    let il = interleaving_mode3(l.interleaving)
        .map(|i| format!("I={i}"))
        .unwrap_or_else(|| "I=未使用".into());
    println!(
        "  Layer {tag}: 変調={:?}  符号化率={}  時間IL={}  セグ数={}",
        l.modulation,
        coding_rate_str(l.coding_rate),
        il,
        l.n_segments
    );
}

fn main() {
    let mut a = env::args().skip(1);
    let path = a.next().expect("usage: tmcc_probe <cap.iq> <fs_hz> [nsym]");
    let fs: f64 = a.next().expect("fs_hz").parse().expect("fs数値");
    let nsym: usize = a.next().map(|s| s.parse().unwrap()).unwrap_or(2000);

    let bytes = fs::read(&path).expect("IQ読み込み");
    let mut s = u8_iq_to_complex(&bytes);
    let mean: Complex32 = s.iter().sum::<Complex32>() / s.len() as f32;
    for v in s.iter_mut() {
        *v -= mean;
    }
    let off = (s.len() / 10).min(50_000);
    let seg_sig = &s[off..];

    // Japanese 1seg uses GI 1/8. Do not let the periodicity heuristic select
    // a false 1/32 peak on a real capture.
    let est = isdbt_dsp::sync::estimate_symbol_sync(seg_sig, FFT_LEN, GuardInterval::G1_8).expect("同期できない");
    eprintln!(
        "② sync: start={} gi={:?} metric={:.3} cfo={:.1}Hz",
        est.symbol_start,
        est.guard,
        est.metric,
        est.cfo_subcarriers as f64 * fs / FFT_LEN as f64
    );

    let demod = OfdmDemod::new(FFT_LEN);
    let specs = demod.demod_stream_tracked(
        seg_sig,
        est.symbol_start,
        est.guard,
        est.cfo_subcarriers,
        nsym,
        8,
    );
    // gr-isdbt estimates residual integer CFO from known TMCC pilot phase.
    // This search is independent of the decoded TMCC data bits.
    let mut best_offset = SEGMENT_BIN_OFFSET;
    let mut best_score = -1.0f32;
    let mut best_sync = 0.0f32;
    for off in (SEGMENT_BIN_OFFSET.saturating_sub(16)..SEGMENT_BIN_OFFSET+17).rev() {
        if off + 432 > FFT_LEN { continue; }
        let candidate_segs: Vec<Vec<Complex32>> = specs.iter().map(|sp| sp[off..off + 432].to_vec()).collect();
        let candidate_bits = equalized_dbpsk_bits(&candidate_segs);
        let candidate_sync = find_frame_sync_joint(&candidate_bits).map(|s| s.matched as f32 / s.total as f32).unwrap_or(0.0);
        let known = integer_offset_score(&specs, off);
        // TMCC sync is the primary objective; known-carrier score breaks ties.
        let score = candidate_sync * 1000.0 + known;
        if score > best_score {
            best_score = score;
            best_offset = off;
            best_sync = candidate_sync;
        }
        eprintln!("   candidate offset {off}: sync={:.3} known={:.3} combined={:.3}", candidate_sync, known, score);
    }
    let best_offset = best_offset;
    eprintln!("② TMCC候補offset選択: offset={} (nominal {}) sync={:.3} combined={:.3}", best_offset, SEGMENT_BIN_OFFSET, best_sync, best_score);

    let segs: Vec<Vec<Complex32>> = specs
        .iter()
        .map(|sp| sp[best_offset..best_offset + 432].to_vec())
        .collect();
    let phase_scores = phase_offset_scores(&segs);
    println!("TMCC 巡回origin別 同期一致率: {:.3} {:.3} {:.3} {:.3}", phase_scores[0], phase_scores[1], phase_scores[2], phase_scores[3]);
    eprintln!(
        "復調 {} シンボル（≈{:.1} フレーム）",
        segs.len(),
        segs.len() as f32 / SYMBOLS_PER_FRAME as f32
    );

    // TMCC：DBPSK復号 → フレーム同期
    let bits = equalized_dbpsk_bits(&segs);
    let fsync = find_frame_sync_joint(&bits).expect("フレーム同期できるビット数がない");

    println!("\n=== ④ TMCC フレーム同期 ===");
    println!("フレーム位相(B0位置) : {}", fsync.phase);
    println!("評価フレーム数        : {}", fsync.n_frames);
    println!(
        "同期語一致            : {}/{} ({:.1}%)",
        fsync.matched,
        fsync.total,
        100.0 * fsync.matched as f32 / fsync.total as f32
    );
    let par: String = fsync
        .parity_per_frame
        .iter()
        .map(|&o| if o { 'O' } else { 'E' })
        .collect();
    println!(
        "各フレームの偶奇      : {par}  （交互={}）",
        fsync.alternates
    );
    println!(
        "判定                  : {}",
        if fsync.is_true_lock() {
            "✅ TMCCロック（16bit同期語が一貫＋偶奇交互＋BCH OK）".to_string()
        } else if fsync.consistent_sync_bits < 16 {
            format!(
                "❌ 偽ロック（同期語16bit中 {}bitしか一貫しない＝熱ノイズ由来。1seg信号なし）",
                fsync.consistent_sync_bits
            )
        } else if !fsync.alternates {
            "△ 同期弱い（even/oddが交互でない＝フレーム整合なし）".to_string()
        } else {
            "△ 同期弱い（C/N不足の可能性）".to_string()
        }
    );
    println!("同期語一貫ビット      : {}/16", fsync.consistent_sync_bits);
    println!("真のロックか          : {}", if fsync.is_true_lock() { "YES" } else { "NO" });

    // 情報部をフレーム間多数決で統合してパース
    let frame = majority_frame(&bits, fsync.phase);
    println!("BCH(273,191)検査     : {}", if tmcc_bch_ok(&frame) { "OK" } else { "NG" });
    println!("多数決フレーム bit数 : {} ones / 204", frame.iter().filter(|&&b| b != 0).count());
    let info = parse_tmcc(&frame);

    println!("\n=== TMCC 伝送パラメータ ===");
    println!(
        "システム識別={} / 伝送切替指標={} / 緊急警報={} / 部分受信={}",
        info.system_id, info.switching_indicator, info.emergency_flag, info.partial_reception
    );
    layer_line("A(1seg)", &info.layer_a);
    layer_line("B", &info.layer_b);
    layer_line("C", &info.layer_c);
    eprintln!(
        "\n注：捕捉は中央1セグのみ。TMCCは全帯域共通なのでB/Cも読めるが、物理的に持つのはLayer A。"
    );
}
