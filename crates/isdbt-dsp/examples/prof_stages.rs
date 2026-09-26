//! 復調パイプライン各段の所要時間を計測して律速段を特定する診断ツール。
//!
//! `stream_decode` の実時間比が 1.0 を超えていない原因を、段ごとに切り分ける。
//! 最適化の変更前後で比較するためのベースライン計測。
//!
//! ```bash
//! cargo run --release -p isdbt-dsp --example prof_stages -- cap.iq 1015873 4000 [seg_off]
//! ```

use isdbt_dsp::deinterleave::{data_carrier_indices, freq_deinterleave, TimeDeinterleaver};
use isdbt_dsp::demap::{qpsk_soft, BitDeinterleaverQpsk};
use isdbt_dsp::demod::OfdmDemod;
use isdbt_dsp::equalize::{
    equalize, estimate_channel, phase_scores_gr_isdbt, track_symbol_phases,
};
use isdbt_dsp::iq::u8_iq_to_complex;
use isdbt_dsp::params::{GuardInterval, FFT_LEN};
use isdbt_dsp::pilots::SegmentPilots;
use isdbt_dsp::sync::estimate_symbol_sync;
use isdbt_dsp::ts::{pack_bits_msb, ByteDeinterleaver, BI_I, BI_M};
use isdbt_dsp::viterbi::{depuncture, Viterbi, PUNCTURE_2_3};
use num_complex::Complex32;
use std::{env, fs, time::Instant};

fn rep(name: &str, t: Instant, n: usize) {
    let s = t.elapsed().as_secs_f64();
    let per = s / n.max(1) as f64 * 1e6;
    eprintln!("{name:<28} {:>8.3}s  {:>8.2} us/sym", s, per);
}

fn main() {
    let mut a = env::args().skip(1);
    let path = a.next().expect("usage: prof_stages <cap.iq> <fs> [nsym] [seg_off]");
    let _fs: f64 = a.next().expect("fs").parse().unwrap();
    let nsym: usize = a.next().map(|s| s.parse().unwrap()).unwrap_or(4000);
    let seg_off: usize = a
        .next()
        .map(|s| s.parse().unwrap())
        .unwrap_or(isdbt_dsp::SEGMENT_BIN_OFFSET);

    let raw = fs::read(&path).expect("IQ");
    let mut s = u8_iq_to_complex(&raw);
    let mean: Complex32 = s.iter().sum::<Complex32>() / s.len() as f32;
    for v in s.iter_mut() {
        *v -= mean;
    }
    let off = (s.len() / 10).min(50_000);
    let seg_sig = &s[off..];

    // ① 同期（1回）
    let t = Instant::now();
    let est = estimate_symbol_sync(seg_sig, FFT_LEN, GuardInterval::G1_8).expect("同期");
    rep("① estimate_symbol_sync", t, 1);

    // ②a 復調: demod_stream_tracked（CP相関トラッキング込み）
    let demod = OfdmDemod::new(FFT_LEN);
    let t = Instant::now();
    let specs = demod.demod_stream_tracked(
        seg_sig,
        est.symbol_start,
        est.guard,
        est.cfo_subcarriers,
        nsym,
        8,
    );
    let n = specs.len();
    rep("② demod_stream_tracked", t, n);

    // ②b 参考: demod_one_tracked を同じ数だけ（1シンボルずつ）
    let t = Instant::now();
    let mut cur = off + est.symbol_start;
    let mut cnt = 0;
    while cnt < n {
        match demod.demod_one_tracked(seg_sig, cur, est.guard, est.cfo_subcarriers, 8) {
            Some((_, next)) => {
                cur = next;
                cnt += 1;
            }
            None => break,
        }
    }
    rep("②b demod_one_tracked", t, cnt);

    // ③ SP位相
    let pilots = SegmentPilots::center_1seg();
    let t = Instant::now();
    let rows: Vec<[f32; 4]> = specs
        .iter()
        .map(|sp| phase_scores_gr_isdbt(&sp[seg_off..seg_off + 432], &pilots))
        .collect();
    let phases = track_symbol_phases(&rows);
    rep("③ phase_scores+track", t, n);

    // ④ estimate_channel（毎シンボル）
    let t = Instant::now();
    let hs: Vec<Vec<Complex32>> = specs
        .iter()
        .enumerate()
        .map(|(k, sp)| {
            let seg = &sp[seg_off..seg_off + 432];
            estimate_channel(seg, phases[k], &pilots)
        })
        .collect();
    rep("④ estimate_channel", t, n);

    // ⑤ equalize
    let t = Instant::now();
    let eqs: Vec<Vec<Complex32>> = specs
        .iter()
        .enumerate()
        .map(|(k, sp)| equalize(&sp[seg_off..seg_off + 432], &hs[k]))
        .collect();
    rep("⑤ equalize", t, n);

    // ⑥ 周波数デインタ + 時間デインタ + デマップ + ビットデインタ
    let t = Instant::now();
    let mut tdi = TimeDeinterleaver::new(4);
    let mut bdi = BitDeinterleaverQpsk::new();
    let mut coded: Vec<f32> = Vec::new();
    for (k, eq) in eqs.iter().enumerate() {
        let data: Vec<Complex32> = data_carrier_indices(phases[k], &pilots)
            .into_iter()
            .map(|l| eq[l])
            .collect();
        let fd = freq_deinterleave(&data);
        for v in tdi.push_symbol(&fd) {
            let de = bdi.push(qpsk_soft(v));
            coded.push(de[1]);
            coded.push(de[0]);
        }
    }
    rep("⑥ deint+demap", t, n);
    eprintln!("   → coded {} ビット", coded.len());

    // ⑦ depuncture
    let t = Instant::now();
    let restored = depuncture(&coded, &PUNCTURE_2_3);
    rep("⑦ depuncture", t, n);
    let take = restored.len() & !1;

    // ⑧ Viterbi（バッチ: decode.rs が使う方）
    let t = Instant::now();
    let bits = Viterbi::new().decode(&restored[..take]);
    rep("⑧ Viterbi::decode", t, n);
    eprintln!("   → {} ビット", bits.len());

    // ⑧b ViterbiStreaming（stream_decode が実際に使う方）
    let t = Instant::now();
    {
        let mut vs = isdbt_dsp::ViterbiStreaming::new(96);
        let mut got = 0usize;
        for t2 in 0..take / 2 {
            if vs.push(restored[2 * t2], restored[2 * t2 + 1]).is_some() {
                got += 1;
            }
        }
        eprintln!("   → streaming {} ビット", got);
    }
    rep("⑧b ViterbiStreaming::push", t, n);

    // ⑨ pack_bits + byte deinterleave
    let t = Instant::now();
    let bytes = pack_bits_msb(&bits, 0);
    let latency = BI_M * BI_I * (BI_I - 1);
    let _stream: Vec<u8> = {
        let mut di = ByteDeinterleaver::new();
        bytes
            .iter()
            .enumerate()
            .filter_map(|(j, &b)| {
                let o = di.push(b);
                (j >= latency).then_some(o)
            })
            .collect()
    };
    rep("⑨ pack+byte_deint", t, n);

    eprintln!("\n実時間比: 1 OFDM シンボル = {} サンプル = {:.1} us", FFT_LEN + est.guard.cp_len(FFT_LEN),
        (FFT_LEN + est.guard.cp_len(FFT_LEN)) as f64 / _fs * 1e6);
}
