//! Measure per-symbol SP phase scores from an IQ capture.
//! Usage: cargo run --release -p isdbt-dsp --example sp_phase_probe -- cap.iq 1015873 [symbols]

use isdbt_dsp::demod::OfdmDemod;
use isdbt_dsp::equalize::{phase_scores, SEGMENT_BIN_OFFSET};
use isdbt_dsp::iq::u8_iq_to_complex;
use isdbt_dsp::params::{FFT_LEN, GuardInterval};
use isdbt_dsp::pilots::SegmentPilots;
use isdbt_dsp::sync::estimate_symbol_sync;

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args.next().expect("usage: sp_phase_probe cap.iq fs [symbols]");
    let _fs: f64 = args.next().unwrap().parse().unwrap();
    let count: usize = args.next().map(|x| x.parse().unwrap()).unwrap_or(500);
    let bytes = std::fs::read(path).expect("IQ read");
    let signal = u8_iq_to_complex(&bytes);
    let off = (signal.len() / 10).min(50_000);
    let signal = &signal[off..];
    let sync = estimate_symbol_sync(signal, FFT_LEN, GuardInterval::G1_8).expect("sync");
    let demod = OfdmDemod::new(FFT_LEN);
    let specs = demod.demod_stream(signal, sync.symbol_start, sync.guard, sync.cfo_subcarriers, count);
    let pilots = SegmentPilots::center_1seg();
    let mut hist = [0usize; 4];
    let mut sum_margin = 0.0f32;
    for (i, spec) in specs.iter().enumerate() {
        let seg = &spec[SEGMENT_BIN_OFFSET..SEGMENT_BIN_OFFSET + 432];
        let scores = phase_scores(seg, &pilots);
        let mut order = [0usize, 1, 2, 3];
        order.sort_by(|&a, &b| scores[b].partial_cmp(&scores[a]).unwrap());
        let best = order[0];
        hist[best] += 1;
        sum_margin += scores[order[0]] - scores[order[1]];
        if i < 20 || i % 100 == 0 {
            println!("symbol={i:4} best={best} scores={:.3},{:.3},{:.3},{:.3} margin={:.3}", scores[0], scores[1], scores[2], scores[3], scores[best]-scores[order[1]]);
        }
    }
    println!("symbols={} histogram={hist:?} mean_margin={:.3}", specs.len(), sum_margin / specs.len().max(1) as f32);
}
