use isdbt_dsp::latm::{BitReader, LatmConfig};
use std::env;

fn main() {
    let path = env::args().nth(1).expect("usage: latm_probe <ts>");
    let data = std::fs::read(path).expect("read ts");
    let mut pes: Vec<u8> = Vec::new();
    for i in (0..data.len()).step_by(188).take(data.len() / 188) {
        if data[i] != 0x47 { continue; }
        let pid = (((data[i + 1] & 0x1f) as usize) << 8) | data[i + 2] as usize;
        if pid != 1411 { continue; }
        let pusi = (data[i + 1] >> 6) & 1 != 0;
        let afc = (data[i + 3] >> 4) & 3;
        let mut p = i + 4;
        if afc & 2 != 0 { p += 1 + data[p] as usize; }
        if afc & 1 == 0 { continue; }
        if pusi { pes.clear(); }
        pes.extend_from_slice(&data[p..i + 188]);
    }
    // PES header: prefix(6) + optional header length at byte 8.
    let audio_start = 9 + pes[8] as usize;
    let audio = if pes.len() > audio_start { &pes[audio_start..] } else { &[] };
    println!("PES bytes: {}, audio bytes: {}, head: {}", pes.len(), audio.len(), &audio[..audio.len().min(16)].iter().map(|b| format!("{b:02x}")).collect::<String>());
    if let Some(cfg) = LatmConfig::parse(BitReader::new(audio)) {
        println!("LATM config: version={} same_framing={} subframes={} programs={} frame_length_type={} latm_get_value={}", cfg.audio_mux_version, cfg.all_streams_same_time_framing, cfg.num_sub_frames, cfg.num_programs, cfg.frame_length_type, cfg.latm_get_value);
    }
    let mut found = 0;
    if let Some(cfg) = LatmConfig::parse_audio_mux_element_prefix(audio) {
        println!("AudioMuxElement prefix: version={} same={} subframes={} programs={} frame_length_type={}", cfg.audio_mux_version, cfg.all_streams_same_time_framing, cfg.num_sub_frames, cfg.num_programs, cfg.frame_length_type);
        found += 1;
    }
    let mut best: Option<(u32, usize, LatmConfig)> = None;
    for bit in 0..audio.len().saturating_sub(2) * 8 {
        if let Some(cfg) = LatmConfig::parse_strict_at_bit(audio, bit) {
            let score = cfg.candidate_score();
            if score > 0 && best.as_ref().map_or(true, |b| score > b.0) {
                best = Some((score, bit, cfg));
            }
        }
    }
    if let Some((score, bit, cfg)) = best {
        println!("best strict candidate bit={} score={} version={} same={} subframes={} programs={} frame_length_type={}", bit, score, cfg.audio_mux_version, cfg.all_streams_same_time_framing, cfg.num_sub_frames, cfg.num_programs, cfg.frame_length_type);
    }
}
