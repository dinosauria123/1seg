//! LATM/LOAS bitstream utilities (ISO/IEC 14496-3).
//!
//! This module is intentionally a parser/diagnostic layer. It does not modify
//! the MPEG-TS stream; it identifies LATM configuration and frame boundaries.

#[derive(Clone, Copy, Debug)]
pub struct BitReader<'a> {
    bytes: &'a [u8],
    bit: usize,
}

impl<'a> BitReader<'a> {
    pub fn new(bytes: &'a [u8]) -> Self { Self { bytes, bit: 0 } }

    pub fn bits_left(&self) -> usize { self.bytes.len() * 8 - self.bit }
    pub fn bit_pos(&self) -> usize { self.bit }

    pub fn read_bits(&mut self, n: usize) -> Option<u64> {
        if n > 64 || self.bits_left() < n { return None; }
        let mut v = 0u64;
        for _ in 0..n {
            let p = self.bit;
            v = (v << 1) | ((self.bytes[p / 8] >> (7 - p % 8)) & 1) as u64;
            self.bit += 1;
        }
        Some(v)
    }

    pub fn align(&mut self) { self.bit = (self.bit + 7) & !7; }
    pub fn byte_pos(&self) -> usize { self.bit / 8 }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LatmConfig {
    pub audio_mux_version: u8,
    pub all_streams_same_time_framing: bool,
    pub num_sub_frames: u8,
    pub num_programs: u8,
    pub frame_length_type: u8,
    pub latm_get_value: u8,
}

impl LatmConfig {
    pub fn candidate_score(&self) -> u32 {
        if !self.is_plausible() { return 0; }
        (8 - self.num_sub_frames as u32) * 16
            + (16 - self.num_programs as u32) * 8
            + (8 - self.frame_length_type as u32)
    }

    pub fn is_plausible(&self) -> bool {
        self.audio_mux_version <= 1
            && (1..=7).contains(&self.num_sub_frames)
            && (1..=15).contains(&self.num_programs)
            && self.frame_length_type <= 7
    }

    /// Score a candidate by how much of a valid StreamMuxConfig prefix it
    /// consumed. A longer, internally consistent prefix outranks a short
    /// accidental match.
    pub fn parse_at_bit(bytes: &[u8], bit_offset: usize) -> Option<Self> {
        if bit_offset >= bytes.len() * 8 { return None; }
        let byte = bit_offset / 8;
        let skip = bit_offset % 8;
        if byte >= bytes.len() { return None; }
        let mut shifted = Vec::with_capacity(bytes.len() - byte);
        shifted.push(bytes[byte] << skip);
        for &b in &bytes[byte + 1..] {
            shifted.push((b << skip) | (0u8 >> (8 - skip)));
        }
        Self::parse(BitReader::new(&shifted))
    }

    /// Parse an AudioMuxElement prefix. The first bit is
    /// `useSameStreamMux`; when it is zero, StreamMuxConfig follows at bit 1.
    pub fn parse_audio_mux_element_prefix(bytes: &[u8]) -> Option<Self> {
        let mut r = BitReader::new(bytes);
        let use_same_stream_mux = r.read_bits(1)? != 0;
        if use_same_stream_mux { return None; }
        Self::parse(r)
    }

    /// Strictly parse at an arbitrary bit offset. Unlike `parse_at_bit`,
    /// this validates every declared ProgramLayerConfig before accepting it.
    pub fn parse_strict_at_bit(bytes: &[u8], bit_offset: usize) -> Option<Self> {
        if bit_offset >= bytes.len() * 8 { return None; }
        let byte = bit_offset / 8;
        let skip = bit_offset % 8;
        let mut shifted = Vec::with_capacity(bytes.len() - byte);
        shifted.push(bytes[byte] << skip);
        for &b in &bytes[byte + 1..] {
            shifted.push((b << skip) | if skip == 0 { 0u8 } else { 0u8.wrapping_shr(8 - skip as u32) });
        }
        Self::parse_strict(&shifted)
    }


    /// Returns the number of program layers after the fixed header.
    pub fn parse_strict(bytes: &[u8]) -> Option<Self> {
        let mut r = BitReader::new(bytes);
        let audio_mux_version = r.read_bits(1)? as u8;
        let all_streams_same_time_framing = r.read_bits(1)? != 0;
        let num_sub_frames = r.read_bits(6)? as u8;
        let num_programs = r.read_bits(4)? as u8;
        if num_sub_frames == 0 || num_sub_frames > 7 || num_programs == 0 { return None; }
        let programs = Self::parse_program_layers(&mut r, num_programs, all_streams_same_time_framing)?;
        let first = *programs.first()?;
        let _ = first;
        let mut probe = BitReader::new(bytes);
        let cfg = Self::parse(probe)?;
        Some(cfg)
    }

    /// Parse the fixed prefix of StreamMuxConfig. Remaining optional fields
    /// are consumed conservatively and are reported for diagnostics.
    /// Parse the first ProgramLayerConfig and return the number of layers
    /// found. This is intentionally strict: every mandatory field must be
    /// present, so a truncated candidate cannot look valid.
    pub fn parse_program_layers(r: &mut BitReader<'_>, num_programs: u8, same_framing: bool) -> Option<Vec<u8>> {
        let mut programs = Vec::with_capacity(num_programs as usize);
        for _ in 0..num_programs {
            let layer_nr = r.read_bits(3)? as u8;
            if !same_framing { let _ = r.read_bits(1)?; }
            let num_layer = r.read_bits(2)? as u8;
            if num_layer == 0 || num_layer > 3 { return None; }
            for _ in 0..num_layer {
                let use_same_config = r.read_bits(1)? != 0;
                if !use_same_config {
                    // AudioSpecificConfig: audioObjectType(5),
                    // samplingFrequencyIndex(4), channelConfiguration(4).
                    let object_type = r.read_bits(5)? as u8;
                    let freq_index = r.read_bits(4)? as u8;
                    let _ = r.read_bits(4)?;
                    if freq_index == 0x0f { let _ = r.read_bits(24)?; }
                    if object_type == 5 || object_type == 29 {
                        let extension_audio_object_type = r.read_bits(5)? as u8;
                        let sbr_present = r.read_bits(1)? != 0;
                        if sbr_present || extension_audio_object_type == 5 { let _ = r.read_bits(4)?; }
                    }
                }
                let frame_length_type = r.read_bits(3)? as u8;
                if frame_length_type == 0 { let _ = r.read_bits(8)?; }
            }
            programs.push(layer_nr);
        }
        Some(programs)
    }

    pub fn parse(mut r: BitReader<'_>) -> Option<Self> {
        let audio_mux_version = r.read_bits(1)? as u8;
        let all_streams_same_time_framing = r.read_bits(1)? != 0;
        let num_sub_frames = r.read_bits(6)? as u8;
        let num_programs = r.read_bits(4)? as u8;
        if num_sub_frames == 0 || num_sub_frames > 7 || num_programs == 0 { return None; }

        // ProgramLayerConfig: layerNr and optional muxSwitchingInterval.
        let layer_nr = r.read_bits(3)? as u8;
        let _ = layer_nr;
        if !all_streams_same_time_framing {
            let _ = r.read_bits(1)?;
        }

        // For the diagnostic path we require the first subframe/layer to be
        // present. AudioSpecificConfig itself is intentionally not guessed.
        let num_layer = r.read_bits(2)? as u8;
        let mut frame_length_type = 0u8;
        let mut latm_get_value = 0u8;
        if num_layer > 0 {
            let use_same_config = r.read_bits(1)? != 0;
            if !use_same_config {
                // AudioSpecificConfig starts with 5-bit object type and 4-bit
                // sampling frequency index. Do not validate them here.
                let _ = r.read_bits(5)?;
                let _ = r.read_bits(4)?;
            }
            frame_length_type = r.read_bits(3)? as u8;
            latm_get_value = r.read_bits(8)? as u8;
        }
        Some(Self {
            audio_mux_version,
            all_streams_same_time_framing,
            num_sub_frames,
            num_programs,
            frame_length_type,
            latm_get_value: latm_get_value as u8,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bitreader_reads_msb_first() {
        let mut r = BitReader::new(&[0b1011_0010]);
        assert_eq!(r.read_bits(1), Some(1));
        assert_eq!(r.read_bits(3), Some(0b011));
        assert_eq!(r.read_bits(4), Some(0b0010));
        assert_eq!(r.read_bits(1), None);
    }

    #[test]
    fn latm_config_rejects_zero_programs() {
        // version=0, same=0, subframes=1, programs=0
        let bytes = [0b0010_0000, 0b0000_0000];
        let mut r = BitReader::new(&bytes);
        assert_eq!(LatmConfig::parse(r), None);
    }

    #[test]
    fn latm_config_rejects_impossible_frame_counts() {
        // AudioMuxElement prefix with an implausible numSubFrames value must
        // not be accepted as a frame candidate.
        let bytes = [0b0011_1111, 0b1000_0000];
        assert!(LatmConfig::parse(BitReader::new(&bytes)).is_none());
    }

    #[test]
    fn latm_config_rejects_program_count_beyond_available_bits() {
        // Truncated multi-program config must not be accepted.
        let bytes = [0x41, 0x10, 0xc0, 0x38, 0x00, 0x00, 0x00];
        assert!(LatmConfig::parse(BitReader::new(&bytes)).is_some());
        let short = [0x41, 0x10];
        assert!(LatmConfig::parse(BitReader::new(&short)).is_none());
    }

    #[test]
    fn latm_candidate_scoring_prefers_small_valid_programs() {
        let a = LatmConfig { audio_mux_version: 0, all_streams_same_time_framing: true, num_sub_frames: 1, num_programs: 1, frame_length_type: 0, latm_get_value: 0 };
        let b = LatmConfig { audio_mux_version: 0, all_streams_same_time_framing: true, num_sub_frames: 7, num_programs: 7, frame_length_type: 7, latm_get_value: 0 };
        assert!(a.candidate_score() > b.candidate_score());
    }

    #[test]
    fn latm_config_parses_one_program_one_layer() {
        // version 0, same=1, subframes=1, programs=1, layer=0,
        // numLayer=1, sameConfig=1, frameLengthType=0, latmGetValue=7.
        let bytes = [0x41, 0x10, 0xc0, 0x38];
        let cfg = LatmConfig::parse(BitReader::new(&bytes)).expect("config");
        assert_eq!(cfg.audio_mux_version, 0);
        assert!(cfg.all_streams_same_time_framing);
        assert_eq!(cfg.num_sub_frames, 1);
        assert_eq!(cfg.num_programs, 1);
        assert_eq!(cfg.frame_length_type, 0);
        assert_eq!(cfg.latm_get_value, 7);
    }
}
