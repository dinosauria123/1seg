//! ISDB-T 1セグ **ストリーミング復調器 CLI**（連続ライブ対応）。
//! IQ(stdin/file) → MPEG-TS(stdout/file) を、届いた端から復号して吐き続ける薄いラッパー。
//! 中核は [`isdbt_dsp::StreamingDecoder`]。
//!
//! ライブ再生（表示のある端末で）：
//! ```bash
//! rtl_sdr -f 497142857 -s 1015873 -g 30 - | \
//!   cargo run --release -q -p isdbt-dsp --example stream_decode -- - - | ffplay -
//! ```

use isdbt_dsp::StreamingDecoder;
use std::io::{Read, Write};
use std::{env, fs};

fn main() {
    let mut a = env::args().skip(1);
    let inpath = a.next().expect("usage: stream_decode <in|-> <out|->");
    let outpath = a.next().expect("out");

    let mut reader: Box<dyn Read> = if inpath == "-" {
        Box::new(std::io::stdin().lock())
    } else {
        Box::new(fs::File::open(&inpath).expect("in"))
    };
    let mut out: Box<dyn Write> = if outpath == "-" {
        Box::new(std::io::stdout().lock())
    } else {
        Box::new(fs::File::create(&outpath).expect("out"))
    };

    // Keep the initial buffered MPEG-TS (PAT/PMT/SPS/PPS and an IDR) so a
    // player joining mid-stream receives a decodable starting point.
    let mut dec = StreamingDecoder::new();
    dec.set_live(false);
    let mut raw = vec![0u8; 1 << 18];
    // Keep TS until the first H.264 SPS/PPS/IDR has arrived.  A live receiver
    // commonly starts between GOPs; forwarding P-slices before the first SPS
    // makes ffmpeg report "no frame" and discards the decodable start.
    let mut pending_ts: Vec<u8> = Vec::new();
    let mut started = false;
    let mut announced = false;
    loop {
        let n = match reader.read(&mut raw) {
            Ok(0) | Err(_) => break,
            Ok(n) => n,
        };
        let ts = dec.feed(&raw[..n]);
        if !ts.is_empty() {
            if !started {
                pending_ts.extend_from_slice(&ts);
                // Detect an Annex-B SPS (00 00 01 67) in the buffered TS.
                // Do not require an IDR here: some GOPs may have PPS in a
                // separate PES and the SPS is sufficient to stop the initial
                // P-slice-only prefix from being forwarded.
                let find_nal = |n: u8| {
                    (0..pending_ts.len().saturating_sub(5))
                        .find(|&j| {
                            (pending_ts[j..].starts_with(&[0x00, 0x00, 0x00, 0x01, n])
                                || pending_ts[j..].starts_with(&[0x00, 0x00, 0x01, n]))
                        })
                };
                if let (Some(sps), Some(_pps), Some(idr)) =
                    (find_nal(0x67), find_nal(0x68), find_nal(0x65))
                {
                    // Drop the initial P-slice-only prefix.  Start at a TS
                    // packet boundary at or before SPS so ffmpeg sees a
                    // complete PES containing SPS/PPS/IDR.
                    let start = (sps / 188) * 188;
                    let _ = out.write_all(&pending_ts[start..]);
                    pending_ts.clear();
                    started = true;
                    eprintln!("SPS/PPS/IDR検出: TS offset {} 以降を出力", start);
                }
            } else {
                let _ = out.write_all(&ts);
            }
            let _ = out.flush();
            if !announced {
                announced = true;
                eprintln!("ロック→ライブ復码開始 (SPS待ち={})", !started);
            }
        }
    }
    let (ndec, nblk) = dec.stats();
    eprintln!(
        "終了: {ndec}/{nblk} ブロック復号 ({:.1}%)",
        100.0 * ndec as f32 / nblk.max(1) as f32
    );
}
