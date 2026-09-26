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

/// 入力ファイルを書き手がまだ活着しているか調べる。
///
/// `ISDBT_WRITER` で指定したファイルの先頭 1 バイトが b'1' なら「書き手あり」、
/// それが無い（ファイルが無い/空/0xffffffff）なら「不在」とみなす。
/// ライブスクリプトは rtl_sdr を立ち上げる際に `echo 1 > $ISDBT_WRITER` を書き、
/// 停止時にそのファイルを消す（または 0 を書く）。
fn writer_alive() -> bool {
    match env::var("ISDBT_WRITER") {
        Ok(p) if !p.is_empty() => fs::read(&p)
            .map(|b| b.first().copied() == Some(b'1'))
            .unwrap_or(false),
        // 環境変数が無ければ「不在」ではなく「常に待つ」扱いにする。
        // 終了は呼び出し側の Ctrl-C で行う。
        _ => true,
    }
}

fn main() {
    let mut a = env::args().skip(1);
    let inpath = a.next().expect("usage: stream_decode <in|-> <out|-> [--live] [--chunk N]");
    let outpath = a.next().expect("out");
    // --live: ロック後、バックログを捨ててライブエッジから出力する（低遅延・連続出力）。
    //   VLC は入力が一瞬途切れると "buffer deadlock prevented" で停止するため、
    //   ファイル再生ではなくライブ視聴ではこちらを使う。
    let mut live = false;
    let mut chunk: usize = 1 << 18;
    // PAT/PMT 注入。1seg は PAT(PID 0) を欠くため、VLC などのプレイヤーが
    // program を解決できず "buffer deadlock prevented" で停止する。既定で有効。
    let mut inject_psi = true;
    // --follow: 入力が「まだ伸びているファイル」のとき、EOF(=Ok(0)) でも終了せず
    // 新しく書き込まれたぶんを待ち続ける。
    //
    // なぜ必要か（2026-09-26 実測）:
    // パイプ経路 `rtl_sdr | stream_decode` では TS 出力率が実時間の 1.6%
    // （60秒で 978 シンボルしか復号されない、RS は 1 ブロックも出ない）に落ちる。
    // 同じバイナリでファイル入力なら 100%（実時間の 1.5 倍）なのでこれは
    // パイプ経路のスケジューリング問題で、復調性能のせいではない。
    //
    // rtl_sdr → FIFO/ファイル（ continuously 追記）→ stream_decode --follow
    // とすれば、ファイル経路の実時間性能（60,761 B/s）を保ったまま
    // 無限に復調・再生できる。
    let mut follow = false;
    // PSI の再送間隔（TS パケット数）。約 0.5 秒ごと。
    let psi_period: usize = 10_000 / 188;
    let mut psi_count = psi_period;
    // PCR の注入周期（TS パケット数）。MPEG-TS の推奨は PCR 間隔 100 ms 以下。
    // 1 TS パケット ≒ 1 ms なので 20 パケット ≒ 20 ms。
    //
    // 1 反復で 30〜50 パケット出るため、40 パケット周期だと 1 反復に 1 本
    // しか入らず実効間隔が 1〜2 秒に延びる。実測では 30 秒の TS に PCR が
    // 52 本しか出ず、VLC が 5 秒以上の遅延を検知してフレームを落とし続けた
    // （実測 2026-09-26）。20 パケットなら毎反復で入る。
    let pcr_period: u32 = 20;
    let mut pcr_countdown: u32 = pcr_period;
    let mut pcr_ticks: u64 = 0;
    // PAT/PMT の continuity counter。注入ごとに 0 に戻すと libdvbpsi が
    // 「TS discontinuity (received 0, expected N) for PID 0/8136」を出し続け、
    // PSI を信用せず demux を再起動して Position だけ進む状態になる
    // （実測 2026-09-26: 映像も音も出ない / buffer deadlock prevented）。
    // ContinuityTracker も PSI の CC を正規化してしまう。既に
    // psi_packets が正しい CC を付けているので、PSI PID は
    // Tracker に触らせない（PSI_PIDS と同じ_pid）。
    //
    // 実測: Tracker が PMT の CC を `...,9,1,10,...` に書き換えていた
    // （9→10 ではなく 1 になる = 別のカウンタ状態と混線）。
    //
    // PAT と PMT の continuity counter は**別々**に回す。
    //
    // 実測 2026-09-26: 1 つのカウンタを共有すると PMT の列が
    // `0,1,...,11,8,9,...` と折り返し、libdvbpsi が
    // `TS discontinuity (received 13, expected 1) for PID 0` を出し、
    // VLC は demux を再起動して 0:00 のまま止まった。
    // 逆に両方 0 固定にすると `TS duplicate (received 0, expected 1)`。
    let mut pat_cc: u8 = 0;
    let mut pmt_cc: u8 = 0;
    // PCR を入れる映像 PID の continuity counter。
    let mut pcr_cc: u8 = 0;
    let mut cc_track = isdbt_dsp::ts::ContinuityTracker::new();
    // PES の PTS は放送側の絶対時刻（実測 71,611 秒）なので、注入する PCR（0 起点）
    // と 71,611 秒ずれて VLC が `Timestamp conversion failed` を出し、
    // 音声がノイズになる。PCR 起点に合わせて平行移動する。
    let mut pts_norm = isdbt_dsp::ts::PtsNormalizer::new();
    while let Some(flag) = a.next() {
        match flag.as_str() {
            "--live" => live = true,
            "--follow" => follow = true,
            "--chunk" => chunk = a.next().expect("--chunk N").parse().expect("chunk"),
            "--no-psi" => inject_psi = false,
            other => panic!("unknown flag: {other}"),
        }
    }

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
    dec.set_live(live);
    eprintln!("mode: {}{}", if live { "live" } else { "file" }, if live { "（低遅延）" } else { "" });
    let mut raw = vec![0u8; chunk];
    // Keep TS until the first H.264 SPS/PPS/IDR has arrived.  A live receiver
    // commonly starts between GOPs; forwarding P-slices before the first SPS
    // makes ffmpeg report "no frame" and discards the decodable start.
    let mut pending_ts: Vec<u8> = Vec::new();
    let mut started = false;
    let mut announced = false;
    loop {
        // ファイル入力 + --follow なら、書き手が居る間は EOF でも終了しない。
        // `writer_alive` は別の monitor が 0/1 で書き込むファイル。
        let n = match reader.read(&mut raw) {
            Ok(0) | Err(_) => {
                if !follow {
                    break;
                }
                if !writer_alive() {
                    eprintln!("入力が EOF（書き手も不在）— 終了");
                    break;
                }
                // 50ms だけ待って続きを読み直す（低遅延を優先）。
                std::thread::sleep(std::time::Duration::from_millis(50));
                continue;
            }
            Ok(n) => n,
        };
        let mut ts = dec.feed(&raw[..n]);
        // 復調で落ちたパケットの穴を continuity counter で詰める。これが無いと
        // 映像 PID の CC が 99.9% 不一致になり、デコーダが以降の全ストリームを
        // continuity error として破棄する（実測: 画面が壊れながら再生）。
        if !ts.is_empty() {
            cc_track.normalize(&mut ts);
            pts_norm.normalize(&mut ts);
        }
        if std::env::var("ISDBT_DEBUG").is_ok() {
            let (rc, rb) = dec.rs_error_stats();
            let (_, _, drop, mis) = dec.rs_quality();
            eprintln!(
                "[dbg] in={n}B out={}B locked={} backlog={}sym rs_err={rc}/{rb}blk rs_drop={drop} rs_mis={mis}",
                ts.len(),
                dec.is_locked(),
                dec.backlog_syms(),
            );
            // 復調パイプライン各段の通過数（94% 欠損の切り分け用）
            eprintln!(
                "[pipe] bits={} bytes={} drop_comm={} drop_lat={} to_rs={} nblk={}",
                dec.dbg_bits(), dec.dbg_bytes(), dec.dbg_drop_commutator(),
                dec.dbg_drop_latency(), dec.dbg_bytes_to_rs(), dec.dbg_nblk(),
            );
            eprintln!(
                "[sym] calls={} warmup={} carriers={} bits/sym={:.1} bits/carrier={:.2}",
                dec.dbg_syms(), dec.dbg_syms_warmup(), dec.dbg_carriers(),
                dec.dbg_bits() as f64 / dec.dbg_syms().max(1) as f64,
                dec.dbg_bits() as f64 / (dec.dbg_syms() * dec.dbg_carriers()).max(1) as f64,
            );
            // 等化器の時間的ドリフト監視。h 平均電力が累積平均の半分を
            // 下回ると「復調境界のずれ」か「受信条件の変化」を疑う。
            eprintln!(
                "[chan] h_avg={:.2} 位相分散={:.4} CFO={:+.5} sym_mod4 {} ソフト={:.4} 劣化={}",
                dec.dbg_h_avg(), dec.dbg_h_phase_var(), dec.dbg_cfo_slope(),
                if dec.dbg_sym_mod4_used() == dec.dbg_sym_mod4_actual() { "OK" } else { "MISMATCH" },
                dec.dbg_soft_abs(), dec.dbg_soft_degraded(),
            )
        }
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
                    // PAT/PMT を先頭に差し込む。1seg は PAT を欠くため、
                    // VLC などのプレイヤーが program を解決できない。
                    if inject_psi {
                        for p in isdbt_dsp::psi::psi_packets(pat_cc, pmt_cc) {
                            let _ = out.write_all(&p);
                        }
                        pat_cc = pat_cc.wrapping_add(1);
                        pmt_cc = pmt_cc.wrapping_add(1);
                    }
                    // Drop the initial P-slice-only prefix.  Start at a TS
                    // packet boundary at or before SPS so ffmpeg sees a
                    // complete PES containing SPS/PPS/IDR.
                    let start = (sps / 188) * 188;
                    if let Err(e) = out.write_all(&pending_ts[start..]) {
                        eprintln!("出力エラー: {e}");
                    }
                    pending_ts.clear();
                    started = true;
                    eprintln!("SPS/PPS/IDR検出: TS offset {} 以降を出力", start);
                }
            } else {
                // PCR を定期注入する。復調した 1seg TS には PCR が 1 本も
                // 無いため、これが無いと VLC も ffplay も 0:00 のまま止まる。
                //
                // PID は**映像 PID**。PMT の PCR_PID と一致していないと
                // タイムスタンプ源として使われず、VLC が
                // "more than 5 seconds of late video -> dropping frame" を
                // 繰り返して映像が静止画のままになる（実測 2026-09-26）。
                // CC も映像 PID の連番に合わせる。
                //
                // 周期は**TS パケット数**で数える。1 反復で 30〜50 パケット出る
                // ため、反復数で数えると注入間隔が 1〜2 秒に伸びる。かつては 1
                // ずつ減らしていたので 40 パケット経過しても 1 回しか発火せず
                // 「30 秒の TS に PCR 2 本」になっていた（実測 2026-09-26）。
                let pkts = (ts.len() / 188) as u32;
                if pcr_countdown <= pkts {
                    pcr_countdown = 0;
                } else {
                    pcr_countdown -= pkts;
                }
                if pcr_countdown == 0 {
                    // 1 TS パケット ≒ 1 ms。40 ms 周期にする。
                    pcr_ticks = pcr_ticks.wrapping_add(27_000_000 * (pcr_period as u64) / 1000);
                    pcr_cc = pcr_cc.wrapping_add(1);
                    let _ = out.write_all(&isdbt_dsp::psi::pcr_packet(
                        isdbt_dsp::psi::PID_VIDEO,
                        pcr_cc,
                        pcr_ticks,
                    ));
                    pcr_countdown = pcr_period;
                }

                if inject_psi && psi_count == 0 {
                    // PSI は**無限に**周期再送する。カウンタを 0 で止めると
                    // 再送が止まり、VLC は PMT を 1 個しか受け取れず以降すべての
                    // PID を unknown として 0:00 のまま止まる
                    // （実測 2026-09-26: "first packet for pid=8136 cc=0xb" のみ）。
                    for p in isdbt_dsp::psi::psi_packets(pat_cc, pmt_cc) {
                        let _ = out.write_all(&p);
                    }
                    pat_cc = pat_cc.wrapping_add(1);
                    pmt_cc = pmt_cc.wrapping_add(1);
                    psi_count = psi_period;
                }
                if psi_count > 0 {
                    psi_count -= 1;
                }
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
