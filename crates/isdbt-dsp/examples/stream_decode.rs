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
    // --vlc-ready: ファイル出力のあと automatically に ffmpeg remux を通し、
    // VLC / ffplay が確実に開ける TS に書き換える（既定で有効）。
    //
    // なぜ必要か（実測 2026-09-29）:
    // demod が出す生 TS は以下を満たさない。
    //   - null パケット（PID 0x1FFF）が全体の 18%。1seg の帯域が 400 kbps しか
    //     なく、 stuffing が元から多い。生 TS では demod が 1 本ずつ排出する。
    //   - PCR 専用パケット (afc=0b10) として `psi::pcr_packet` を注入するが、
    //     libdvbpsi は PCR_PID 上の CC を映像 PID の CC と共有とみなすことがある。
    // ffmpeg の `-c copy` remux はこれらを是正し、 PID を 0x0100/0x0101 に
    // 振り替え、 null を落とし、 PSI/PCR を正規化する。実測で
    // `duration` 36.08 秒・H.264 320x180・MB エラー 159 件で再生できる。
    //
    // stdout（パイプ）へ出すときは live に途切れずolerant 必要性があるため
    // remux しない（呼び出し側が ffplay へ直結する）。
    let mut vlc_ready = outpath != "-";
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
            "--vlc-ready" => vlc_ready = true,
            other => panic!("unknown flag: {other}"),
        }
    }

    // `--follow` で IQ ファイルを読むとき、外部のトリマー（iq_trim.py）が
    // `fallocate --collapse-range` で**ファイル先頭を潰す**ことがある。
    // collapse は inode を保ったまま内容を左へ詰めるので、ファイル上の
    // オフセットは変わらないまま**内容だけ n バイト分ずれる**。
    //
    // 放置すると reader が「すでに消えた領域」を読み続ける（= 復調が壊れる）。
    // トリマーは `/tmp/isdbt_collapsed` に累積 collapse バイト数を書き、
    // ここでは 250ms ごとにそれを読んで**オフセットを補正**する。
    //
    // 補正は `SeekFrom::Current(-delta)`。累積値なので差分だけ戻す。
    // 復調器の状態（等化器・Viterbi・PRBS 位相）はそのままよい。再取得
    //（stream.rs REACQUIRE_EVERY=800 シンボル）が自力で境界を取り直す。
    //
    // **File は 1 つだけ開く**。2 つ開くと `iq_trim.py` が `/proc/*/fd` を
    // 走査して pos を読む際に、どちらの pos を使うか分からなくなる
    // （実測 2026-10-04 12:07: fd 3 と fd 4 が同じファイルを指し、fdinfo/3 の
    // pos は 0 のまま → stream_decode が 1 バイトも読まない）。
    let mut input_file: Option<fs::File> = if inpath == "-" {
        None
    } else {
        Some(fs::File::open(&inpath).expect("in"))
    };
    // 累積値は IQ ファイルごとに `isdbt_iq_<freq>.iq.collapsed` に分かれる。
    // 単一ファイル共有だとチャンネル切替後に前のチャンネルの値が contaminate
    // し、pos=0 から `SeekFrom::Current(-n)` が EINVAL で失敗する
    // （実測 2026-10-04 12:43、ch14 へ切り替えた直後に発生）。
    let collapsed_path = format!("{inpath}.collapsed");
    let mut collapsed_seen: u64 = 0;
    let use_file = input_file.is_some();
    let stdin_lock = if use_file { None } else { Some(std::io::stdin().lock()) };
    let mut reader: Box<dyn Read> = match stdin_lock {
        Some(l) => Box::new(l),
        None => Box::new(std::io::empty()),
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
        // 外部トリマーの collapse 通知を監視し、ファイルオフセットを補正する。
        //
        // `fallocate --collapse-range --offset 0 --length n` は [0, n) を消して
        // 残りを左へ詰める。ファイル上のオフセットは変わらないので、reader は
        // 「すでに消えた n バイト」を読み続ける。ここで `pos -= n` 補正する。
        //
        // 補正しない場合の実害（実測 2026-10-04 12:03）:
        //   collapse 後に pos が sz を超えると reader は常に EOF を読む。
        //   `--follow` は writer_alive なら終了しないので**無言で空振り**し、
        //   TS が 1 バイトも出力されなくなる。
        if follow && input_file.is_some() {
            if let Ok(txt) = fs::read_to_string(&collapsed_path) {
                if let Ok(total) = txt.trim().parse::<u64>() {
                    if total > collapsed_seen {
                        let delta = total - collapsed_seen;
                        collapsed_seen = total;
                        use std::io::Seek;
                        if let Some(f) = input_file.as_mut() {
                            // 補正できない（既にファイル末尾を越えた）場合は
                            // ログに残す。reader は EOF 読みで空振りになるので、
                            // 復調器の再取得（800 シンボル周期）に賭けるしかない。
                            let newpos = f.stream_position().unwrap_or(0);
                            let se = f.seek(std::io::SeekFrom::Current(-(delta as i64)));
                            eprintln!(
                                "[collapse] 累積={collapsed_seen}B 補正=-{delta}B \
                                 pos {newpos} → {} ({:?})",
                                newpos.saturating_sub(delta),
                                se.err()
                            );
                        }
                    }
                }
            }
        }
        let n = match if use_file {
            match input_file.as_mut().unwrap().read(&mut raw) {
                Ok(v) => Ok(v),
                Err(e) => Err(e),
            }
        } else {
            reader.read(&mut raw)
        } {
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
            let seen = dec.rs_blocks_seen();
            // `symerr` = RS が推定した symbol 誤り数の累計。
            // `rs_bit_errors`（`訂正bit`）は 16 で頭打ちなので診断に使えない。
            let (se, _) = dec.rs_symbol_error_stats();
            let spb = se as f64 / seen.max(1) as f64;
            let fr = dec.rs_fail_reasons();
            // 直近の (ブロック番号, depu_pos % 4)。1 OFDM フレーム = 64 ブロック
            // なので、**64 ブロック周期で同じ値へ戻る**のが正しい。
            // 戻らなくなれば depuncture の位相がスリップしている = 復調が壊れる。
            // `soft` = Viterbi へ入る前のソフト値の自信度（|x|、理想 1.0）。
            // これが 1.0 のままなのに RS が全滅するなら、demap 以降の問題。
            // 逆にこれが落ちていれば、等化器/demap 段が壊れている。
            let sf = dec.dbg_soft_abs();
            let havg = dec.dbg_h_avg();
            // `m4` = 機械的に決めた sym_mod4 と、信号から測った実際の位相。
            // **不一致なら pilot 配置が 1 象限ずれ**て等化器が「健全な誤り」を出す。
            let (m4u, m4a) = (dec.dbg_sym_mod4_used(), dec.dbg_sym_mod4_actual());
            // TS 同期バイト 0x47 の保持率 = byte 整列が保たれているか。
            let (syc, sy) = dec.dbg_rs_sync();
            // Viterbi トレリスの内部状態。
            // `vspread` = 最良-最酷 metric 乖離（トレリスの「判断力」）
            // `vfin`   = 有限状態数（64 全部生きていれば健全）
            // これらは Viterbi 内部の劣化を検出する唯一の窓。
            let (vn, vsp, vfin, vq) = dec.dbg_viterbi();
            let _ = vn;
            let dp = {
                let v = dec.dbg_depu_phase();
                let tail: Vec<(usize, usize)> = v.iter().rev().take(4).rev().copied().collect();
                tail.iter().map(|(b, p)| format!("{b}:{p}")).collect::<Vec<_>>().join(",")
            };
            let dc = dec.disc_count();
            eprintln!(
                "[dbg] in={n}B out={}B locked={} backlog={}sym 訂正blk={rc} 訂正bit={rb} 総blk={seen} drop={drop} mis={mis} symerr={se} symerr/blk={spb:.2} fail={fr:?} depu={dp:?} soft={sf:.4} h={havg:.2} m4={m4u}/{m4a} sync={sy}/{syc} vspread={vsp:.2} vfin={vfin} vq={vq} disc={dc}",
                ts.len(),
                dec.is_locked(),
                dec.backlog_syms(),
            );
            // 診断: SP 1 本単位の異常率（`ISDBT_SPBAD=1`）。
            //
            // 判定: 全 k が同程度なら SNR 劣化（確率的）、
            // 特定の少数の k だけ突出なら `prbs_pilot_values()` の
            // ビットレベルのバグ（決定的）。
            if std::env::var("ISDBT_SPBAD").is_ok() {
                let rates = dec.dbg_sp_bad_rate();
                for ph in 0..4 {
                    let mut line = format!("[spbad] ph={ph}");
                    for k in 0..36 {
                        if rates[ph][k] >= 0.0 {
                            line.push_str(&format!(" {:.2}", rates[ph][k]));
                        }
                    }
                    eprintln!("{line}");
                }
            }
            // 復調パイプライン各段の通過数
            eprintln!(
                "[pipe] bits={} bytes={} drop_comm={} drop_lat={} to_rs={} nblk={}",
                dec.dbg_bits(), dec.dbg_bytes(), dec.dbg_drop_commutator(),
                dec.dbg_drop_latency(), dec.dbg_bytes_to_rs(), dec.dbg_nblk(),
            );
            // 診断: 1 シンボルの平均/最大処理時間。CPU 律速かの判定に使う。
            {
                let (tn, tsum, tmax) = dec.dbg_timing();
                if tn > 0 {
                    eprintln!(
                        "[time] syms={} avg={:.1}us max={}us 計={:.1}s",
                        tn, tsum as f64 / tn as f64, tmax, tsum as f64 / 1e6
                    );
                }
            }
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
            );
            // 診断: soft クランプ（±8.0）の飽和率。飽和が増えると soft 平均は
            // 上がるが解像度が失われ、トレリスのパス間メトリック差が縮む。
            {
                let (tot, sat, nan) = dec.dbg_saturation();
                if tot > 0 {
                    eprintln!(
                        "[sat] 総={} 飽和={} ({:.4}%) 非有限={} ({:.4}%)",
                        tot, sat, sat as f64 / tot as f64 * 100.0,
                        nan, nan as f64 / tot as f64 * 100.0
                    );
                }
            }
            // 診断: Viterbi トレリスの健全性。metric 乖離が狭まる /
            // 有限状態数が減る / out キューが枯渇するなら、トレリスが
            // 「判断不能」に落ちている。乗離は理論上 depth 程度に張り付くはず。
            {
                let (vn, vspread, vfin, vout) = dec.dbg_viterbi();
                if vn > 0 {
                    eprintln!(
                        "[vit] n={} 乖離={:.3} 有限状態={}/64 out長={}",
                        vn, vspread, vfin, vout
                    );
                }
            }
            // 診断: 整列パラメータ（commutator / reset_off / block_phase）。
            // これらは起動時 1 回だけ決まる。後半で真の整列位置からずれるなら
            // 「固定値だが他の量がドリフトする」構造の証拠になる。
            {
                let (cm, ro, bp) = dec.dbg_align_params();
                eprintln!("[align] commutator={} reset_off={} block_phase={}", cm, ro, bp);
            }
            // 診断: 等化後の外挿発散。外挿 carrier で `|H|^2` が閾値を超えると
            // そこで soft が ±8 に張り付き、RS 訂正限界を超える。
            // これが時間とともに増えるのか定常的なのか、時系列で判定する。
            if std::env::var("ISDBT_EQSTAT").is_ok() {
                let (hot, n, pos, ema) = dec.dbg_eq_hot();
                if n > 0 {
                    // 分子・分母の**瞬時平均**（区間_reset しない）。
                    // 累積平均は変化が見えないので、区間ごとに作り直す。
                    let (y, h, h2, y2) = dec.dbg_eq_win();
                    // SP 位置だけの窓平均。FFT 窓ずれの影響が HERE で出る。
                    let (ysp, hsp) = dec.dbg_eq_sp();
                    eprintln!(
                        "[eqhot] hot={} n={} ({:.3}%) maxEMA={:.2} |Y|={:.4} |H|={:.4} EQ={:.4} y2={:.4} Ysp={:.4} Hsp={:.4} bf={:+.4} SegSP={:.4} Ph=[{:.3},{:.3},{:.3},{:.3}] CI={:.4} INC={:.3} R204={:.4} R205={:.4} R216={:.4} R100={:.4} I204={:.4} I205={:.4} I216={:.4} I100={:.4} PV={:#010x} T204={:.5} T210={:.5} T192={:.5} T216={:.5} T100={:.5} T207={:.5} RN={} RJ={}",
                        hot, n, hot as f64 / n as f64 * 100.0, ema, y, h, h2, y2, ysp, hsp,
                        dec.dbg_boundary_frac().0, dec.dbg_seg_sp(), {
                            let p = dec.dbg_seg_sp_phase();
                            (p[0], p[1], p[2], p[3])
                        }.0,
                        {
                            let p = dec.dbg_seg_sp_phase();
                            (p[0], p[1], p[2], p[3])
                        }.1,
                        {
                            let p = dec.dbg_seg_sp_phase();
                            (p[0], p[1], p[2], p[3])
                        }.2,
                        {
                            let p = dec.dbg_seg_sp_phase();
                            (p[0], p[1], p[2], p[3])
                        }.3,
                        dec.dbg_sp_coherence(), dec.dbg_sp_incoh_frac(),
                        dec.dbg_track_pairs()[0],
                        dec.dbg_track_pairs()[1],
                        dec.dbg_track_pairs()[2],
                        dec.dbg_track_pairs()[3],
                        dec.dbg_track_pairs()[6],
                        dec.dbg_track_pairs()[7],
                        dec.dbg_track_pairs()[8],
                        dec.dbg_track_pairs()[9],
                        dec.dbg_pval204().0,
                        dec.dbg_ts_delta()[0],
                        dec.dbg_ts_delta()[1],
                        dec.dbg_ts_delta()[2],
                        dec.dbg_ts_delta()[3],
                        dec.dbg_ts_delta()[4],
                        dec.dbg_ts_delta()[5],
                        dec.dbg_raq_miss().0,
                        dec.dbg_raq_miss().1
                    );
                    // 診断: 絶対 spectrum bin 502..=522 の `|seg[k]|²`。
                    // `l` ではなく spectrum 位置で null の幅と位置を見る。
                    let absv = dec.dbg_abs_profile();
                    let mut aout = String::new();
                    for (i, v) in absv.iter().enumerate() {
                        aout.push_str(&format!("{:.5} ", v));
                    }
                    eprintln!("[absprof] {}", aout);
                    // null profile を出力する。11 キャリアの ema をそれぞ���
                    // `[prof]` として 1 行。
                    if std::env::var("ISDBT_PROF").is_ok() {
                        let pr = dec.dbg_profile();
                        let mut pl = format!("[prof] hot={:.3}", hot as f64 / n as f64 * 100.0);
                        for v in pr.iter() {
                            pl.push_str(&format!(" {v:.5}"));
                        }
                        eprintln!("{pl}");
                    }
                    // 位置Diagnose: 頻度が時間とともに増えるのか、
                    // 位置（周波数）が固定なのか拡大するかを切り分ける。
                    // 432 bin のうち発散した bin の分布を dump する。
                    if std::env::var("ISDBT_EQPOS").is_ok() && n > 0 && n % 200_000 < 60_000 {
                        let mut s = String::new();
                        let tot: u64 = pos.iter().sum();
                        if tot > 0 {
                            for (b, &v) in pos.iter().enumerate() {
                                if v * 200 > tot {
                                    s.push_str(&format!("{}:{}({:.1}%) ", b, v,
                                        v as f64 / tot as f64 * 100.0));
                                }
                            }
                        }
                        eprintln!("[eqpos] 総={} {}", tot, s);
                    // **実キャリア番号 `l`** 単位の分布。旧 `[eqpos]` は
                    // `i*432/n` の 432 等分 bin なので 4 キャリアずつ
                    // まとめられていた（実測ミス 3）。ここでは `l` 精度。
                    // SP 帯域像の前半/後半比較。周波数選択性フェージング
                    // なら山形全体が「うねる」、1 本だけなら l=204 固有。
                    if std::env::var("ISDBT_SPBAND").is_ok() {
                        let (a, na, b, nb) = dec.dbg_sp_band_split();
                        if nb > 0 {
                            let mut s = String::new();
                            for l in 0..432 {
                                if a[l] > 0.0 && b[l] > 0.0 {
                                    s.push_str(&format!(
                                        "{l}:{:.3}/{:.3} ",
                                        a[l].sqrt(),
                                        b[l].sqrt()
                                    ));
                                }
                            }
                            eprintln!("[spband] na={na} nb={nb} {s}");
                        }
                    }
                    if std::env::var("ISDBT_HOTL").is_ok() {
                        let hl = dec.dbg_hot_carriers();
                        let mut lt = String::new();
                        for (l, &v) in hl.iter().enumerate() {
                            if v * 100 > tot {
                                lt.push_str(&format!("{l}:{v} "));
                            }
                        }
                        eprintln!("[hotl] 総={tot} {lt}");
                    }
                    }
                }
            }
            // 診断: `boundary_frac` の値域。設計上の前提は [-0.5, 0.5]。
            // これを外えると線形補間の重みも繰り上がりも破綻する。
            if std::env::var("ISDBT_FRACSTAT").is_ok() {
                let f = dec.dbg_boundary_frac();
                eprintln!("[frac] boundary_frac={:+.4} timing_frac={:+.4} frac_carry={:+.4}", f.0, f.1, f.2);
            }
            // 診断: `cur` と 1 シンボル長 / 1 OFDM フレームの整除性。
            // FFT 窓境界が一貫していれば `cur % sym` は常に 0。
            // これが時間とともにずれていれば、境界ドリフトが原因。
            if std::env::var("ISDBT_CUREM").is_ok() {
                eprintln!(
                    "[currem] cur={} %sym={} %frame={} k={}",
                    dec.dbg_cur(), dec.dbg_cur_rem(), dec.dbg_cur_frame_rem(),
                    dec.dbg_syms()
                );
            }
            // 診断: DC 固定値 vs 局所平均。差が開いているなら DC ドリフト。
            // `self.cur`（絶対インデックス）と `self.buf.len()` も出す。
            eprintln!(
                "[dc] 固定=({:+.4},{:+.4}) 局所=({:+.4},{:+.4}) 差=({:+.4},{:+.4}) cur={} buf={}",
                dec.dbg_dc_fixed(), dec.dbg_dc_fixed_im(),
                dec.dbg_dc_local(), dec.dbg_dc_local_im(),
                dec.dbg_dc_local() - dec.dbg_dc_fixed(),
                dec.dbg_dc_local_im() - dec.dbg_dc_fixed_im(),
                dec.dbg_cur(), dec.dbg_buf_len(),
            );
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
                    // PCR は 27 MHz カウンタ。周期 20 パケットなら +20 ms。
                    pcr_ticks = pcr_ticks.wrapping_add(
                        27_000_000u64 * (pcr_period as u64) / 1000u64,
                    );
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
    // 診断: 落下 block の `block_idx % 256` 分布。1 [dbg] 間隔(214 blk)あたり
    // 36 個が常に落ちるので、特定の block 位置が構造的に落ちているかを見る
    // （commutator / reset_off / block_phase の周期 64 block との関連）。
    // 診断: 1 TSD あたりの degraded 数のヒストグラム。
    // 平均が動かないまま「8 以上」（RS 訂正限界）の占比だけ増えれば、バースト性
    // （クラスタリング）が原因。RS は t=8 の閾値判定なので、正常工作域は 0-7。
    if std::env::var("ISDBT_HIST").is_ok() {
        let (h, n) = dec.dbg_degraded_hist();
        let mut s = String::new();
        for (i, &v) in h.iter().enumerate() {
            if v > 0 {
                s.push_str(&format!("{}:{}({:.1}%) ", i, v, v as f64 / n.max(1) as f64 * 100.0));
            }
        }
        eprintln!("[hist] ブロック数={} {}", n, s);
    }
    // 診断: 連続訂正不能（バースト）長の分布と最長値。
    // `BURST_THRESHOLD`=32 が発動条件なので、**32 を超えるバーストが
    // 1 つもなければ discontinuity 注入は一度も起作用していない**。
    // 診断: TMCC フレーム境界の実測ブロック間隔。
    //
    // gr-isdbt `tmcc_decoder_1seg_impl.cc` は PRBS リセットの基準を
    // **TMCC 同期語が現れる位置**（`d_frame_end`）から作る。我々は
    // RS ブロック番号の 64 周期でリセットしており、この 2 つが
    // 一致する保証はない。ずれれば後半で PRBS 位相がずれて RS が壊れる。
    // 段別バイトダンプ（`ISDBT_DUMP=<blk>`）。
    // 「どこで情報が失われるか」を 1 ブロック単位で追う。
    if std::env::var("ISDBT_DUMP").is_ok() {
        let (soft, vit, byte) = dec.dump_stages();
        if !byte.is_empty() {
            let name = std::env::var("ISDBT_DUMP").unwrap();
            let base = format!("/tmp/dump_{}", name);
            // soft: 0/1/2 (2=erasure) を '0'/'1'/'.' に
            let s2: String = soft.iter().map(|&b| match b { 2 => '.', 1 => '1', _ => '0' }).collect();
            let v2: String = vit.iter().map(|&b| if b != 0 { '1' } else { '0' }).collect();
            let hex = |v: &[u8]| v.iter().map(|b| format!("{:02X}", b)).collect::<Vec<_>>().join("");
            std::fs::write(format!("{}.soft.txt", base), &s2).ok();
            // 生の f32（オフライン再生用。量子化すると情報が失われる）
            let f = dec.dump_soft_f32();
            if !f.is_empty() {
                let mut o = String::new();
                for v in &f { o.push_str(&format!("{:.6} ", v)); }
                std::fs::write(format!("{}.softf32.txt", base), o).ok();
            }
            std::fs::write(format!("{}.vit.txt", base), &v2).ok();
            std::fs::write(format!("{}.byte.hex", base), hex(&byte)).ok();
            let blocks = dec.dump_blocks();
            if blocks.len() > 1 {
                for (i, b) in blocks.iter().enumerate() {
                    let hx: String = b.iter().map(|x| format!("{:02X}", x)).collect();
                    std::fs::write(format!("{}.blk{:02}.hex", base, i), hx).ok();
                }
                let mut o = String::new();
                for (i, b) in blocks.iter().enumerate() {
                    let hd = b.iter().take(8).map(|x| format!("{:02X}", x)).collect::<Vec<_>>().join("");
                    o.push_str(&format!("{}:{} ", i, hd));
                }
                eprintln!("[dumpblocks] {} blocks", blocks.len());
                eprintln!("[dumpblocks] {}", o);
            }
            let pre = dec.dump_pre();
            if !pre.is_empty() {
                std::fs::write(format!("{}.pre.hex", base), hex(&pre)).ok();
            }
            eprintln!("[dump] blk={} soft={}B vit={}b byte={}B -> {}",
                name, soft.len(), vit.len(), byte.len(), base);
        }
    }
    // 診断: ロック判定の主張 vs 実運用の RS 符号語率。
    // 診断: **原子的な** 1 ブロック分の記録。
    // soft / vit / RS 入力が同一ブロック由来であることが保証される。
    if std::env::var("ISDBT_ATOM").is_ok() {
        if let Some((sf, vt, by)) = dec.dump_atom() {
            let name = std::env::var("ISDBT_ATOM").unwrap_or_default();
            let base = format!("/tmp/atom_{}", if name.is_empty() { "0" } else { &name });
            let mut o = String::new();
            for v in &sf { o.push_str(&format!("{:.6} ", v)); }
            std::fs::write(format!("{}.softf32", base), o).ok();
            let vb: String = vt.iter().map(|b| if *b != 0 { '1' } else { '0' }).collect();
            std::fs::write(format!("{}.vit", base), vb).ok();
            let hx: String = by.iter().map(|b| format!("{:02X}", b)).collect();
            std::fs::write(format!("{}.byte", base), hx).ok();
            // deinterleaver の idx。offline 再生で同じ出力列を得るために必要。
            let di = dec.dump_deint_idx();
            std::fs::write(format!("{}.didx", base), di.to_string()).ok();
            eprintln!("[atomdump] soft={}B vit={}b byte={}B didx={} -> {}",
                sf.len(), vt.len(), by.len(), di, base);
        }
    }
    if std::env::var("ISDBT_RSVPROBE").is_ok() {
        let (v, n) = dec.dbg_rs_valid();
        if let Some((cm, ro, bp, f)) = dec.dbg_lock_claim() {
            eprintln!(
                "[rsprobe] ロック判定: commutator={} reset_off={} block_phase={} 評価窓RS率={:.3}",
                cm, ro, bp, f
            );
        }
        eprintln!(
            "[rsprobe] 実運用: RS符号語 {}/{} = {:.1}%",
            v, n,
            if n > 0 { v as f64 / n as f64 * 100.0 } else { 0.0 }
        );
    }
    // 診断: PRBS リセットが起きた block_idx の列。
    // 周期 64・offset 17 なら [17, 81, 145, ...] と等間隔になる。
    // 崩れていれば間隔が fleet ずれる = 位相ドリフトの原因。
    if std::env::var("ISDBT_PRSBALL").is_ok() {
        let v = dec.dbg_prbs_resets();
        let mut gaps = String::new();
        for w in v.windows(2) {
            gaps.push_str(&format!("{} ", w[1] - w[0]));
        }
        eprintln!("[prbsall] resets={} 先頭={:?}", v.len(), v.first());
        eprintln!("[prbsall] 間隔 = {}", gaps);
    }
    if std::env::var("ISDBT_TMCCFRM").is_ok() {
        let h = dec.dbg_tmcc_frame_pos();
        let mut gaps = String::new();
        for w in h.windows(2) {
            gaps.push_str(&format!("{} ", w[1] - w[0]));
        }
        eprintln!("[tmccfrm] 境界数={} 先頭={:?}", h.len(), h.first());
        eprintln!("[tmccfrm] ブロック間隔 = {}", gaps);
    }
    if std::env::var("ISDBT_BURST").is_ok() {
        let (h, mx) = dec.dbg_drop_burst();
        let mut out = String::new();
        for (i, c) in h.iter().enumerate() {
            if *c > 0 {
                out.push_str(&format!("{}:{} ", i + 1, c));
            }
        }
        let mut ge = [0u64; 5];
        for (i, c) in h.iter().enumerate() {
            let len = i + 1;
            for (k, thr) in [8usize, 16, 24, 32, 48].iter().enumerate() {
                if len >= *thr {
                    ge[k] += c;
                }
            }
        }
        eprintln!(
            "[burst] max={} dist={} >=8:{} >=16:{} >=24:{} >=32:{} >=48:{}",
            mx, out, ge[0], ge[1], ge[2], ge[3], ge[4]
        );
    }
    // 診断: 全ブロックの `depu_pos % 4`。**復号不能ブロックも含む**ので、
    // 劣化域（drop 100%）でも値が更新され続ける。64 ブロック周期で戻るはず。
    if std::env::var("ISDBT_DEPUALL").is_ok() {
        let d = dec.dbg_depu_all();
        let mut out = String::new();
        for (b, p) in d.iter() {
            out.push_str(&format!("{b}:{p} "));
        }
        eprintln!("[depuall] {}", out);
    }
    // 診断: `depu_pos % 4` の推移。64 ブロック（1 OFDM フレーム）ごとに
    // 同じ値へ戻るかを見る。戻らなければ depuncture の位相がスリップしている。
    if std::env::var("ISDBT_DEPSTAGE").is_ok() {
        let d = dec.dbg_depu_phase();
        // 連続する 2 ブロック間隔で取り、ブロック番号と mod 4 を並べる
        let mut out = String::new();
        for w in d.windows(2) {
            if w[1].0 - w[0].0 == 2 {
                out.push_str(&format!("{}:{} ", w[0].0, w[0].1));
            }
        }
        eprintln!("[depustage] {}", out);
    }
    // 診断: `reacquire` のジャンプ履歴。
    //
    // 「reacquire が徐々に間違った答えを選ぶ確率が上がっている」なら
    // 大きくジャンプした回数が blk 数とともに増える。`Δ` 自体は
    // 0..16 に収まるので絶対値は小さく、**blk 軸での変化**を見る。
    // 診断: `|h[204]|` / `|h[216]|` / `|h[205]|` の瞬時分布。
    //
    // 窓平均は分布の形を隠す。`hot = |Y|²/|H|² > 10` は**瞬時値**の
    // 閾値判定なので、平均 grow と hot 増加が矛盾なく両立するには
    // 「下側の裾が伸びる」ことが必要。4 時点の凍結累積で時系列化。
    if std::env::var("ISDBT_HDIST").is_ok() {
        let (_cur, snap) = dec.dbg_h_dist();
        for (ph, s) in snap.iter().enumerate() {
            for (slot, hst) in s.iter().enumerate() {
                let tot: u64 = hst.iter().sum();
                if tot == 0 {
                    continue;
                }
                let mut out = String::new();
                for (b, c) in hst.iter().enumerate() {
                    if *c > 0 {
                        out.push_str(&format!("{:.3}:{} ", -6.0 + b as f64 / 5.0, c));
                    }
                }
                eprintln!("[hdist] ph={ph} slot={slot} n={tot} {}", out);
            }
        }
    }
    if std::env::var("ISDBT_RAQJUMP").is_ok() {
        let h = dec.dbg_raq_history();
        let mut out = String::new();
        for w in h.chunks(2) {
            if w.len() == 2 {
                let blk = w[0];
                let jump = (w[1] as i64) / 2 - 1000;
                out.push_str(&format!("{blk}:{jump} "));
            }
        }
        eprintln!("[raqjump] {}", out);
    }
    if std::env::var("ISDBT_DROPPOS").is_ok() {
        let (dist, total) = dec.dbg_drop_pos();
        let mut line = String::new();
        for (i, &v) in dist.iter().enumerate() {
            if v > 0 {
                line.push_str(&format!("{}:{} ", i, v));
            }
        }
        eprintln!("[droppos] 合計={} {}", total, line);
    }

    // --- VLC ready remux ---
    //
    // 生 TS は 1seg の帯域制約（400 kbps）と demod 側の PSI 注入 Meehan の
    // 都合で、VLC が確実に開ける形式になっていない。ffmpeg の `-c copy`
    // remux は null 詰め物を落とし、 PID を 0x0100/0x0101 に振り替え、
    // PSI/PCR/CC を正規化するので、**出力ファイル自体を VLC ready にする**。
    //
    // 実測 2026-09-29: remux 前は null 1643/9078 (18%)・CC 違反 82 件で
    // VLC が 0:00 に停止。remux 後は null 0・CC 正常・duration 36.08 秒・
    // H.264 320x180 で正常に再生できる。
    if vlc_ready && outpath != "-" {
        remux_vlc_ready(&outpath);
    }
}

/// 出力量を ffmpeg remux で VLC ready な TS に書き換える（in-place）。
///
/// `ffmpeg` が見つからない場合は警告して元のまま残す（-demod の結果は捨てない）。
fn remux_vlc_ready(path: &str) {
    let ffmpeg = match std::env::var("FFMPEG").ok().or_else(|| {
        // PATH に ffmpeg がない場合に備えて、このワークスペースの同梱版も試す。
        let p = format!(
            "{}/.hermes/tools/ffmpeg-9.0.1-linux-x64/bin/ffmpeg",
            std::env::var("HOME").unwrap_or_default()
        );
        if std::path::Path::new(&p).exists() { Some(p) } else { None }
    }) {
        Some(f) => f,
        None => {
            eprintln!(
                "[remux] ffmpeg が見つからないため VLC ready 化はスキップ。\
                 PATH に ffmpeg を入れるか FFMPEG=/path/to/ffmpeg を指定してください。"
            );
            return;
        }
    };
    let tmp = format!("{path}.vlcready.tmp");
    let st = std::process::Command::new(&ffmpeg)
        .args([
            "-v", "error", "-y",
            // 壊れた DTS を無視して PTS を生成する（実測: そのままでは
            // `Timestamp conversion failed` で VLC が止まる）。
            "-fflags", "+genpts+igndts",
            "-i", path,
            "-c", "copy",
            // バッファ溜めを無効化し低遅延にする。
            "-muxdelay", "0",
            "-f", "mpegts",
            &tmp,
        ])
        .status();
    match st {
        Ok(s) if s.success() => {
            match std::fs::rename(&tmp, path) {
                Ok(()) => {
                    let n = std::fs::metadata(path).map(|m| m.len()).unwrap_or(0);
                    eprintln!(
                        "[remux] VLC ready 化完了: {} バイト（ffmpeg -c copy）",
                        n
                    );
                }
                Err(e) => {
                    eprintln!("[remux] 差し替えに失敗: {e}（{tmp} に書き出し済み）");
                }
            }
        }
        _ => {
            eprintln!("[remux] ffmpeg remux に失敗。元の生 TS のまま残す。");
            let _ = std::fs::remove_file(&tmp);
        }
    }
}
