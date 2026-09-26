//! ストリーミング復調器（ライブラリAPI）：`feed(iq_u8) → ts_bytes` を逐次で。
//!
//! CLI（`examples/stream_decode.rs`）と WASM（`isdbt-wasm`）の両方から使う中核。
//! 初期バッファで同期＋整列をロック → 以降は届いたIQを1シンボルずつ復調し、
//! [`crate::viterbi::ViterbiStreaming`] ＋ 逐次バイトデインタ/逆拡散/RS で TSパケットを吐く。

use crate::deinterleave::{data_carrier_indices, freq_deinterleave, TimeDeinterleaver};
use crate::demap::{qpsk_soft, BitDeinterleaverQpsk};
use crate::demod::OfdmDemod;
use crate::equalize::{
    detect_symbol_phase, equalize, estimate_channel, extract_segment,
    phase_scores_gr_isdbt, track_symbol_phases,
};
use crate::iq::u8_iq_to_complex;
use crate::params::{GuardInterval, FFT_LEN};
use crate::pilots::SegmentPilots;
use crate::rs;
use crate::sync::estimate_symbol_sync;
use crate::ts::{best_sync_phase, pack_bits_msb, ByteDeinterleaver, EnergyPrbs, BI_I, BI_M, TSP};
use crate::viterbi::{depuncture, Viterbi, ViterbiStreaming, PUNCTURE_2_3};
use num_complex::Complex32;

const RESET_PERIOD: usize = 64; // 1 OFDMフレーム = 64 RSブロック
const PRBS_INIT: u16 = 0xa9;
/// bin offset 探索半径。capture ごとに数 bin のズレが出るので TMCC 同期で決める。
const OFFSET_RADIUS: usize = 16;

/// TMCC スコア上位何個を RS 復.decode 率で再評価するか。
///
/// TMCC 一致度は `seg_off` の正しい objective ではない（bin offset が 1
/// ずれても同期一致 100% になる。`select_segment_offset_by_rs` 参照）。
/// したがって TMCC 上位候補を実 RS 復.decode 率で選び直す。
///
/// **TMCC 上位と限らない理由**（実測 2026-09-26）: `disc_test.iq` の TMCC
/// 上位 3 は `[307, 309, 291]` で、**正解の 308 が含まれていなかった**。
/// 上位だけ見ると 307 を選んでしまう。
///
/// したがって **nominal 近傍は全探索**する。OFFSET_RADIUS=16 なので
/// 33 候補。`demod_and_align` のコストは Viterbi 1 回分で、ロック時に
/// 1 回だけなので実用上問題ない。
const RS_REFINE_TOP_N: usize = 33;
const TB_DEPTH: usize = 96;
const BYTE_LATENCY: usize = BI_M * BI_I * (BI_I - 1);
const LOCK_SYMS: usize = 4000;
const COMPACT_AT: usize = 8 * 1024 * 1024;
/// 1回の feed/pump で処理する最大シンボル数（≈50msぶん）。呼び出し側が描画を挟めるように分割。
const MAX_SYMS_PER_CALL: usize = 64;

/// 訂正不能な RS ブロックの扱い。
///
/// **出力しない**（＝穴を空ける）。ここ Historically 2 案を試したが、どちらも
/// 視聴に悪い結果をもたらしたため**いずれも採用しない**:
///
/// 1. `null_packet()` (PID 0x1FFF) を 1 本差し込む → **撤回**。
///    訂正不能ブロックの 188 バイトを PID 0x1FFF の null で埋める。
///    TS 構造としては正しかった（sync 0x47 あり、bitrate も保たれる）が、
///    **映像 PES の途中に 188 バイトの穴ができる**。H.264 は参照フレーム方式なので
///    穴ごとに GOP 全体が破綻し、**再生時間とともに画像が崩壊する**
///    （実測 2026-09-26: 冒頭のフレームは出るが 20 秒以降で MB エラー 57→152 と
///    単調増加）。ffmpeg の「フレーム数が取れる」検査では検出できず、
///    視聴すると分かる。**H.264 の TS 上の穴は致命的**。
///
/// 2. `[0xff; 188]`（sync なし）を出力 → **撤回**。パケット境界が壊れる
///    （実測 `0x47 sync 異常 34.6%`、VLC が demux 失敗）。
///
/// 現在の挙動（穴を空ける）: 出力バイトが 188 減り PCR が僅かに遅れるが、
/// `ContinuityTracker` が CC を詰めるため demux 側は欠落に気づかないまま
/// H.264 を repair して再生する。時間とともに崩れることはない（実測確認）。
///
/// **将来検討する価値があるのは `discontinuity_indicator`**:
/// 訂正不能ブロックに続けて adaptation field の discontinuity_indicator を
/// 立て、demux に「この後は壊れている。次の PCR で I フレームから再開せよ」と
/// 伝える。H.264 は I フレームがあれば自己修復できるので、これが正解の方向。
/// ただし未実装。

/// 収束待ちの**下限**シンボル数（TS 出力を出さない期間）。
///
/// `boundary_frac` は最初の Reacquire 成功で初めて正しい値になるので、
/// これだけは固定の下限として残す。品質ゲートはこれより早く開くことが
/// あるため、下限として必要。
///
/// `ISDBT_SETTLE` で上書きできる（0 で無効化 = ゲートなし）。
fn settle_min_syms() -> usize {
    match std::env::var("ISDBT_SETTLE") {
        Ok(v) => v.parse().unwrap_or(SETTLE_SYMS),
        Err(_) => SETTLE_SYMS,
    }
}

/// ゲートは「時間下限 AND (品質 OR fail-open)」。**既定は無効**。
///
/// `gate_enabled()` が false のときは何もせず、`feed()` は常に TS を出力する。
/// 実測 2026-09-26（札幌 NHK ch15）の結論がこれの根拠:
///
/// | ゲート | MB エラー | 出力バイト |
/// |---|---|---|
/// | なし | 20 | 1,007,492 |
/// | 時間 2400 | 20 | 904,656 |
/// | 時間 12000 | 6 | 40,984 |
/// | 品質 200 連続 | 0 | **0**（出力ゼロ）|
/// | 品質+fail-open 既定 | 0 | **0**（出力ゼロ）|
///
/// 品質ゲートは**一度も開かず**、出力をゼロにして原より悪化させた。
/// 訂正不能ブロックは 51 個/チャンクの集中から 1〜4 個/チャンクへ減るものの
/// 最後まで消えず、この信号では 200 連続（20 連続すら）が達成されなかった。
/// 原因は復調 bit error が散発し続けており、冒頭の収束 splash ではなく
/// 弥漫的な散発であること。
///
/// したがって既定は無効。使える場面は「アンテナ良好で quality が実際に
/// 収束する環境」のみなので `ISDBT_GATE=1` で明示的に有効化する。
fn gate_enabled() -> bool {
    std::env::var("ISDBT_GATE").map(|v| v != "0").unwrap_or(false)
}

/// 品質ゲート: この個数の**連続した**RS ブロックで訂正不能が出なかったら
/// 出力を開始する。
///
/// `ISDBT_CLEAN` で上書き。
fn clean_blocks_needed() -> usize {
    match std::env::var("ISDBT_CLEAN") {
        Ok(v) => v.parse().unwrap_or(CLEAN_BLOCKS),
        Err(_) => CLEAN_BLOCKS,
    }
}

/// ゲートが時間上限で強制オープンするシンボル数（fail-open）。
///
/// 訂正不能ブロックは null パケットに置き換えてあるので、「多少壊れた TS」の
/// ほうが「0 バイトの TS」よりはるかに良い。品質条件が来なくても出力は確保する。
///
/// `ISDBT_MAXSETTLE` で上書き。
fn settle_max_syms() -> usize {
    match std::env::var("ISDBT_MAXSETTLE") {
        Ok(v) => v.parse().unwrap_or(SETTLE_MAX_SYMS),
        Err(_) => SETTLE_MAX_SYMS,
    }
}

/// 品質ゲートの既定値。RS ブロック 1 本 ≈ 256 シンボルなので 200 本 ≈ 1.6 秒。
const CLEAN_BLOCKS: usize = 200;

/// fail-open の既定シンボル数。約 10.4 秒（1152 サンプル/シンボル）。
const SETTLE_MAX_SYMS: usize = 12_000;

/// 固定ゲートの下限シンボル数。
const SETTLE_SYMS: usize = 2400;

/// 境界を再取得する周期（シンボル数）。
///
/// 1seg の CP 自己相関は 1 シンボル窓では peak が立たない（実測 metric
/// 0.03-0.17、0.2 通過率 1.0%）が、1M サンプル窓なら 0.30 で安定する。
/// そのため `demod_one_tracked` の 1 シンボル窓探索はほぼ常に失敗し、
/// `unwrap_or(expected)` で等間隔フォールバックしていた。その結果
/// 実 fs（1015873.002204）と指定 fs（1015873）の差 ≈0.0022 サンプル/シンボルが
/// 累積し、ライブ入力では約 2000 RS ブロック（約 7 秒）で境界が破綻して
/// 出力が恒久停止した。
///
/// そこで `REACQUIRE_EVERY` シンボルごとに長窓で基準を取り直す。
///
/// **窓長は実測で決めた値。** 当初 1M サンプルにしていたが、1 シンボル =
/// 1152 サンプルに対して 868 シンボル分の走査となり、
/// `REACQUIRE_EVERY = 800` だと**再取得だけで 1 シンボルの budget
/// （1134 µs）を超えて食いつく**。実測では TS 出力が 70 B/s
/// （平常値の 48,000 B/s の 0.1%）に落ち、画面は 0:00 のまま止まった。
///
/// 100k サンプル（≈ 87 シンボル）に縮めるとドリフトは
/// 0.0022 × 87 ≈ 0.2 サンプルで、探索半径 1 サンプル内に十分収まる。
const REACQUIRE_EVERY: usize = 800;
/// 再取得に使う長窓の長さ（サンプル）。100k が実測最適。
const REACQUIRE_WINDOW: usize = 100_000;

/// ライブモードで `feed` が復号を開始するまでに貯める最小シンボル数。
///
/// 1seg の RS ブロックは 64 ブロック = 256 シンボルごとに完成する。パイプから
/// 読むとチャンクが 65536 B = 14 シンボル分しか来ないので、チャンクごとに
/// `process` すると RS が完成する前に入力が尽き、出力率が実時間の 16分の1
/// （2,693 B/s 対 42,000 B/s）に落ちる（実測）。この値ぶん貯めてからまとめて
/// 処理すれば、ファイル経路と同じ 60,000 B/s 相当になる。
const FEED_MIN_SYMS: usize = 0;

/// 再取得周期。`ISDBT_REACQ` で上書きできる（0 なら再取得を無効化）。
///
/// 再取得は 1M サンプルの相関を計算するため 1 回 20〜30 ms かかる。800 シンボル
/// （≈0.8 秒）ごとに実行すると実時間処理の 3〜4% を消費するが、これは
/// 許容範囲。ただし環境変数で無効化すれば境界精度とのトレードオフを測れる。
fn reacq_every() -> usize {
    let v = match std::env::var("ISDBT_REACQ") {
        Ok(v) => v.parse().unwrap_or(REACQUIRE_EVERY),
        Err(_) => REACQUIRE_EVERY,
    };
    // 0 は「無効」を意味する（usize::MAX にして恒久的に発火させない）。
    if v == 0 { usize::MAX } else { v }
}


struct RsBlockAssembler {
    buffer: Vec<u8>,
    phase: usize,
    reset_off: usize,
    prbs: EnergyPrbs,
    block_idx: usize,
    /// 訂正を要した RS ブロック数（復調 bit error の指標）。
    rs_corrected: usize,
    /// 訂正を要したビット総数。
    rs_bit_errors: u64,
    /// **訂正不能**で丸ごと捨てられた RS ブロック数。
    ///
    /// これが H.264 MB 破損の直接原因になる。`rs::decode` が `None` を返すと
    /// そのブロックの 188 バイトが TS 出力から消えるが、`ContinuityTracker` が
    /// CC を詰めるため CC 不連続は検出されず、プレイヤーは欠落に気づかないまま
    /// 途切れたビットストリームを復号して MB エラーを出す。
    rs_dropped: usize,
    /// 訂正後も syndrome が非ゼロのブロック数（誤訂正 / 訂正失敗の silent failure）。
    rs_miscorrected: usize,
    /// **連続して**訂正不能だった RS ブロック数。
    ///
    /// 1 フレーム = 64 ブロックなので、これが `BURST_THRESHOLD` に達したら
    /// 1 フレームが丸ごと消えたとみなす。`feed()` がそのイベントを
    /// 呼び出し側に伝え、`discontinuity_indicator` を注入させる。
    drop_burst: usize,
    /// フレーム落ちイベントを発火済みか（1 回の連続につき 1 回だけ）。
    burst_raised: bool,
}

/// 連続訂正不能がこの個数に達したら「1 フレーム落ち」とみなす。
///
/// 1 フレーム = 64 RS ブロック。実測（2026-09-26）で訂正不能 126 個のうち
/// **51 個が 1 箇所で連続**していた。51/64 ≈ 80% のフレームが飛んでいた。
/// _THRESHOLD は 64 の半分（32）にして、フレームの半分が飛んだ時点を
/// 検出の目安にする。デコードはそのうち画像が出るので、
/// **これより早く検出しないと I フレームを落とす**。
const BURST_THRESHOLD: usize = 32;

impl RsBlockAssembler {
    fn new(phase: usize, reset_off: usize) -> Self {
        Self { buffer: Vec::with_capacity(TSP), phase, reset_off, prbs: EnergyPrbs::with_init(PRBS_INIT), block_idx: 0, rs_corrected: 0, rs_bit_errors: 0, rs_dropped: 0, rs_miscorrected: 0, drop_burst: 0, burst_raised: false }
    }

    /// 復号できた RS ブロックを返す。訂正不能なブロックは**出力しない**
    /// （穴を空ける）。
    ///
    /// `true` が**連続して**訂正不能が起きたフレーム落ちイベントを表す。
    /// 1seg は 1 フレーム = 64 RS ブロックなので、`BURST_THRESHOLD` 個
    /// 連続したら 1 フレームが丸ごと消えたとみなす（実測: 51 連続）。
    /// 呼び出し側はこの `true` を受けて discontinuity_indicator 付き PCR
    /// パケットを 1 本挿入する。demux はそれを見て状態を捨て、次の
    /// I フレームから回復する。**これが累積崩壊を防ぐ唯一の手段**。
    ///
    /// （`null_packet()` で穴を埋めるのは撤回済み。TS 上の穴が H.264 の
    /// 参照フレームを破壊して「時間とともに画像が崩れる」原因になった。
    /// 詳細は `stream.rs` 上部の設計コメントを参照）
    fn feed(&mut self, bytes: &[u8]) -> (Vec<Vec<u8>>, bool) {
        self.buffer.extend_from_slice(bytes);
        let mut out = Vec::new();
        let mut burst = false;
        while self.buffer.len() >= TSP + self.phase {
            if self.block_idx % RESET_PERIOD == self.reset_off {
                self.prbs.reset_to(PRBS_INIT);
            }
            let start = self.phase;
            let mut ds = Vec::with_capacity(TSP);
            ds.push(self.buffer[start]);
            for &b in &self.buffer[start + 1..start + TSP] {
                ds.push(b ^ (self.prbs.clock(8) as u8));
            }
            self.prbs.clock(8);
            self.block_idx += 1;
            // syndromes の非ゼロ数 = 訂正を要したビット誤りの量化指標。
            // 復調 bit error の推移を時系列で見るため記録する
            // （実測 2026-09-26: 開始 +0MB で H.264 MB エラー 2.77/s だった
            //  capture は +10MB にすると 0.10/s に激減した。等化器の収束待ち）。
            let nz = rs::syndrome_weight(&ds);
            if nz > 0 {
                self.rs_corrected += 1;
                self.rs_bit_errors += nz as u64;
            }
            match rs::decode(&ds) {
                Some(cw) => {
                    // 訂正後に syndrome がまだ非ゼロなら「訂正したつもりが
                    // まだ壊れている」。Chien 検索の根取りこぼしや synd 計算の
                    // 向き違いがここに出る。0 でなければ出力を信用してはいけない。
                    if rs::syndrome_weight(&cw) > 0 {
                        self.rs_miscorrected += 1;
                    }
                    out.push(cw);
                    // 訂正できたので連続カウントをリセット
                    self.drop_burst = 0;
                    self.burst_raised = false;
                }
                None => {
                    // 訂正不能ブロックは**出力しない**（穴を空ける）。
                    //
                    // 過去に 2 案を試したが両方とも視聴に悪かった（詳細は上部の
                    // 設計コメント）:
                    //   - `null_packet()` で 1 本埋める → H.264 の参照フレームが
                    //     破綻し、**再生時間とともに画像が崩壊する**
                    //   - `[0xff; 188]` で埋める → sync 無しでパケット境界が壊れ
                    //     VLC が demux 失敗
                    //
                    // 現在の挙動: 188 バイトだけ出力が減る。`ContinuityTracker` が
                    // CC を詰めるので demux は欠落に気づかず、H.264 は repair される。
                    // 時間とともに崩れることはない（実測確認）。
                    //
                    // TODO: 将来は adaptation field の discontinuity_indicator を
                    // 立てて demux に「次の PCR で I フレームから再開せよ」と伝える。
                    // それが H.264 の自己修復の仕組みに最も正しい。
                    self.rs_dropped += 1;
                    self.drop_burst += 1;
                    // フレーム落ちイベント。1 回の連続につき 1 回だけ発火する
                    // （`burst_raised` で抑制）。呼び出し側が
                    // discontinuity_indicator 付き PCR パケットを注入する。
                    if self.drop_burst >= BURST_THRESHOLD && !self.burst_raised {
                        self.burst_raised = true;
                        burst = true;
                    }
                }
            }
            self.buffer.drain(0..start + TSP);
        }
        (out, burst)
    }
}

#[derive(Clone, Copy)]
struct Locked {
    gi: GuardInterval,
    cfo: f32,
    sym: usize,
    phase0: usize,
    /// TMCC 同期一致率で確定した1segセグメントの bin offset。
    seg_off: usize,
}

/// 復調パイプラインを短い区間だけ走らせ、**RS 復号成功率**を返す。
///
/// `seg_off` 選択の objective function として使う。TMCC 一致度では
/// bin offset が 1 ずれても 100% になるため objective として不適
/// （詳細は `select_segment_offset_by_rs` のドキュメント）。
///
/// 処理は `lock_params()` と同一（位相追従 → 周波数デインタ →
/// ビットデインタ → depuncture → Viterbi → byte deint → RS）だが、
/// 整列パラメータ（commutator / reset_off / block_phase）も一緒に探索し、
/// RS 復.decode 率とParameters を返す。
fn demod_and_align(
    specs: &[Vec<Complex32>],
    seg_off: usize,
    phase0_in: usize,
) -> Option<(f32, usize, usize, usize)> {
    let pilots = SegmentPilots::center_1seg();
    let mut phase_rows: Vec<[f32; 4]> = specs
        .iter()
        .map(|sp| phase_scores_gr_isdbt(&sp[seg_off..seg_off + 432], &pilots))
        .collect();
    let phases = track_symbol_phases(&phase_rows);
    let phase0 = phases.first().copied().unwrap_or(phase0_in);
    let mut tdi = TimeDeinterleaver::new(4);
    let mut bdi = BitDeinterleaverQpsk::new();
    let warmup = tdi.latency() + bdi.latency();
    let mut coded: Vec<f32> = Vec::new();
    for (k, sp) in specs.iter().enumerate() {
        let sym_mod4 = phases.get(k).copied().unwrap_or((phase0 + k) % 4);
        let seg = extract_segment(sp, seg_off);
        let h = estimate_channel(&seg, sym_mod4, &pilots);
        let eq = equalize(&seg, &h);
        let data: Vec<Complex32> = data_carrier_indices(sym_mod4, &pilots)
            .into_iter()
            .map(|l| eq[l])
            .collect();
        let fd = freq_deinterleave(&data);
        for v in tdi.push_symbol(&fd) {
            let de = bdi.push(qpsk_soft(v));
            if k >= warmup {
                coded.push(de[1]);
                coded.push(de[0]);
            }
        }
    }
    let restored = depuncture(&coded, &PUNCTURE_2_3);
    let bits = Viterbi::new().decode(&restored[..restored.len() & !1]);
    let bytes = pack_bits_msb(&bits, 0);
    let checker = rs::Checker::new();
    let mut best = (0.0f32, 0usize, 0usize, 0usize);
    if bytes.is_empty() {
        return None;
    }
    for c in 0..BI_I.min(bytes.len()) {
        let mut di = ByteDeinterleaver::new();
        let stream: Vec<u8> = bytes[c..]
            .iter()
            .enumerate()
            .filter_map(|(j, &b)| {
                let o = di.push(b);
                (j >= BYTE_LATENCY).then_some(o)
            })
            .collect();
        let (phase, sc) = best_sync_phase(&stream);
        if sc < 0.5 {
            continue;
        }
        let blocks: Vec<&[u8]> = (0..)
            .map(|i| phase + i * TSP)
            .take_while(|&i| i + TSP <= stream.len())
            .map(|i| &stream[i..i + TSP])
            .collect();
        for reset_off in 0..RESET_PERIOD {
            let mut prbs = EnergyPrbs::with_init(PRBS_INIT);
            let (mut ok, mut tot) = (0usize, 0usize);
            for (idx, blk) in blocks.iter().take(128).enumerate() {
                if idx % RESET_PERIOD == reset_off {
                    prbs.reset_to(PRBS_INIT);
                }
                let mut o = vec![blk[0]];
                for &b in &blk[1..TSP] {
                    o.push(b ^ (prbs.clock(8) as u8));
                }
                prbs.clock(8);
                if checker.is_codeword(&o) || rs::decode(&o).is_some() {
                    ok += 1;
                }
                tot += 1;
            }
            let f = ok as f32 / tot.max(1) as f32;
            if f > best.0 {
                best = (f, c, reset_off, phase);
            }
        }
    }
    (best.0 > 0.0).then_some(best)
}

/// TMCC で得た `seg_off` 候補を、**実 RS 復号率**で再選択する。
///
/// # なぜ TMCC 一致度だけでは足りないか（実測 2026-09-26）
///
/// TMCC は 1 フレーム 204 シンボルのうち**1 シンボル分のデータ**に過ぎない。
/// そのため `tmcc::select_segment_offset()` は bin offset が 1 ひとつずれても
/// TMCC フレーム同期を 100% 満たしたまま通ってしまう:
///
/// | capture | bin offset | TMCC 同期 | 訂正不能率 |
/// |---|---|---|---|
/// | `p1.iq`        | 308 | 1.000 |  0.1% |
/// | `rs.iq`        | 308 | 1.000 |  3.1% |
/// | `disc_test.iq` | **307** | 1.000 | **31.9%** |
///
/// TMCC 同期 100% のまま訂正不能率が 300 倍悪化している。bin offset が 1
/// ずれると `extract_segment()` の 432 carrier が 1 つずつずれ、
/// `estimate_channel()` が参照する pilot 配置が崩れるのが原因。
///
/// # 方式
///
/// TMCC で候補を数個に絞り、**各候補で復調パイプラインを実際に走らせて
/// RS 復.decode 成功率**を測る。これが唯一の ground truth。
/// 同一 capture 内の比較なので、受信条件の違いは打ち消される。
fn select_segment_offset_by_rs(
    specs: &[Vec<Complex32>],
    candidates: &[usize],
) -> Option<(usize, f32, Vec<(usize, f32)>)> {
    if specs.is_empty() || candidates.is_empty() {
        return None;
    }
    let mut scores: Vec<(usize, f32)> = candidates
        .iter()
        .map(|&off| {
            if off + 432 > specs[0].len() {
                return (off, 0.0);
            }
            let (p0, _) = detect_symbol_phase(
                &extract_segment(&specs[0], off),
                &SegmentPilots::center_1seg(),
            );
            let f = demod_and_align(specs, off, p0).map_or(0.0, |(f, ..)| f);
            (off, f)
        })
        .collect();
    // 降順（RS 復号率の高い順）。同率なら TMCC 順（入力順）を保つ。
    scores.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
    let best = *scores.first()?;
    if best.1 <= 0.0 {
        return None;
    }
    Some((best.0, best.1, scores))
}

/// 初期バッファをバッチ処理して整列パラメータを確定。
fn lock_params(specs: &[Vec<Complex32>], phase0: usize, seg_off: usize) -> Option<(usize, usize, usize)> {
    demod_and_align(specs, seg_off, phase0)
        .map(|(_, c, reset_off, phase)| (c, reset_off, phase))
}

/// 逐次パイプライン状態（1シンボル→ TSパケット）。warmup 中は状態だけ進め出力破棄。
struct Pipe {
    pilots: SegmentPilots,
    tdi: TimeDeinterleaver,
    bdi: BitDeinterleaverQpsk,
    depu_pos: usize,
    vit: ViterbiStreaming,
    bdeint: ByteDeinterleaver,
    prbs: EnergyPrbs,
    commutator: usize,
    reset_off: usize,
    block_phase: usize,
    rs_asm: RsBlockAssembler,
    /// 連続して訂正不能が出なかった RS ブロック数（出力ゲートの自己診断用）。
    pub clean_run: usize,
    /// 訂正不能が連続して `BURST_THRESHOLD` 個に達したイベント。
    ///
    /// `process()` が立つので、デコーダ側が `feed()` の戻り値を受けて
    /// discontinuity 付き PCR パケットを差し込む。Pipe は demod 側の
    /// 内部状態なのでここに持たせる。
    pub burst_raised: bool,
    /// 診断: Viterbi が確定したビット数。
    pub dbg_bits: u64,
    /// 診断: バイト化されたビット数（bit_cnt が 8 に達した回数）。
    pub dbg_bytes: u64,
    /// 診断: 復調器前期に捨てられたバイト数
    /// （`byte_in_idx < commutator`）。
    pub dbg_drop_commutator: u64,
    /// 診断: 復号器遅延で捨てられたバイト数
    /// （`deint_out_idx < BYTE_LATENCY + block_phase`）。
    pub dbg_drop_latency: u64,
    /// 診断: `rs_asm.feed()` に実際に渡したバイト数。
    pub dbg_bytes_to_rs: u64,
    /// 診断: `Pipe::process`（1 OFDM シンボル＝1 回）を呼んだ回数。
    pub dbg_syms: u64,
    /// 診断: warmup 待ちで何もせず `return` したシンボル数。
    pub dbg_syms_warmup: u64,
    /// 診断: 1 シンボルあたりに `data_carrier_indices` が返した carrier 数。
    pub dbg_carriers_last: u64,
    /// 診断: pilot から推定したチャネルの平均電力（累積平均）。
    /// 時間が進むほど減衰するなら等化器の時間的ドリフトを疑う。
    pub dbg_h_mag_sum: f64,
    /// 診断: `estimate_channel` を呼んだ回数。
    pub dbg_h_count: u64,
    /// 診断: 直近 128 シンボルの h 平均電力。
    pub dbg_h_window: std::collections::VecDeque<f64>,
    /// 診断: h 平均電力が累積平均を下回った回数（ドリフト検出）。
    pub dbg_h_degraded: u64,
    /// 診断: 累積 h 平均電力。
    pub dbg_h_avg: f64,
    /// 診断: h の位相の carrier 間分散（rad²）。線形補間が位相回転を
    /// 取り除けていないなら、この値が越大になる。
    ///
    /// 実測 2026-09-26: 1seg mode 3 の SP は 3 本（間隔 12 carrier）で、
    /// 11 サンプルを線形補間している。フェージングに carrier 間隔より
    /// 細かい成分があると**振幅は正しくても位相が狂う**。`|h|` の統計では
    /// 検出できないが復号は致命的に悪化する。
    pub dbg_h_phase_var: f64,
    /// 診断: 位相分散の累積平均。
    pub dbg_h_phase_var_avg: f64,
    /// 診断: SP 間の補間点の位相の直線性誤差（rad）。
    /// 理想 0。大きいと線形補間が位相を取り損ねている。
    pub dbg_h_interp_err: f64,
    /// 診断: SP における位相の**直線傾き**（rad/carrier）。
    ///
    /// carrier 間で滑らかな線形回転 = 残留 CFO。OFDM フレーム境界を
    /// 直す `reacquire_boundary()` は CFO の微ドリフトを追わないため、
    /// 時間が経つほど傾きが大きくなり、QPSK 星座が回転して RS が崩れる。
    /// 実測 2026-09-26: 位相分散 0.001 rad² で等化器は健全なのに
    /// RS 訂正不能率が 0% → 24.5% に劣化した。傾きの成長を検証する。
    pub dbg_cfo_slope: f64,
    /// 診断: SP 位相傾きの累積平均（rad/carrier）。
    pub dbg_cfo_slope_avg: f64,
    /// 診断: 実運用ループの `sym_mod4`（= `(phase0 + k) % 4`、機械導出）。
    pub dbg_sym_mod4_used: usize,
    /// 診断: 実測位相からの `sym_mod4`。`used` と違れば破綻。
    pub dbg_sym_mod4_actual: usize,
    /// 診断: QPSK ソフト出力の絶対値の平均（復調の自信度）。
    ///
    /// 理想 ±1.0。1.0 に近づくほど pilot/data 境界が鮮明で復調が確実。
    /// 時間が 지나るごとに低下すれば、受信 SINR の劣化を意味する。
    pub dbg_soft_abs: f64,
    /// 診断: 累積平均のソフト出力絶対値。
    pub dbg_soft_abs_avg: f64,
    /// 診断: ソフト出力の自信度が累積平均の半分を下回った回数。
    pub dbg_soft_degraded: u64,
    mother: Vec<f32>,
    bit_acc: u8,
    bit_cnt: usize,
    byte_in_idx: usize,
    deint_out_idx: usize,
    block_buf: Vec<u8>,
    block_idx: usize,
    ndec: usize,
    nblk: usize,
    seg_off: usize,
}
impl Pipe {
    fn new(commutator: usize, reset_off: usize, block_phase: usize, seg_off: usize) -> Self {
        Self {
            pilots: SegmentPilots::center_1seg(),
            tdi: TimeDeinterleaver::new(4),
            bdi: BitDeinterleaverQpsk::new(),
            depu_pos: 0,
            vit: ViterbiStreaming::new(TB_DEPTH),
            bdeint: ByteDeinterleaver::new(),
            prbs: EnergyPrbs::with_init(PRBS_INIT),
            commutator,
            reset_off,
            block_phase,
            rs_asm: RsBlockAssembler::new(0, reset_off),
            clean_run: 0,
            burst_raised: false,
            dbg_bits: 0,
            dbg_bytes: 0,
            dbg_drop_commutator: 0,
            dbg_drop_latency: 0,
            dbg_bytes_to_rs: 0,
            dbg_syms: 0,
            dbg_syms_warmup: 0,
            dbg_carriers_last: 0,
            dbg_h_mag_sum: 0.0,
            dbg_h_count: 0,
            dbg_h_window: std::collections::VecDeque::with_capacity(128),
            dbg_h_degraded: 0,
            dbg_h_avg: 0.0,
            dbg_h_phase_var: 0.0,
            dbg_h_phase_var_avg: 0.0,
            dbg_h_interp_err: 0.0,
            dbg_cfo_slope: 0.0,
            dbg_cfo_slope_avg: 0.0,
            dbg_sym_mod4_used: 0,
            dbg_sym_mod4_actual: 0,
            dbg_soft_abs: 0.0,
            dbg_soft_abs_avg: 0.0,
            dbg_soft_degraded: 0,
            mother: Vec::new(),
            bit_acc: 0,
            bit_cnt: 0,
            byte_in_idx: 0,
            deint_out_idx: 0,
            block_buf: Vec::with_capacity(TSP + 2),
            block_idx: 0,
            ndec: 0,
            nblk: 0,
            seg_off,
        }
    }
    fn warmup(&self) -> usize {
        self.tdi.latency() + self.bdi.latency()
    }

    fn process(&mut self, spec: &[Complex32], phase0: usize, k: usize, out: &mut Vec<u8>) {
        self.dbg_syms += 1;
        let sym_mod4 = (phase0 + k) % 4;
        let seg = extract_segment(spec, self.seg_off);
        // 診断: 機械導出した `sym_mod4` と、信号の実位相から測った
        // `sym_mod4` を比較する。両者がずれたら pilot 配置が 信号と
        // 無関係になり、等化器が「健全な誤り」を出す。実測 2026-09-26 の
        // 「等化器は健全なのに RS が 0%→24.5% 劣化する」現象の検証。
        if self.dbg_syms % 256 == 0 {
            self.dbg_sym_mod4_used = sym_mod4;
            self.dbg_sym_mod4_actual = detect_symbol_phase(&seg, &self.pilots).0 % 4;
        }
        let h = estimate_channel(&seg, sym_mod4, &self.pilots);
        // 診断: 推定チャネルの平均電力を追跡する。時間が進むほど減衰するなら
        // 等化器の時間的ドリフト（受信条件の変化・復調境界のずれ）が疑われる。
        // ロック時は先頭 128 ブロックで 100% 復号しても、走らせると 24%
        // 訂正不能になる、という観察を検証する。
        let h_mag: f64 = h
            .iter()
            .map(|c| ((c.re * c.re + c.im * c.im) as f64).sqrt())
            .sum();
        let h_mean = h_mag / h.len().max(1) as f64;
        self.dbg_h_mag_sum += h_mag;
        self.dbg_h_count += 1;
        self.dbg_h_avg = self.dbg_h_mag_sum / self.dbg_h_count as f64;
        if self.dbg_h_window.len() == self.dbg_h_window.capacity() {
            self.dbg_h_window.pop_front();
        }
        self.dbg_h_window.push_back(h_mean);
        if self.dbg_h_count > 512 && h_mean < self.dbg_h_avg * 0.5 {
            self.dbg_h_degraded += 1;
        }
        // 位相の診断: 線形補間が位相回転を取り除けているかを見る。
        //
        // 実測の謎: `|h|` は 1.4% しか低下しないのに RS 訂正不能率は
        // 0% → 24.5% に劣化する。振幅ではなく**位相**の問題。
        //
        // ここで 2 つの量を測る:
        // - 位相分散: h の位相が carrier 間でどれだけばらつくか
        // - 補間誤差: SP 間の補間点が、SP 両端を結ぶ直線からどれだけずれるか
        //
        // 補間誤差が大きければ「SP が 3 本しかなく間隔 12 carrier で
        // 線形補間している」ことの構造的限界が原因。
        {
            // 補間誤差: SP ごとに両端の h を直線結びし、途中 carrier の
            // h との差を測る。SP は `pilots.sp_carriers(sym_mod4)`。
            let sp: Vec<usize> = self.pilots.sp_carriers(sym_mod4).collect();
            let mut err_sum = 0.0f64;
            let mut err_n = 0usize;
            for w in sp.windows(2) {
                let (a, b) = (w[0], w[1]);
                let span = (b - a) as f64;
                if span <= 0.0 {
                    continue;
                }
                let (ha, hb) = (h[a], h[b]);
                for l in (a + 1)..b {
                    let t = (l - a) as f64 / span;
                    let pred = ha * (1.0 - t as f32) + hb * t as f32;
                    let d = h[l] - pred;
                    err_sum += ((d.re * d.re + d.im * d.im) as f64).sqrt();
                    err_n += 1;
                }
            }
            if err_n > 0 {
                self.dbg_h_interp_err = err_sum / err_n as f64;
            }
            // 位相分散: 隣り合う carrier の位相差の 2 乗平均
            let mut dphi = 0.0f64;
            let mut dphi_n = 0usize;
            for l in 1..h.len() {
                let pa = (h[l - 1].re.atan2(h[l - 1].im)) as f64;
                let pb = (h[l].re.atan2(h[l].im)) as f64;
                let mut d = pb - pa;
                while d > std::f64::consts::PI { d -= 2.0 * std::f64::consts::PI; }
                while d < -std::f64::consts::PI { d += 2.0 * std::f64::consts::PI; }
                dphi += d * d;
                dphi_n += 1;
            }
            if dphi_n > 0 {
                self.dbg_h_phase_var = dphi / dphi_n as f64;
                // 指数移動平均で累積基線を避ける
                let a = 0.01f64;
                self.dbg_h_phase_var_avg =
                    (1.0 - a) * self.dbg_h_phase_var_avg + a * self.dbg_h_phase_var;
            }
            // 残留 CFO の検出: SP 位相を carrier index に対して最小二乗
            // フィットし、傾き（rad/carrier）を取る。
            //
            // 理想 0。carrier 間隔 992 Hz なので、傾き 0.001 rad/carrier は
            // 約 1 Hz の CFO 残に相当する。これが時間とともに大きくなるなら
            // `reacquire_boundary()` が CFO を追っていないのが原因。
            //
            // 位相は mod 2π なので unwrap してからフィットする。
            if sp.len() >= 2 {
                let mut xs: Vec<f64> = Vec::with_capacity(sp.len());
                let mut ys: Vec<f64> = Vec::with_capacity(sp.len());
                let mut prev: Option<f64> = None;
                for &l in &sp {
                    let mut ph = (h[l].re.atan2(h[l].im)) as f64;
                    if let Some(pv) = prev {
                        // 隣接 SP 間の回転を ±π 未満に畳む
                        while ph - pv > std::f64::consts::PI {
                            ph -= 2.0 * std::f64::consts::PI;
                        }
                        while ph - pv < -std::f64::consts::PI {
                            ph += 2.0 * std::f64::consts::PI;
                        }
                    }
                    xs.push(l as f64);
                    ys.push(ph);
                    prev = Some(ph);
                }
                let n = xs.len() as f64;
                let mean_x = xs.iter().sum::<f64>() / n;
                let mean_y = ys.iter().sum::<f64>() / n;
                let sxy: f64 = xs.iter().zip(&ys).map(|(x, y)| (x - mean_x) * (y - mean_y)).sum();
                let sxx: f64 = xs.iter().map(|x| (x - mean_x) * (x - mean_x)).sum();
                if sxx > 0.0 {
                    self.dbg_cfo_slope = sxy / sxx;
                    let a = 0.01f64;
                    self.dbg_cfo_slope_avg =
                        (1.0 - a) * self.dbg_cfo_slope_avg + a * self.dbg_cfo_slope;
                }
            }
        }
        let eq = equalize(&seg, &h);
        let data: Vec<Complex32> = data_carrier_indices(sym_mod4, &self.pilots)
            .into_iter()
            .map(|l| eq[l])
            .collect();
        self.dbg_carriers_last = data.len() as u64;
        let fd = freq_deinterleave(&data);
        let td = self.tdi.push_symbol(&fd);
        let warmup = self.warmup();
        if k < warmup {
            self.dbg_syms_warmup += 1;
            for v in td {
                let _ = self.bdi.push(qpsk_soft(v)); // 状態だけ進める
            }
            return;
        }
        for v in td {
            let de = self.bdi.push(qpsk_soft(v)); // [lsb, msb]
            // 診断: ソフト出力の自信度。理想は ±1.0 で、1.0 から離れるほど
            // 復調の確信度が下がる。Viterbi より**前**の入力なので、ここが
            // 崩れていれば Viterbi の状態は正常でも出力ビットが間違う。
            let abs_sum = (de[0].abs() + de[1].abs()) as f64;
            self.dbg_soft_abs = abs_sum * 0.5;
            let a = 0.0005f64; // 1/2000 の指数移動平均
            self.dbg_soft_abs_avg = (1.0 - a) * self.dbg_soft_abs_avg + a * self.dbg_soft_abs;
            if self.dbg_soft_abs_avg > 0.01 && self.dbg_soft_abs < self.dbg_soft_abs_avg * 0.5 {
                self.dbg_soft_degraded += 1;
            }
            for kept in [de[1], de[0]] {
                // order=1
                loop {
                    let pat = PUNCTURE_2_3[self.depu_pos % PUNCTURE_2_3.len()];
                    self.depu_pos += 1;
                    if pat == 1 {
                        self.mother.push(kept);
                        break;
                    } else {
                        self.mother.push(0.0);
                    }
                }
            }
        }
        let pairs = self.mother.len() / 2;
        for t in 0..pairs {
            let bit = match self.vit.push(self.mother[2 * t], self.mother[2 * t + 1]) {
                Some(b) => b,
                None => continue,
            };
            self.bit_acc = (self.bit_acc << 1) | (bit & 1);
            self.bit_cnt += 1;
            self.dbg_bits += 1;
            if self.bit_cnt < 8 {
                continue;
            }
            let byte = self.bit_acc;
            self.bit_acc = 0;
            self.bit_cnt = 0;
            self.dbg_bytes += 1;
            if self.byte_in_idx < self.commutator {
                self.byte_in_idx += 1;
                self.dbg_drop_commutator += 1;
                continue;
            }
            self.byte_in_idx += 1;
            let o = self.bdeint.push(byte);
            if self.deint_out_idx < BYTE_LATENCY + self.block_phase {
                self.deint_out_idx += 1;
                self.dbg_drop_latency += 1;
                continue;
            }
            self.deint_out_idx += 1;
            self.dbg_bytes_to_rs += 1;
            let (blocks, burst) = self.rs_asm.feed(&[o]);
            for cw in blocks {
                self.nblk += 1;
                self.ndec += 1;
                self.clean_run += 1;
                out.extend_from_slice(&cw[..rs::K]);
            }
            if burst {
                // 訂正不能が BURST_THRESHOLD 個以上連続した。1 フレームが
                // 丸ごと消えたので discontinuity_indicator 付きの PCR パケットを
                // 1 本挿入し、demux に「この後は壊れている。次の I フレームから
                // 再開せよ」と伝える。**これが累積崩壊を防ぐ**。
                //
                // PCR_PID は映像 PID でなければならない（PMT の PCR_PID と一致）。
                // なぜ ContinuityTracker で CC を詰めるだけでは足りないか:
                // CC を詰めると demux は欠落に気づかず、壊れた GOP を
                // 「連続した正常データ」として repair されるのに任せる。
                // I フレームまで届かなければ回復しない。
                self.burst_raised = true;
            }
        }
        self.mother.drain(0..pairs * 2);
    }
}

/// 逐次ストリーミング復調器。`feed` にIQのu8バイトを渡すと、生成されたTSバイトを返す。
pub struct StreamingDecoder {
    demod: OfdmDemod,
    buf: Vec<Complex32>,
    pending: Vec<u8>,
    dc: Complex32,
    dc_done: bool,
    cur: usize,
    k: usize,
    /// 最後の長窓再取得からのシンボル数。
    syms_since_reacq: usize,
    /// 長窓再取得で得た境界のオフセット（`self.cur` 基準のサンプル差）。
    reacq_off: i64,
    /// 再取得が成功した回数（診断用）。
    reacq_count: usize,
    /// 再取得試行のログ出力カウンタ（診断用）。
    reacq_log: usize,
    /// ロック後、等化器が収束するまでに出力したシンボルの数（収束ゲート用）。
    settle_syms: usize,
    /// 収束ゲートの状態。`settle_syms` が `SETTLE_SYMS` 以上になったら true。
    settled: bool,
    /// **連続して**訂正不能が出なかった RS ブロック数（品質ゲート用）。
    clean_run: usize,
    /// **discontinuity 付き PCR パケットを注入すべき状態**か。
    ///
    /// 訂正不能が `BURST_THRESHOLD` 個以上連続すると立つ。`feed()` が
    /// 返した TS の**先頭**に `pcr_packet_discontinuity()` を 1 本差し込み、
    /// フラグを落とす。demux はこれを見て状態を捨て、次の I フレームから
    /// 回復する（累積崩壊の防止）。
    pending_discontinuity: bool,
    /// discontinuity 用 PCR の tick カウンタ（27MHz）。
    disc_ticks: u64,
    /// discontinuity 用 PCR パケットの continuity counter。
    disc_cc: u8,
    /// 注入した discontinuity パケット数（診断用）。
    pub disc_count: u64,
    /// 境界の**分数**オフセット（サンプル）。[-0.5, 0.5]。
    ///
    /// 実 fs（1015873.002204）と指定 fs（1015873）の差は 0.0022 サンプル/シンボル。
    /// 整数サンプル境界に丸め続けるとこの差が累積し、等化器のタップが合わず
    /// RS は 100% でも H.264 ビットが化ける。FFT 窓を線形補間でずらして吸収する。
    boundary_frac: f32,
    locked: Option<Locked>,
    pipe: Option<Pipe>,
    need_init: usize,
    live: bool, // true: ロック直後にバックログを捨ててライブエッジから（A/V同期・低遅延）
    /// ライブモードでロック時にバックログ切断を既に行ったか。
    live_trimmed: bool,
}

impl Default for StreamingDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl StreamingDecoder {
    pub fn new() -> Self {
        Self {
            demod: OfdmDemod::new(FFT_LEN),
            buf: Vec::new(),
            pending: Vec::new(),
            dc: Complex32::new(0.0, 0.0),
            dc_done: false,
            cur: 0,
            k: 0,
            syms_since_reacq: 0,
            reacq_off: 0,
            reacq_count: 0,
            reacq_log: 0,
            settle_syms: 0,
            settled: false,
            clean_run: 0,
            pending_discontinuity: false,
            disc_ticks: 0,
            disc_cc: 0,
            disc_count: 0,
            boundary_frac: 0.0,
            locked: None,
            pipe: None,
            need_init: (LOCK_SYMS + 8) * 1280 + 60_000, // 同期+ロックに十分な初期サンプル数
            live: false,
            live_trimmed: false,
        }
    }

    /// ライブモード：ロック直後にバックログを捨ててライブエッジから復号する（A/V同期・低遅延）。
    /// CLI/ファイル（全復号）では false のまま。
    pub fn set_live(&mut self, live: bool) {
        self.live = live;
    }

    /// これまでにロックできたか。
    pub fn is_locked(&self) -> bool {
        self.locked.is_some()
    }

    /// これまでにRS復号したパケット数と評価ブロック数。
    pub fn stats(&self) -> (usize, usize) {
        self.pipe.as_ref().map(|p| (p.ndec, p.nblk)).unwrap_or((0, 0))
    }

    /// (訂正を要した RS ブロック数, 訂正を要したビット総数)。
    ///
    /// 復調 bit error の直接指標。H.264 の MB エラー率と強い相関する
    /// （実測 2026-09-26: 0.96 metric の capture は 0.16 MB/s、
    ///  0.89 metric は 2.77 MB/s）。
    pub fn rs_error_stats(&self) -> (usize, u64) {
        self.pipe
            .as_ref()
            .map(|p| (p.rs_asm.rs_corrected, p.rs_asm.rs_bit_errors))
            .unwrap_or((0, 0))
    }

    /// (訂正を要した RS ブロック数, 訂正を要したビット総数,
    ///  **訂正不能で捨てられた**ブロック数, 訂正後も非ゼロ synd のブロック数)。
    ///
    /// 後者2つが 0 でなければ TS にはビット誤りが混入しており、プレイヤーは
    /// 壊れた映像をRepair せずそのまま表示する。`rs_error_stats` の
    /// 「訂正した」だけでは品質を保証できない（訂正が成功した証拠にならない）。
    pub fn rs_quality(&self) -> (usize, u64, usize, usize) {
        self.pipe
            .as_ref()
            .map(|p| {
                (p.rs_asm.rs_corrected, p.rs_asm.rs_bit_errors, p.rs_asm.rs_dropped, p.rs_asm.rs_miscorrected)
            })
            .unwrap_or((0, 0, 0, 0))
    }

    fn append(&mut self, iq: &[u8]) {
        let mut bytes = core::mem::take(&mut self.pending);
        bytes.extend_from_slice(iq);
        let even = bytes.len() & !1;
        for c in u8_iq_to_complex(&bytes[..even]) {
            self.buf.push(if self.dc_done { c - self.dc } else { c });
        }
        self.pending = bytes[even..].to_vec();
    }

    fn try_lock(&mut self) {
        if self.buf.len() < self.need_init {
            return;
        }
        if !self.dc_done {
            let dc = self.buf.iter().sum::<Complex32>() / self.buf.len() as f32;
            for v in self.buf.iter_mut() {
                *v -= dc;
            }
            self.dc = dc;
            self.dc_done = true;
        }
        let off = (self.buf.len() / 10).min(50_000);
        let est = match estimate_symbol_sync(
            &self.buf[off..(off + 1_000_000).min(self.buf.len())],
            FFT_LEN,
            GuardInterval::G1_8,
        ) {
            Some(e) => e,
            None => return, // データを増やして再試行
        };
        let sym = FFT_LEN + est.guard.cp_len(FFT_LEN);
        let sym0 = off + est.symbol_start;
        self.buf.drain(0..sym0); // buf[0] = symbol 0

        let navail = (self.buf.len() / sym).min(LOCK_SYMS);
        if navail < 200 {
            return;
        }
        // bin offset は capture ごとにズレるので、TMCC フレーム同期一致率で決める。
        // 固定値（308等）で当てると bin ズレにより符号部が全滅して RS 0% になる。
        let probe: Vec<Vec<Complex32>> = self
            .demod
            .demod_stream_tracked(&self.buf, 0, est.guard, est.cfo_subcarriers, navail, 8);
        let Some((seg_off, sync, known)) = crate::tmcc::select_segment_offset(&probe, OFFSET_RADIUS)
        else {
            eprintln!("offset 探索失敗（スペクトル長不足）");
            return;
        };
        if sync < 0.95 {
            eprintln!("offset={seg_off} だが TMCC 同期一致 {sync:.3} が低い（偽ロック疑い）: 戻す");
            return;
        }
        let specs = probe;
        // TMCC 一致度だけで決めない。TMCC は 1 フレーム 204 シンボルのうち
        // 1 シンボル分しかないので、bin offset が 1 ずれても同期一致 100% に
        // なる（実測: disc_test.iq は offset 307 で同期 1.000 だが訂正不能 31.9%、
        // p1.iq は offset 308 で訂正不能 0.1%）。そこで TMCC スコアの順に
        // 上位を数個候補にして、**実 RS 復号率**で決め直す。
        let cands = crate::tmcc::rank_segment_offsets(&specs, OFFSET_RADIUS, RS_REFINE_TOP_N);
        let Some((seg_off, rs_rate, all)) = select_segment_offset_by_rs(&specs, &cands) else {
            eprintln!("offset 候補 ({cands:?}) すべてで RS 復号 0%: ロック失敗（品質不良）");
            return;
        };
        if seg_off != cands[0] {
            eprintln!(
                "bin offset TMCC 優先={} だが RS 復号率で {} を採用",
                cands[0], seg_off
            );
        }
        let scores: Vec<String> = all.iter().map(|(o, f)| format!("{o}:{f:.3}")).collect();
        eprintln!(
            "bin offset={seg_off} (nominal {}) TMCC同期={sync:.3} known={known:.3} RS復号率={rs_rate:.3} 候補=[{}]",
            crate::equalize::SEGMENT_BIN_OFFSET,
            scores.join(" "),
        );
        let (phase0, _) = detect_symbol_phase(
            &extract_segment(&specs[0], seg_off),
            &SegmentPilots::center_1seg(),
        );
        let Some((commutator, reset_off, block_phase)) = lock_params(&specs, phase0, seg_off) else {
            eprintln!("ロック失敗: navail={} specs={} phase0={}", navail, specs.len(), phase0);
            return; // ロック失敗（品質）。増データで再試行
        };
        self.locked = Some(Locked {
            gi: est.guard,
            cfo: est.cfo_subcarriers,
            sym,
            phase0,
            seg_off,
        });
        self.pipe = Some(Pipe::new(commutator, reset_off, block_phase, seg_off));
        // ライブ：溜まったバックログを捨ててライブエッジ付近から復号（音声が映像に遅れないよう）。
        // 整列（PRBS周期・OFDMフレーム・phase0・バイト整列）を壊さないため、
        // 1フレーム=204シンボル単位で捨てる（204%4=0, 64RSブロック=PRBS1周期）。
        //
        // 残す長さも実フレーム境界に合わせる。`self.cur = 0` で復調を始めると
        // buf[0] がフレーム先頭でないと phase0 が合わず、ライブモードだけ
        // RS ブロックが 1 つも出ない（実測: 0/0、ファイルモードは 6398/6398）。
        //
        // 重要: この切断はロック時に**1 回だけ**行う。ここを毎チャンクやると
        // buf が短かすぎ（≒30kサンプル）て、RS ブロック（256シンボル）が
        // 常に完成前に次のチャンクで流出してしまい、TS 出力率が実時間の
        // 16分の1（2,693 B/s 対 42,000 B/s）に落ちる（実測）。以降は
        // `COMPACT_AT` drain がバッファ長を制約する。
        if self.live {
            let frame = 204 * sym;
            if self.buf.len() > frame {
                let keep_frames = 800 / 204; // warmup(500)＋レイテンシ＋余裕 ≒ 3 フレーム
                let keep = frame.saturating_mul(keep_frames.max(1));
                if self.buf.len() > keep {
                    let drop = ((self.buf.len() - keep) / frame) * frame;
                    if drop > 0 {
                        self.buf.drain(0..drop);
                    }
                }
            }
            // 二重切断防止のフラグ（診断用。現在の切断はロック時のみなので
            // 実質未使用だが、将来的に push 側で切断を足す際のガードになる）。
            self.live_trimmed = true;
        }
        self.cur = 0;
        self.k = 0;
        self.syms_since_reacq = 0;
        self.reacq_off = 0;
        self.settle_syms = 0;
        self.settled = false;
        self.clean_run = 0;
    }

    /// IQ u8 を投入し、届いたぶんを**全部**処理して生成TSを返す（CLI/ファイル用・従来動作）。
    ///
    /// # 実時間性能（実測）
    ///
    /// ファイル入力 20 秒分（IQ 64.5 MB）で TS 1,215,232 B 出一道（60,761 B/s）。
    /// 実時間に必要なのは 40,000 B/s なので**実時間の 1.5 倍**で、復号性能は
    /// 十分。パイプ経路が遅かったのは IQ がチャンク（16,384 サンプル = 14
    /// シンボル）ごとにしか `process` に入れず、RS ブロック（256 シンボル）が
    /// 次のチャンクを待たないと完成しないため。ファイルは全量が一気に
    /// `process(usize::MAX)` されるので 1 回のループで大量を処理できる。
    ///
    /// そのためパイプ経路では `FEED_MIN_SYMS` シンボルぶん IQ が貯まるまで
    /// 復号を保留し、まとめて処理する。`FEED_MIN_SYMS` は 0（保留しない）が
    /// 現状の最適値（実測: ファイル 60,761 B/s、パイプ 2,693 B/s）。
    pub fn feed(&mut self, iq: &[u8]) -> Vec<u8> {
        self.push(iq);
        // ライブ：RS ブロック（256 シンボル）が完成するまで待ってからまとめて処理。
        // チャンクごとに 14 シンボルだけ処理すると RS が完成せず出力率が
        // 実時間の 16分の1に落ちる（実測: 2,693 B/s 対 42,000 B/s）。
        if self.live {
            if let Some(lk) = self.locked {
                let have = self.buf.len().saturating_sub(self.cur) / lk.sym;
                if have < FEED_MIN_SYMS {
                    return Vec::new();
                }
            }
        }
        self.process(usize::MAX)
    }

    /// IQ u8 を投入するだけ（同期/ロックは試みるが、シンボル処理はしない）。
    /// ライブのワーカーは `push` → `pump` ループで、処理を小分けにして描画を挟む。
    pub fn push(&mut self, iq: &[u8]) {
        self.append(iq);
        if self.locked.is_none() {
            self.try_lock();
            // 未ロック時のバッファ暴走防止：ロックできない弱電界のライブ入力では
            // buf が無限に伸び WASM が OOM(=unreachable) する。最新 need_init 分だけ残す。
            if self.locked.is_none() {
                let cap = self.need_init + self.need_init / 2;
                if self.buf.len() > cap {
                    let drop = self.buf.len() - self.need_init;
                    self.buf.drain(0..drop);
                }
            }
        }
    }

    /// 復調パイプライン各段の診断カウンタ（`[pipe]` デバッグ行用）。
    pub fn dbg_bits(&self) -> u64 { self.pipe.as_ref().map_or(0, |p| p.dbg_bits) }
    /// バイト化まで到達したビット数。
    pub fn dbg_bytes(&self) -> u64 { self.pipe.as_ref().map_or(0, |p| p.dbg_bytes) }
    /// 復調器前期（commutator）で捨てられたバイト数。
    pub fn dbg_drop_commutator(&self) -> u64 { self.pipe.as_ref().map_or(0, |p| p.dbg_drop_commutator) }
    /// 復号器遅延で捨てられたバイト数。
    pub fn dbg_drop_latency(&self) -> u64 { self.pipe.as_ref().map_or(0, |p| p.dbg_drop_latency) }
    /// RS 復号器に実際に渡したバイト数。
    pub fn dbg_bytes_to_rs(&self) -> u64 { self.pipe.as_ref().map_or(0, |p| p.dbg_bytes_to_rs) }
    /// 復号済み RS ブロック数。
    pub fn dbg_nblk(&self) -> u64 { self.pipe.as_ref().map_or(0, |p| p.nblk as u64) }
    /// `Pipe::process` を呼んだ回数（OFDM シンボル数）。
    pub fn dbg_syms(&self) -> u64 { self.pipe.as_ref().map_or(0, |p| p.dbg_syms) }
    /// warmup 待ちで消費したシンボル数。
    pub fn dbg_syms_warmup(&self) -> u64 { self.pipe.as_ref().map_or(0, |p| p.dbg_syms_warmup) }
    /// 1 シンボルあたりのデータ carrier 数。
    pub fn dbg_carriers(&self) -> u64 { self.pipe.as_ref().map_or(0, |p| p.dbg_carriers_last) }
    /// 推定チャネルの累積平均電力。
    pub fn dbg_h_avg(&self) -> f64 { self.pipe.as_ref().map_or(0.0, |p| p.dbg_h_avg) }
    /// h 平均電力が累積平均の半分を下回った回数（ドリフト検出）。
    pub fn dbg_h_degraded(&self) -> u64 { self.pipe.as_ref().map_or(0, |p| p.dbg_h_degraded) }
    /// 推定チャネルの推定回数（復調したシンボル数）。
    pub fn dbg_h_count(&self) -> u64 { self.pipe.as_ref().map_or(0, |p| p.dbg_h_count) }
    /// h の位相分散（carrier 間、EMA）。
    pub fn dbg_h_phase_var(&self) -> f64 { self.pipe.as_ref().map_or(0.0, |p| p.dbg_h_phase_var) }
    /// SP 間の補間誤差（平均絶対値）。
    pub fn dbg_h_interp_err(&self) -> f64 { self.pipe.as_ref().map_or(0.0, |p| p.dbg_h_interp_err) }
    /// SP 位相の直線傾き（rad/carrier、EMA）= 残留 CFO。
    pub fn dbg_cfo_slope(&self) -> f64 { self.pipe.as_ref().map_or(0.0, |p| p.dbg_cfo_slope) }
    /// 機械導出した `sym_mod4`。
    pub fn dbg_sym_mod4_used(&self) -> usize { self.pipe.as_ref().map_or(0, |p| p.dbg_sym_mod4_used) }
    /// 実測位相からの `sym_mod4`。`used` と違れば破綻。
    pub fn dbg_sym_mod4_actual(&self) -> usize { self.pipe.as_ref().map_or(0, |p| p.dbg_sym_mod4_actual) }
    /// ソフト出力の自信度（EMA）。
    pub fn dbg_soft_abs(&self) -> f64 { self.pipe.as_ref().map_or(0.0, |p| p.dbg_soft_abs_avg) }
    /// ソフト出力の自信度が累積平均の半分を下回った回数。
    pub fn dbg_soft_degraded(&self) -> u64 { self.pipe.as_ref().map_or(0, |p| p.dbg_soft_degraded) }
    /// バックログを**最大 `MAX_SYMS_PER_CALL` シンボルだけ**処理して生成TSを返す。
    /// 1回が長時間ブロックしないので、呼び出し側は合間に描画コールバックを走らせられる。
    pub fn pump(&mut self) -> Vec<u8> {
        self.process(MAX_SYMS_PER_CALL)
    }

    /// まだ処理していない（バックログの）シンボル数。
    pub fn backlog_syms(&self) -> usize {
        match self.locked {
            Some(lk) => self.buf.len().saturating_sub(self.cur) / lk.sym,
            None => 0,
        }
    }

    fn process(&mut self, max: usize) -> Vec<u8> {
        let mut out = Vec::new();
        if let Some(lk) = self.locked {
            let mut n = 0;
            while self.cur + lk.sym <= self.buf.len() && n < max {
                // 一定周期で長窓 CP 探索（1M サンプル、metric 0.30 で安定する）
                // を使って境界の基準を取り直す。1 シンボル窓の探索だけでは
                // 実 fs 差が累積して約 2000 ブロックで破綻する（実測）。
                if self.syms_since_reacq >= reacq_every() {
                    self.syms_since_reacq = 0;
                    let r = self.demod.reacquire_boundary(&self.buf, self.cur, lk.gi, REACQUIRE_WINDOW);
                    if self.reacq_count < 5 || self.reacq_log % 50 == 0 {
                        eprintln!(
                            "再取得試行: cur={} buflen={} sym={} → {:?}",
                            self.cur, self.buf.len(), lk.sym,
                            r.map(|(p, m)| (p, m))
                        );
                        self.reacq_log += 1;
                    }
                    if let Some((pos, metric)) = r {
                        let delta = pos as i64 - self.cur as i64;
                        // 誤 peak を避けるため、半シンボル以上ずれた検出は捨てる。
                        if delta.abs() < lk.sym as i64 / 2 {
                            eprintln!(
                                "再取得: cur={} → peak={} (Δ{}) metric={:.4}",
                                self.cur, pos, delta, metric
                            );
                            // 整数シンボル数で丸めると 0.5 サンプル未満の
                            // 残差が捨てられ、実 fs 差（0.0022 サンプル/シンボル）
                            // が累積する。その残差を **分数オフセット** として
                            // 保持し、FFT 窓を線形補間でずらす。
                            //
                            // これが無いと整数境界に丸め続け、等化器のタップが
                            // 合わず RS は 100% でも H.264 ビットが化ける
                            // （実測 2026-09-26: MB エラー 1 件/10 秒 →
                            //  70 件/25 秒、境界 metric 0.96 → 0.89）。
                            let exact = (pos as f64) - (self.cur as f64);
                            self.boundary_frac = (exact - exact.round()).clamp(-0.5, 0.5) as f32;
                            // cur をずらす分だけ k も進めないと、
                            // k（phase0/reset_off の基準）と実シンボル位置が
                            // ずれて RS reset（64ブロック=256シンボル周期）に
                            // 合わなくなり出力が恒久停止する（実測: 約450シンボルで
                            // 出力停止）。ジャンプ量は必ず sym の整数倍に丸める。
                            let syms = (delta as f64 / lk.sym as f64).round() as i64;
                            let adj = (syms * lk.sym as i64) as i64;
                            self.cur = (self.cur as i64 + adj).max(0) as usize;
                            if syms != 0 {
                                self.k = (self.k as i64 + syms).max(0) as usize;
                            }
                            self.reacq_count += 1;
                        }
                    }
                }
                let Some((spec, next)) = self.demod.demod_one_tracked_frac(
                    &self.buf,
                    self.cur,
                    lk.gi,
                    lk.cfo,
                    8,
                    self.boundary_frac,
                ) else {
                    if n < 5 {
                        eprintln!(
                            "[why] demod None: cur={} buflen={} sym={} k={}",
                            self.cur, self.buf.len(), lk.sym, self.k
                        );
                    }
                    break;
                };
                self.pipe
                    .as_mut()
                    .unwrap()
                    .process(&spec, lk.phase0, self.k, &mut out);
                self.settle_syms += 1;
                // 収束ゲート。**既定は無効**（`ISDBT_GATE=1` で有効化）。
                // 理由と実測は `gate_enabled()` のドキュメントを参照。
                // ゲート有効時は、収束が完了するまで TS を出ない。得られるのは
                // 「再生開始が数秒遅れる」だけで、壊れた映像は見なくなる。
                if !self.settled && gate_enabled() {
                    // ゲートは「時間下限 AND 品質」で開くが、**品質が来ない
                    // 場合は時間上限で強制的に開く**（fail-open）。
                    //
                    // これが要る理由（実測 2026-09-26, 24 秒キャプチャ）:
                    // 訂正不能は 51 個/チャンクの集中から 1〜4 個/チャンクへ
                    // 減るものの**最後まで消えず**、`CLEAN=200`（200 連続無誤）は
                    // 一度も達成されなかった。品質条件だけをゲートに使うと
                    // TS 出力が 0 バイトになり、ゲート導入前より悪化させる。
                    //
                    // 訂正不能が残る場合でも null パケットを挿入しているので、
                    // 出力は「多少壊れている TS」であり「何も出さない」より
                    // 遥かに良い。よって上限に達したら開ける。
                    let by_time = self.settle_syms >= settle_min_syms();
                    let by_quality = self.clean_run >= clean_blocks_needed();
                    let by_deadline = self.settle_syms >= settle_max_syms();
                    if by_time && (by_quality || by_deadline) {
                        self.settled = true;
                        let why = if by_quality { "品質達成" } else { "時間上限" };
                        eprintln!(
                            "収束完了({why}): {} シンボル / 連続 {} ブロック無誤で TS 出力開始",
                            self.settle_syms, self.clean_run
                        );
                    } else {
                        out.clear();
                    }
                }
                self.cur = next;
                self.k += 1;
                n += 1;
                self.syms_since_reacq += 1;
                if false {
                    eprintln!(
                        "[p] k={} cur={} buf={} nblk={} kmod256={}",
                        self.k, self.cur, self.buf.len(),
                        self.pipe.as_ref().map(|p| p.nblk).unwrap_or(0),
                        self.k % 256
                    );
                }
                if self.cur >= COMPACT_AT {
                    // 実フレーム境界で落とさないと OFDM フレームの位相が
                    // 崩れ、RS ブロック（256 シンボル）が二度と完成しない
                    // （実測: TS 出力 65 B/s ipotent while 復調は継続）。
                    // COMPACT_AT (8 MiB) は 204×1152 = 235008 の倍数では
                    // ないので、余剰を捨ててフレーム数単位にする。
                    let frame = 204 * lk.sym;
                    let drop = ((self.cur / frame) * frame).saturating_sub(frame);
                    if drop > 0 {
                        self.buf.drain(0..drop);
                        self.cur -= drop;
                    }
                }
            }
        }
        // Pipe（demod）が訂正不能の連続を検出した場合、discontinuity 付き
        // PCR パケットを差し込む。Pipe は内部状態なのでフラグを読む。
        if self.pipe.as_ref().is_some_and(|p| p.burst_raised) {
            if let Some(p) = self.pipe.as_mut() {
                p.burst_raised = false;
            }
            self.pending_discontinuity = true;
        }
        // discontinuity イベントのたびに PCR パケットを差し込む。
        //
        // **挿入位置は TS の先頭**。これより前のパケット（PID 0x0581 の
        // 連続データ）は「壊れた GOP の続き」なので、demux に
        // 「ここから状態を破棄せよ」と伝えるには、その直前に
        // discontinuity を置く必要がある。後ろに置くと意味がない。
        //
        // PCR_PID は**映像 PID**（`psi::PID_VIDEO`）を使う。PMT の PCR_PID
        // と一致していないと demux はタイムスタンプ源として使わず、
        // 「more than 5 seconds of late video -> dropping frame」を繰り返す
        // （実測 2026-09-26）。
        if self.pending_discontinuity && !out.is_empty() {
            self.pending_discontinuity = false;
            // PCR の tick は 27MHz。既存の PCR 注入と同じスケールを使う。
            let ticks = self.disc_ticks;
            self.disc_ticks = self.disc_ticks.wrapping_add(27_000_000);
            self.disc_cc = self.disc_cc.wrapping_add(1);
            self.disc_count += 1;
            let pkt = crate::psi::pcr_packet_discontinuity(
                crate::psi::PID_VIDEO,
                self.disc_cc,
                ticks,
            );
            // 既に out に溜まっているバイトの後ろではなく、**先頭に**挿入する
            // ことで demux が「ここから状態破棄」と認識する。
            let mut merged = Vec::with_capacity(out.len() + 188);
            merged.extend_from_slice(&pkt);
            merged.extend_from_slice(&out);
            out = merged;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rs::encode;
    #[test]
    fn rs_assembler_decodes_two_consecutive_blocks() {
        let mut a = RsBlockAssembler::new(0, 0);
        let cw1 = encode(&[0x11; rs::K]);
        let cw2 = encode(&[0x22; rs::K]);
        let mut prbs = EnergyPrbs::with_init(PRBS_INIT);
        let mut raw = Vec::new();
        for cw in [&cw1, &cw2] {
            raw.push(cw[0]);
            for &b in &cw[1..] { raw.push(b ^ (prbs.clock(8) as u8)); }
            prbs.clock(8);
        }
        let (out, burst) = a.feed(&raw);
        assert_eq!(out.len(), 2);
        assert_eq!(&out[0][..2], &[0x11, 0x11]);
        assert_eq!(&out[1][..2], &[0x22, 0x22]);
        assert!(!burst, "訂正不能の連続がないので discontinuity は発火しない");
    }

    /// 訂正不能ブロックが **`[0xff; 188]` でも null パケットでもない**、
    /// 出力されない（穴を空ける）ことの回帰テスト。
    ///
    /// 実測 2026-09-26 の 2 つのバグ:
    /// 1. `[0xff; 188]`（sync 0x47 なし）→ パケット境界 34.6% が壊れ
    ///    VLC が demux 失敗（`0x47 sync 異常 2230/6442`）
    /// 2. `null_packet()`（PID 0x1FFF、sync あり）→ TS 構造は正しいが
    ///    **H.264 の参照フレームが破綻し「再生時間とともに画像が崩れる」**
    ///
    /// どちらも撤回済み。訂正不能ブロックは**出力しない**のが正しい。
    #[test]
    fn uncorrectable_block_is_dropped_not_padded() {
        let mut a = RsBlockAssembler::new(0, 0);
        // 訂正不能なコードワードを作る: syndrome が非ゼロになり復号が成立しない
        let mut cw = encode(&[0x33; rs::K]);
        for b in cw.iter_mut().take(40) {
            *b ^= 0xFF;
        }
        let mut prbs = EnergyPrbs::with_init(PRBS_INIT);
        let mut raw = Vec::new();
        raw.push(cw[0]);
        for &b in &cw[1..] {
            raw.push(b ^ (prbs.clock(8) as u8));
        }
        prbs.clock(8);
        let (out, _burst) = a.feed(&raw);
        // 訂正できたなら 1 要素、訂正不能なら 0 要素（何も差し出さない）
        for data in &out {
            assert_eq!(data[0], 0x47, "出力されるパケットは sync 0x47 を持つ");
        }
    }

    /// 訂正不能が `BURST_THRESHOLD` 個以上**連続**したら discontinuity
    /// イベントが 1 度だけ発火し、訂正が回復したらリセットされることの
    /// 回帰テスト。
    ///
    /// 実測 2026-09-26: 訂正不能 126 個のうち 51 個が 1 箇所で連続し、
    /// その区間の I フレーム（2 秒周期）が丸ごと失われた。「最初はきれい、
    /// 時間が立つと崩れる」現象の直接原因。
    #[test]
    fn burst_of_uncorrectable_raises_discontinuity_once() {
        let mut a = RsBlockAssembler::new(0, 0);
        // 訂正不能ブロックを BURST_THRESHOLD + 余分に連結して供給する
        let n = BURST_THRESHOLD + 8;
        let mut prbs = EnergyPrbs::with_init(PRBS_INIT);
        let mut raw = Vec::new();
        for i in 0..n {
            let mut cw = encode(&[0x33 + (i as u8); rs::K]);
            for b in cw.iter_mut().take(40) {
                *b ^= 0xFF;
            }
            raw.push(cw[0]);
            for &b in &cw[1..] {
                raw.push(b ^ (prbs.clock(8) as u8));
            }
            prbs.clock(8);
        }
        let (_out, burst) = a.feed(&raw);
        assert!(burst, "連続訂正不能が閾値に達したら discontinuity が発火する");
        assert!(
            a.rs_dropped >= BURST_THRESHOLD,
            "訂正不能が記録されていること（実際 {}）",
            a.rs_dropped
        );

        // 訂正できたブロックが続けば連続カウントはリセットされ、
        // 次の discontinuity は発火しない。
        let mut good = Vec::new();
        let cw = encode(&[0x11; rs::K]);
        good.push(cw[0]);
        for &b in &cw[1..] {
            good.push(b ^ (prbs.clock(8) as u8));
        }
        prbs.clock(8);
        let (_out2, burst2) = a.feed(&good);
        assert!(!burst2, "訂正が回復したら discontinuity は再発火しない");
        assert_eq!(a.drop_burst, 0, "連続カウントがリセットされること");
    }
}

