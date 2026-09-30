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

/// エネルギー逆拡散 PRBS のリセット周期（RS ブロック数）。
///
/// 実測 2026-09-26: 固定値 64 では「先頭 2 フレームは完璧、後半で訂正不能が
/// 累積する」症状が出た。原因は、ロック探索 (`lock_params`) が
/// **先頭 128 ブロック（= 周期 64 ならちょうど 2 周期）**だけで
/// `reset_off` を選ぶため、周期 32 でも 64 でも 78 でも探索窓内では
/// 区別がつかず、最後の 1 つを選んでしまう。結果、途中で周期がずれ、
/// 以降すべての PRBS 解除が_bit ずれ続ける。
///
/// `ISDBT_RPERIOD` で上書き可能にして、周期そのものを走査できるようにした。
///
/// **訂正（2026-09-29）**: 旧コメントは「実装の kept bit 数が
/// 204×384×2×**3/4** = 117,504 で、64 ブロック分（96,256 bit）と一致しない」
/// として周期に疑義enorause ていたが、これは **レート計算の誤り**。
/// 1seg は**レート 2/3**（`PUNCTURE_2_3`）であり 3/4 ではない。
/// レート 2/3 で計算するとちょうど 64.0 TSP/フレームになる:
///
/// ```text
/// rate 1/2 -> 48.0 TSP    rate 2/3 -> 64.0 TSP  ★1seg
/// rate 3/4 -> 72.0 TSP    rate 7/8 -> 84.0 TSP
/// ```
///
/// 実測（atk3.iq、`ISDBT_RPERIOD` 走査）も一致:
///
/// | 周期 | drop/総blk | symerr/blk |
/// |---|---|---|
/// | **64** | **11890** | **7.24** |
/// | 72 | 14710 | 7.91 |
/// | 78 | 14968 | 7.97 |
///
/// 周期 **64 が正解**であり、3/4 ベースの「不一致」は存在しない。
/// この誤計算が「周期がズレている」という誤った探索方向を生んでいた。
fn reset_period() -> usize {
    let v = match std::env::var("ISDBT_RPERIOD") {
        Ok(v) => v.parse().unwrap_or(RESET_PERIOD),
        Err(_) => RESET_PERIOD,
    };
    if v == 0 { usize::MAX } else { v.max(1) }
}

const RESET_PERIOD: usize = 64; // 1 OFDMフレーム = 64 RSブロック
/// SP ベースのファイン・タイミング追跡を有効にするか。
///
/// 既定は**無効**。`estimate_symbol_sync` の整数サンプル位置と
/// 800 シンボルごとの再取得だけで十分な精度があると考えていたが、
/// 実測では分割 1.34% / 連続 24〜58% の差が残るため、有効化して
/// 効果を測る。`ISDBT_SPTRACK=1` で有効。
fn sp_track_enabled() -> bool {
    std::env::var("ISDBT_SPTRACK").map(|v| v != "0").unwrap_or(false)
}

/// SP タイミング推定の EMA 係数。小さいほど平滑で、大きいほど追従が速い。
fn sp_alpha() -> f32 {
    match std::env::var("ISDBT_SPALPHA") {
        Ok(v) => v.parse().unwrap_or(0.02),
        Err(_) => 0.02,
    }
}

/// Pipe を再生成する周期（RS ブロック数）。0 で無効。
///
/// **実測 2026-09-26**: 連続処理の状態が時間とともに汚染される。
/// 同じ IQ を分割して singly デコードすると 0.3〜1.6%、連結すると 24%:
/// ```text
/// h1 単独（40MB）      総blk=4086  drop=65    1.59%
/// h2 単独（40MB）      総blk=4138  drop=13    0.31%
/// h1+h2 連結（80MB）   総blk=9565  drop=2306  24.11%
/// h1 2回連結（80MB）   総blk=9565  drop=5543  57.95%
/// ```
/// h1 を 2 回並べただけで 2 AMA.Group目が全滅するので、IQ 側の問題では
/// ない。demod → Viterbi → byte 整列 → RS のいずれかの内部状態が
/// 長時間走ると破綻する。パイプラインの落ち込みはない
/// （`to_rs=1,951,413` に対し `drop_lat=2,303` は一定、`drop_comm=0`）ので、
/// 破綻は RS 到達前の内部状態に起因する。
///
/// 分割すると 0.3% に戻る事実は「区切り直せば直る」ことを意味するので、
/// 一定周期で Pipe を再生成して状態を初期化する。位置は 1 OFDM フレーム
/// （204 シンボル = 64 RS ブロック）の倍数にすることで、フレーム境界と
/// PRBS 周期を壊さない。
///
/// `ISDBT_RESETBLK` で上書き可。0 で無効化（長期連続受信時の比較用）。
fn reset_blk() -> usize {
    let v = match std::env::var("ISDBT_RESETBLK") {
        Ok(v) => v.parse().unwrap_or(RESET_BLK),
        Err(_) => RESET_BLK,
    };
    if v == 0 { usize::MAX } else { v }
}

/// Pipe を再生成する周期（OFDM フレーム数）。RS ブロック周期を 64 で割る。
/// `ISDBT_DEPUALL=1` で、全 RS ブロックの depuncture 位相を記録する。
///
/// 既定は無効（64K 要素の `Vec` を持つため）。
/// `ISDBT_PRSBALL=1` で PRBS リセット位置を全記録する。
fn dbg_prbs_all_enabled() -> bool {
    std::env::var("ISDBT_PRSBALL").map(|v| v != "0").unwrap_or(false)
}

fn dbg_depu_all_enabled() -> bool {
    std::env::var("ISDBT_DEPUALL").map(|v| v != "0").unwrap_or(false)
}

fn reset_frames() -> usize {
    let b = reset_blk();
    // `reset_blk() == 0`（既定無効）のときは `usize::MAX` ではなく
    // 「リセットしない」=  到達しない巨大値にして、外側の `>=` が永続 false になるようにする。
    if b == 0 || b == usize::MAX { usize::MAX } else { (b / 64).max(1) }
}

const RESET_BLK: usize = 0;

/// 1 ISDB-T フレーム = 204 OFDM シンボル（TMCC が全帯域共通で 1 フレームを通知する）。
const TMCC_SYMS_PER_FRAME: usize = 204;

/// 段別ダンプのリング長。1 RS ブロック分に十分大きい。
const DUMP_RING: usize = 8192;

/// 1 RS ブロック（204 バイト）に必要な soft 値の長さ。
///
/// 204 バイト = 1632 情報ビット、レート 2/3 なので 1728 トレリスステップ、
/// 1 ステップ 2 値（X, Y）で **3456 値**。余裕をみて少し多めに取る。
const RS_BLOCK_SOFT: usize = 3456;

/// 保持する**トレリスステップ数**。
///
/// soft は 1 ステップあたり 2 値（X, Y）、vit は 1 ビット。
/// リング長を「soft 値数」と「vit ビット数」で別々にすると
/// **同じ時間窓にならない**（実測: soft 60000 値 = 30000 ステップ
/// に対し vit 24000 ビットしかなく、比較対象がずれていた）。
/// よってステップ数で единиц を決めて両者を揃える。
const RING_STEPS: usize = 30_000;

/// soft リングの保持長。vit の 3 倍（レート 2/3 + X/Y 対）を見込んで
/// VIT_RING_BITS より大きめに取る。
const VIT_RING_SOFT: usize = RING_STEPS * 2; // 0 = 無効（Pipe::resync は Viterbi を壊す。上記コメント参照）
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

/// `buf` を切り詰める境界長（サンプル）。`ISDBT_COMPACT` で上書き可。
///
/// **実測 2026-09-26 の主因候補**: `drain(0..drop)` はサンプル単位の丸め
/// ottleので、切り詰めのたびに FFT 境界が 0〜1 サンプルずれる。ずれた境界は
/// `k` の補正では修復できず、pilot 位置・位相推定が以降の全シンボルで
/// ずれ、RS 訂正不能が累積する。実測でこれと整合する:
/// - h1 単独（40MB, compact 5 回）  drop 1.59%
/// - h2 単独（40MB, compact 5 回）  drop 0.31%
/// - 連結    （80MB, compact 10 回） drop 24.11%
/// つまり compact 回数にほぼ比例して崩れる。
///
/// `ISDBT_COMPACT=0` で無効化できる（メモリは `[buf.len()]` に比例して増える
/// が、80MB IQ なら 40MB 程度）。
fn compact_at() -> usize {
    let v = match std::env::var("ISDBT_COMPACT") {
        Ok(v) => v.parse().unwrap_or(COMPACT_AT),
        Err(_) => COMPACT_AT,
    };
    if v == 0 { usize::MAX } else { v }
}
/// 1回の feed/pump で処理する最大シンボル数（≈50msぶん）。呼び出し側が描画を挟めるように分割。
const MAX_SYMS_PER_CALL: usize = 64;

/// 1 回の feed/pump で処理する最大シンボル数。`ISDBT_MAXSYMS` で上書き可。
///
/// 実測 2026-09-26: 同じ 80MB の IQ を
/// - 4 分割して singly デコード → drop 1.34%
/// - 1 本で連続処理        → drop 24.11%
///
/// IQ データは正常なので、悪化は「連続処理の長さ」依存。`feed()` の
/// チャンク分割（`MAX_SYMS_PER_CALL`）ごとに"Some 状態が積み上がるなら、
/// 分割粒度を陋くすると惡化するはず。0（`usize::MAX`）で無効化して
/// 粒度の影響を切り分ける。
fn max_syms_per_call() -> usize {
    let v = match std::env::var("ISDBT_MAXSYMS") {
        Ok(v) => v.parse().unwrap_or(MAX_SYMS_PER_CALL),
        Err(_) => MAX_SYMS_PER_CALL,
    };
    if v == 0 { usize::MAX } else { v }
}

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
/// 実 fs（1015873.002204）と指定 fs（1015873）の差 ≈2.5e-6 サンプル/シンボルが
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
/// 2.5e-6 × 87 ≈ 0.2 サンプルで、探索半径 1 サンプル内に十分収まる。
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
/// 強制再ロック周期（処理シンボル数）。0 で無効。
///
/// 実測 2026-09-26: 連続 386MB を 1 本で処理すると drop 率が 0.2% → 90.4% と
/// 単調悪化するが、同じ IQ の後半 200MB を**別プロセス**で処理すると 0.10%。
/// つまり IQ は正常で、悪化は連続処理の内部状態に依存する。
///
/// Pipe::resync()（部分的リセット）では 90% → 99.8% に**悪化**する（warm-up と
/// RS assembler 状態を壊すため）。そこで `lk` を丸ごと作り直す、**完全な
/// 再ロック**を試す。これが 0.2% まで戻れば「内部状態の累積」で確定。
fn relock_every() -> usize {
    match std::env::var("ISDBT_RELOCK") {
        Ok(v) => v.parse().unwrap_or(0),
        Err(_) => 0,
    }
}

/// 分数キャリーを有効にするか（`ISDBT_CARRY=1`）。
///
/// 実測 2026-09-27: 既定は無効。`demod_one_tracked_frac` は `next` に端数の
/// 繰り上がりを 넣まないため、探索基準が常に整数サンプルに戻り、実 fs と
/// 指定 fs の差 2.5e-6 サンプル/シンボルが累積する。pirate 仮説。
/// 等化後の `|H|^2` を「発散」とみなす閾値。`ISDBT_EQHOT` で上書き可。
///
/// 理想は 1.0 付近。10 を超えた carrier は等化で 10 倍以上増幅され、
/// soft が ±8 のクランプに張り付く = 復調不能を意味する。
fn eq_hot_threshold() -> f64 {
    match std::env::var("ISDBT_EQHOT") {
        Ok(v) => v.parse().unwrap_or(10.0),
        Err(_) => 10.0,
    }
}

/// 等化後の外挿発散を時系列で追跡する（`ISDBT_EQSTAT=1`）。
pub fn eq_stat_enabled() -> bool {
    std::env::var("ISDBT_EQSTAT").map(|v| v != "0").unwrap_or(false)
}

fn carry_enabled() -> bool {
    std::env::var("ISDBT_CARRY").map(|v| v != "0").unwrap_or(false)
}

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
    /// 復号**できた**ブロック数と、そのうち先頭バイトが TS 同期 `0x47` だった数。
    ///
    /// 1seg の 1 TSD = 204 バイト = TS パケット 188 バイト + 16 バイト parity で、
    /// descramble 後の先頭バイトは必ず `0x47`。したがって
    /// 「同期バイトが一致したブロックの割合」は **byte 整列（commutator /
    /// block_phase / BYTE_LATENCY / byte deinterleaver）が保たれているか**を
    /// Viterbi の良否とは独立に判定できる。
    ///
    /// これが崩れていれば「soft は正しいのに RS が壊れる」原因が
    /// 整列のズレであり、Viterbi 自体には無実。
    /// 連続訂正不能長の分布（添字 = 連続長-1、値 = 発生回数）。
    /// PRBS リセットが起きた block_idx の列（`ISDBT_PRSBALL=1`）。
    prbs_reset_log: Vec<usize>,
    pub drop_burst_hist: [u64; 64],
    /// 最長バースト。
    pub drop_burst_max: u64,
    /// 段別ダンプの対象ブロック番号（`rs_blocks_seen` 基準）。
    pub dump_target: u64,
    /// 段別ダンプを有効にするか。
    pub dump_span: u64,
    /// 実運用の RS 符号語率を数えるか（`ISDBT_RSVPROBE=1`）。
    pub rs_valid_probe: bool,
    /// 実運用で RS 符号語（synd 全 0）だったブロック数。
    pub rs_valid: u64,
    /// 実運用でアセンブリされたブロック総数。
    pub rs_seen: u64,
    pub dump_enabled: bool,
    /// 実際の RS 入力ブロック（descramble 後、204 バイト）。
    pub dump_byte_pre: Vec<u8>,
    pub dump_byte_all: Vec<Vec<u8>>,
    pub dump_byte: Vec<u8>,
    pub rs_sync_ok: u64,
    pub rs_seen_decoded: u64,
    /// 訂正を要した RS ブロック数（復調 bit error の指標）。
    rs_corrected: usize,
    /// 訂正を要したビット総数。
    ///
    /// **注意: この値は診断に使えない**（実測 2026-09-29）。
    /// 加算しているのは `syndrome_weight()` の戻り値だが、あれは
    /// `[u8; NROOTS=16]` の非ゼロ要素**個数**なので 16 で頭打ちになる。
    /// 実測では `bit/blk` が全期間 16.0 に張り付いたまま drop が
    /// 0% → 100% に悪化し、「良い状態」と「最悪の状態」が同じ値になる。
    /// 正しい指標は `rs_symbol_errors`（RS が推定した symbol 誤り数）。
    rs_bit_errors: u64,
    /// RS が推定した symbol 誤り数の総和（RS の訂正能力 t=8 と比較する）。
    ///
    /// 訂正できたブロックもできなかったブロックも**同じ量**として数えるので、
    /// 時間平均を取れば「復調が実際にどれだけ壊れているか」が直接見える。
    pub rs_symbol_errors: u64,
    /// 復号失敗の理由ごとの回数（`rs::Fail` の添字）。
    ///
    /// 「訂正不能」は原因が 4 種類あるので、合計だけでは何が起きているか
    /// 分からない。実測で drop が 0% → 100% に単調悪化したとき、どこが
    /// 破綻したのかを切り分けるための指標。
    pub rs_fail_reasons: [u64; 4],
    /// **訂正不能**で丸ごと捨てられた RS ブロック数。
    ///
    /// これが H.264 MB 破損の直接原因になる。`rs::decode` が `None` を返すと
    /// そのブロックの 188 バイトが TS 出力から消えるが、`ContinuityTracker` が
    /// CC を詰めるため CC 不連続は検出されず、プレイヤーは欠落に気づかないまま
    /// 途切れたビットストリームを復号して MB エラーを出す。
    rs_dropped: usize,
    /// 訂正後も syndrome が非ゼロのブロック数（誤訂正 / 訂正失敗の silent failure）。
    rs_miscorrected: usize,
    /// 診断: 落下した block の `block_idx % 64` 分布。
    pub drop_by_mod64: [u32; 64],
    /// 診断: 落下した block の `block_idx % 256` 分布。
    pub drop_by_mod256: [u32; 256],
    /// 診断: 合計落下数（分布の分母）。
    pub drop_seen_blocks: usize,
    /// 復号器が**受け取った** RS ブロック総数（訂正不要を含む）。
    ///
    /// `rs_corrected` は synd が非ゼロだったブロックだけを数えるので、
    /// 「総ブロック数」が欲しいときはこれを使う。訂正不能率の分母も
    /// 本来はこちらを使うべき。
    pub rs_blocks_seen: u64,
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
        Self { buffer: Vec::with_capacity(TSP), phase, reset_off, prbs: EnergyPrbs::with_init(PRBS_INIT), block_idx: 0, rs_valid_probe: false, rs_valid: 0, rs_seen: 0, dump_target: u64::MAX, dump_span: 1, dump_enabled: false, dump_byte: Vec::new(), dump_byte_all: Vec::new(), dump_byte_pre: Vec::new(), prbs_reset_log: Vec::new(), drop_burst_hist: [0u64; 64], drop_burst_max: 0, rs_sync_ok: 0, rs_seen_decoded: 0, rs_corrected: 0, rs_bit_errors: 0, rs_symbol_errors: 0, rs_fail_reasons: [0; 4], rs_dropped: 0, rs_miscorrected: 0, rs_blocks_seen: 0, drop_burst: 0, burst_raised: false, drop_by_mod64: [0u32; 64], drop_by_mod256: [0u32; 256], drop_seen_blocks: 0 }
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
            // `ISDBT_RESETOFF=<n>` で reset 位相を外部から固定する（診断用）。
            //
            // 通常は `lock_params` が RS 復号率で `0..rp` を探索して決めるが、
            // その探索は**先頭の短い窓**でのみ評価されるため、「序頭は正しく
            // 後半で崩れる」症状なら正しい位相を選べている保証がない。
            // そこで位相を外部から固定し、**全区間**の drop 率で逐一評価する。
            // 診断: 強制した reset 位相を初回だけ出力する（どの探索結果が
    // 使われたかを確認するため）。
    if self.rs_blocks_seen == 0 {
        eprintln!(
            "[roff] lock_params が選んだ reset_off={} (ISDBT_RESETOFF={:?}) rp={}",
            self.reset_off,
            std::env::var("ISDBT_RESETOFF").ok(),
            reset_period()
        );
    }
    let ro = match std::env::var("ISDBT_RESETOFF") {
                Ok(v) => v.parse().unwrap_or(self.reset_off),
                Err(_) => self.reset_off,
            };
            if self.block_idx % reset_period() == ro {
                self.prbs.reset_to(PRBS_INIT);
                // 診断: PRBS リセットが実際にどの block_idx で起きているか。
                //
                // 前提は「block_idx が 1 ブロックにつき 1 だけ増える」こと。
                // これが崩れると reset_off=17 は正しくても実際の位相がずれる。
                if dbg_prbs_all_enabled() {
                    self.prbs_reset_log.push(self.block_idx);
                }
            }
            let start = self.phase;
            let mut ds = Vec::with_capacity(TSP);
            // ARIB STD-B31 §3.5 Energy dispersal:
            //   "All signals other than the synchronization byte in each of the
            //    transmission TSPs ... are EXCLUSIVE ORed using PRBSs"
            //   "the shift register must also perform shifting of the
            //    synchronization byte"
            //
            // 同期バイトも 1 バイトぶんシフト寄存器を進める。合計 204 バイト
            // ぶんが消費されるが、**消費する順序は「残り 203 バイト → 最後に
            // 同期バイト 1 バイト」**。
            //
            // 実測 2026-09-26: 「同期バイト分を先に消費」に並べ替えると
            // 復号が完全に壊れる（80MB 連続で drop 9565/9565 = 100%、
            // 分割 20MB でも drop 100%）。ARIB 3.5 の "shifting of the
            // synchronization byte" は「同期バイトの**位置**でも
            // シフトする」という意味であり、「同期バイトの**分**を
            // 残りより先に消費する」という意味ではない。実装は
            // 旧来の順序を維持する（実測で 100% 復号する）。
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
            // 段別バイトダンプ: **実際に RS へ入る 204 バイト**を保存する。
            //
            // 呼び出し側の byte ステージから取ると RS ブロック境界と
            // ずれて`](先頭が 0x8F で 0x47 にならない)`ので.dump、
            // アセンブラが 1 ブロックを完成させたタイミングで取る。
            // 段別ダンプ: RS へ入る 1 ブロック（descramble 後、204 バイト）。
            //
            // soft / vit は**このブロックに対応する範囲だけ**が欲���い。
            // アセンブラに入る直前の `self.dbg_bytes_to_rs` カウンタを
            // ブロック先頭で記録し、長さで切り出す（Viterbi depth と
            // byte 遅延を厳密に合わせるため）。
            if self.dump_enabled {
                // `rs_blocks_seen` はこの後で 1 増える（`+= 1` は下）。
                // したがって対象ブロックの判定は「増加前 + 1 == target」。
                let this_blk = self.rs_blocks_seen + 1;

                if this_blk >= self.dump_target
                    && this_blk < self.dump_target + self.dump_span
                {
                    // 参考: `ds` は descramble 後のバイト列。実測すると
                    // syndrome が 16/16 非ゼロで RS 符号語になっていないため、
                    // descramble **前**の生バイトも並べて確認する。
                    self.dump_byte_pre = self.buffer[start..start + TSP].to_vec();
                    if self.dump_byte_all.len() < 64 * 204 {
                        eprintln!(
                            "[dumptrace] rs_blocks_seen={} phase={} block_idx={} buf_len={} start={}",
                            self.rs_blocks_seen, self.phase, self.block_idx,
                            self.buffer.len(), start
                        );
                        self.dump_byte_all.push(ds.clone());
                    }
                    self.dump_byte = ds.clone();
                    eprintln!("[dump] assembler hit block {}", self.rs_blocks_seen);
                    // 選択済み soft / vit は Pipe 側で切り出す
                    // （ここではアセンブラなのでアクセスできない）。
                }
            }
            // 診断: **実運用**での RS 符号語率を数える。
            //
            // ロック時の `RS復.decode率=0.934` は `demod_and_align` の
            // 短い評価窓での値であり、これとは別物。実運用の streaming で
            // どれだけのブロックが実際に RS 符号語になるかを数える。
            if self.rs_valid_probe {
                if rs::is_codeword(&ds) {
                    self.rs_valid += 1;
                }
                self.rs_seen += 1;
            }
            let nz = rs::syndrome_weight(&ds);
            // 復号器が受け取ったブロックを数える。訂正不要（synd 全ゼロ）でも
            // 数える。訂正不能率の正しい分母。
            self.rs_blocks_seen += 1;
            if nz > 0 {
                self.rs_corrected += 1;
                self.rs_bit_errors += nz as u64;
            }
            // `nerr` は BM が推定した symbol 誤り数。訂正の成否に
            // かかわらず同じ量なので、時間平均が意味を持つ。
            let (decoded, nerr, fail) = rs::decode_detail(&ds);
            self.rs_symbol_errors += nerr as u64;
            if let Some(f) = fail {
                self.rs_fail_reasons[f as usize] += 1;
            }
            match decoded {
                Some(cw) => {
                    // TS 同期バイトの保持率（byte 整列 sanity check）。
                    // `cw[0]` は descramble 済みの TSD 先頭バイトで、
                    // 仕様どおり必ず 0x47。
                    self.rs_seen_decoded += 1;
                    if cw[0] == 0x47 {
                        self.rs_sync_ok += 1;
                    }
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
                    // 診断: 連続訂正不能（バースト）の**長さ分布**。
                    //
                    // `BURST_THRESHOLD`=32 は「1 フレーム(64ブロック)の半分」で
                    // _detection 基準としてimba された値だが、**実測のバースト長
                    // より長い可能性**があり、その場合 discontinuity 注入が
                    // 一度も発動せず、H.264 の GOP 破綻を防げない。
                    //
                    // I フレームは 2 秒 = 128 ブロック周期なので、バーストが
                    // I フレームを含む長さに達すれば画像が落ちる。よって
                    // 「何個連続したら I フレームを含むか」が本質的な閾値。
                    if self.drop_burst as usize <= 64 {
                        self.drop_burst_hist[self.drop_burst - 1] += 1;
                    }
                    if self.drop_burst as u64 > self.drop_burst_max {
                        self.drop_burst_max = self.drop_burst as u64;
                    }
                    // 診断: どの block が落ちたかを modulo 64/256 で記録する。
                    // 1 [dbg] 間隔(214 blk)あたり 36 個が常に落ちるので、
                    // 特定の block 位置が構造的に落ちている（commutator /
                    // reset_off / block_phase の周期 64 Cytin）疑いがある。
                    self.drop_by_mod64[self.block_idx % 64] += 1;
                    self.drop_by_mod256[self.block_idx % 256] += 1;
                    self.drop_seen_blocks += 1;
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
        // 探索窓は周期を区別できる長さが要る。128 ブロック（周期 64 の 2 周期）
        // だと周期 32 でも 64 でも 78 でも窓内で同点になり、最後の候補を
        // 選んでしまう。すると実行中に周期がずれ、後半から PRBS 解除が
        // bit ずれ続ける（＝「先頭は完璧、後半で崩れる」）。
        // 走査候補を何周期分も含む長さにして区別できるようにする。
        let rp = reset_period();
        let win = (128..=512).find(|w| *w >= rp * 4).unwrap_or(512);
        for reset_off in 0..rp {
            let mut prbs = EnergyPrbs::with_init(PRBS_INIT);
            let (mut ok, mut tot) = (0usize, 0usize);
            for (idx, blk) in blocks.iter().take(win).enumerate() {
                if idx % rp == reset_off {
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
    /// 診断: 1 TSD あたりの degraded 数（0..14, 15 で飽和）のヒストグラム。
    pub dbg_degraded_hist: [u64; 16],
    /// 診断: ヒストグラムの総ブロック数。
    pub dbg_degraded_blocks: u64,
    /// 診断: 集計中の 1 TSD あたりの degraded カウンタ。
    pub degraded_in_block: usize,
    /// 診断: 1 ブロックを詳細トレースするための状態（`ISDBT_TRACE=<blk>`）。
    pub trace: Option<crate::trace::BlockTrace>,
    /// 診断: 等化後に `|H|^2` が THRESHOLD を超えたキャリア数（累積）。
    pub dbg_eq_hot: u64,
    /// 診断: 累積サンプル数（`dbg_eq_hot` の分母）。
    pub dbg_eq_n: u64,
    /// 診断: 発散キャリアの周波数位置。432 bin の累積ヒストグラム。
    pub dbg_eq_hot_pos: [u64; 432],
    /// 診断: `|H|^2` の最大値（EMA）。
    pub dbg_eq_max_ema: f64,
    /// 診断: 全キャリアの `|Y|^2`（等化前 = `|EQ|^2 * |H|^2`）累積。
    pub dbg_y2_all: f64,
    pub dbg_y2_n: u64,
    /// 診断: 全キャリアの `|H|^2` 累積。
    pub dbg_h2_all: f64,
    /// 診断: 区間平均用の累積（周期的に 0 に戻して「瞬時」値を出す）。
    pub dbg_y2_win: f64,
    pub dbg_h2_win: f64,
    pub dbg_win_n: u64,
    /// 診断: **SP 位置だけ**の `|Y|^2` 窓平均。全キャリア平均は空間的に
    /// 均されて FFT 窓ずれの効果が消えるため、SP に限定する必要がある。
    pub dbg_ysp_win: f64,
    pub dbg_hsp_win: f64,
    pub dbg_spn: u64,
    /// 診断: **等化前** `|seg[l]|` の SP 位置窓平均（`|Y_SP|` の真の原因切り分け用）。
    pub dbg_segsp_win: f64,
    /// 診断: 位相別（`symbol%4` == 0/1/2/3）の `|seg[l]|` 窓平均。
    ///
    /// 4 位相すべてで均等に落ちれば「SP 添字集合全体に共通する原因」、
    /// 特定位相だけなら「位相ごとの SP 添字計算経路のバグ」。
    pub dbg_segsp_ph: [f64; 4],
    pub dbg_segsp_ph_n: [u64; 4],
    /// 診断: SP 位相の**円平均**（PRBS 値を外したときの残余位相）。
    ///
    /// `|seg[l]|` は位相回転で不変なので、振幅低下の原因にはならない。
    /// ここでは「位相がincoherent に散らばっているか」を見る。
    pub dbg_sp_cisum: [f64; 2],
    pub dbg_sp_cin: u64,
    /// 診断: SP ごとの正規化位相 `seg[l]/values[l]`。
    /// `|1 - w|` が τ（0.5）超なら incoherent とみなす。
    /// 36 本中何本 incoherence かが「部分的な destructive interference」を
    /// 検出する（pooled CI は 2/36 の損失では動かない）。
    pub dbg_sp_incoh: u64,
    pub dbg_sp_tot: u64,
    /// 診断: 円平均**引数** `arg(mean(seg[l]/values[l]))`。
    /// 共同的位相回転なら単調に増える（積分なら線形）。
    /// destructive interference なら散らばって平均化され、
    /// 3 此类推論で切り分ける。
    pub dbg_sp_arg: f64,
    pub dbg_sp_arg_n: u64,
    /// 診断: **SP 1 本単位**の異常内訳。
    ///
    /// `dbg_sp_bad_idx[phase][k]` = 位相 `phase`(symbol%4) の k 番目の SP
    /// が窓内で incoherent だった回数。`dbg_sp_seen_idx` は対応する観測数。
    ///
    /// これが「毎回ランダムな数本」なら SNR 劣化、「特定の少数の添字だけ
    /// 繰り返し異常」なら `prbs_pilot_values()` のビットレベルのバグ。
    pub dbg_sp_bad_idx: [[u64; 36]; 4],
    pub dbg_sp_seen_idx: [[u64; 36]; 4],
    /// 診断: 追跡するキャリアの `|H|^2` 累積（窓平均で出す）。
    ///
    /// 添字は局所 `k`（`sp_carriers` の通し番号）。`h[k+1]` が実際の
    /// キャリア番号。null 深化（k=205/207/210）と両端外挿劣化（k=430）を
    /// 対照キャリア（k=100）と比較する。
    pub dbg_track_h2: [f64; 8],
    pub dbg_track_n: u64,
    /// 診断: `values[204]` の実行時ビットパターン（`ISDBT_PVAL=1`）。
    ///
    /// `values` は `Pipe::new` で一度だけ生成される純関数の `Vec<f32>`。
    /// コード監査だけでは「実行時に書き換わっていない」ことを保証できない
    /// ので、実測中にビットパターン（`f32::to_bits`）を記録し、最初の
    /// サンプルと一致するかを確認する。
    pub dbg_pval_bits: u32,
    pub dbg_pval_n: u64,
    /// 診断: `values` **全 432 本**の累積ハッシュ。
    /// `values[204]` だけだと他が書き換わった可能性が残るので、全体を見る。
    /// 最初は 0（未観測）、観測後は以降ずっと同じ値でなければならない。
    pub dbg_pval_hash: u64,
    pub dbg_pval_hash_set: bool,
    /// 診断: 発散した**実キャリア番号** `l` の分布（432 実キャリア単位）。
    ///
    /// 旧 `dbg_eq_hot_pos` は `i * 432 / n` で 432 等分した bin を使って
    /// おり、4 キャリアずつまとめられていた（実測ミス 3: `k=205` を実
    /// キャリア番号と誤認）。ここでは `l = eq_idx[i]` で実キャリアを
    /// そのまま記録する。
    pub dbg_hot_l: [u64; 432],
    /// 診断: 追跡キャリアの**等化前** `|seg[l]|^2` 窓平均（SP 実測値）。
    pub dbg_trk_raw: [f64; 6],
    /// 診断: 単一キャリアの**窓なし時系列**。
    ///
    /// `[spband]` は前半/後半の集計で時間情報が落ちるので、単調上昇か
    /// 振動かが判らない。窓を切らずに累積し、出一条ごとにsnapshot して
    /// 「単調 / 振動 / 階段」の 3 形態を判別する。
    ///
    /// 添字は `[204, 210, 192, 216, 100, 207]`。
    pub dbg_ts: [f64; 6],
    pub dbg_ts_n: u64,
    /// 診断: 直前のスナップショットの累積値（差分計算用）。
    pub dbg_ts_prev: [f64; 6],
    /// 診断: 最新スナップショットの**差分**（1 SP 巡あたりの増分）。
    pub dbg_ts_last: [f64; 6],
    /// 診断: 上記の差分の指数移動平均。
    ///
    /// `dbg_ts_last` は 36 サンプル（1 SP 巡）ごとに更新されるが、
    /// `[eqhot]` の出力間隔は 43878 サンプルなので、そのままだと
    /// 1219 回に 1 回しかサンプルできない。ema で平滑化する。
    pub dbg_ts_ema: [f64; 6],
    /// 診断: null profile 用の 11 キャリア時系列。
    ///
    /// `l=195,198,...,225`（3 間隔、11 本）。全部 SP 位相に一致する
    /// （`l%12 ∈ {0,3,6,9}`）。`l=207/210` の急落が中心にある
    /// null が「単一 profile として深化」するのか「2 本の孤立した
    /// 離散機構」なのかを判別する。
    /// 診断: 絶対 spectrum bin 502..=522 の `|seg[k]|²`（ema）。
    ///
    /// 絶対 bin で見るのは `l` ではなく `seg_off + k` の位置を直接追うため。
    /// `seg_off` は capture ごとに 307/308 とズレるので、`l` で較べると
    /// 同じ `l` でも spectrum 位置が 1 bin ずれる陷阱がある。
    pub dbg_abs: [f64; 21],
    pub dbg_abs_prev: [f64; 21],
    pub dbg_abs_ema: [f64; 21],
    pub dbg_prof: [f64; 11],
    pub dbg_prof_prev: [f64; 11],
    pub dbg_prof_ema: [f64; 11],
    pub dbg_prof_n: u64,
    /// 診断: 追跡キャリアの**補間後** `|h[l]|^2` 窓平均（等化器が見る値）。
    pub dbg_trk_h: [f64; 6],
    /// 診断: SP 帯域像。`SP_LS[l]` の `|seg[l]|^2` を**累積**（窓切���し）。
    ///
    /// 周波数選択性フェージングなら、この帯域像全体が「うねる」。
    /// 1 本だけ孤立して動くなら `l=204` 固有の問題。
    /// 窓を切らないので全体像の形状が失われない。
    pub dbg_sp_band: [f64; 432],
    pub dbg_sp_band_n: u64,
    /// 診断: 前半・後半で分けた SP 帯域像（時間発展の比較用）。
    /// 全体の半分（`dbg_sp_band_n` が 200k 到達）で前半を凍結し、
    /// 後半を別のバッファに積む。
    pub dbg_sp_band1: [f64; 432],
    pub dbg_sp_band1_n: u64,
    /// 診断: 後半の SP 帯域像。
    pub dbg_sp_band2: [f64; 432],
    pub dbg_sp_band2_n: u64,
    /// 診断: 上 2 配列の窓を切るまでのサンプル数。
    /// 診断: `|h[l]|` の**瞬時分布**（対数ヒストグラム）。
    ///
    /// 窓平均は分布の形を隠す。`|H|` の平均が grow しても下側の裾が
    /// 伸びれば `|Y|²/|H|² > 10` の発生率が上がる — つまり「平均 grow」
    /// と「hot 増加」は矛盾なく両立する。**下位パーセンタイルが鍵**。
    ///
    /// bin は `|h|` の自然対数。`ln(|h|)` を [-6, +3] に 45 分割
    /// （1 分割あたり約 0.2 = exp  比 1.22）。`i = (ln|h| + 6) * 5`。
    pub dbg_h_hist: [[u64; 45]; 4],
    /// `dbg_h_hist` を 4 時点で凍結したスナップショット（差分で時系列化）。
    /// `[phase][slot][bin]`。
    pub dbg_h_snap: [[[u64; 45]; 4]; 12],
    /// 現在のフェーズ（0..4）。
    pub dbg_h_phase: usize,
    /// フェーズ内のサンプル数。
    pub dbg_h_sn: u64,
    /// 凍結フェーズ数（`dbg_h_snap` の有効範囲）。
    pub dbg_h_nphase: usize,
    /// slot 0/1/2 = `l=204` / `l=205` / `l=216` の SP 実測,
    /// slot 3 = `l=205` の補間 `|h|`。
    pub dbg_trk_n: u64,
    /// 診断: **スロットごと**の観測数。
    ///
    /// SP 側（`l=204/216/100`）は 4 シンボルに 1 回、データキャリア側
    /// （`l=205/100`）は毎シンボルに観測されるので、分母が同じでは
    /// _rb ない。実測ミス 5 を避けるためスロットごとに数える。
    pub dbg_trk_cnt: [u64; 6],
    /// 診断: 処理済みシンボル数（トレースの `sym_idx` 用）。
    pub syms_done: u64,
    /// 診断: `rs_asm` が返さなかったバイト数（= 訂正不能で消えた分）。
    pub rs_lost: u64,
    /// 診断: 直近 `process` へ渡したキャリア数。
    pub dbg_carriers_last: u64,
    /// 診断: 1 シンボル処理時間の累積（μs）と最大値、計測シンボル数。
    pub t_sum_us: u64,
    /// 診断: (完了ブロック数, その時点の `depu_pos`)。
    pub dbg_depu_at_block: Vec<(usize, usize)>,
    /// 全 RS ブロック（訂正不能含む）の `(ブロック番号, depu_pos % 4)`。
    ///
    /// 1 OFDM フレーム = 64 ブロックなので、**64 ブロック周期で同じ値へ戻る**
    /// のが正しい。戻らなければ depuncture の位相がスリップしている。
    /// `dbg_depu_at_block` は成功ブロックのみなので劣化域では使えない。
    pub dbg_depu_all: Vec<(usize, usize)>,
    pub t_max_us: u64,
    pub t_n: usize,
    /// TMCC フレーム境界（204 シンボルごと）の RS ブロック位置。
    pub tmcc_frame_pos: Vec<usize>,
    /// 段ごとバイトダンプ（`ISDBT_DUMP=<blk>`、既定無効）。
    ///
    /// 「どこで情報が失われるか」を 1 ブロック単位で追うためのもの。
    /// 既存のトレースは統計値（平均/RMS）しか出さないため、
    /// 「健全な平均」に隠れた 1 バイトの変化が見えなかった。
    ///
    /// - `dump_soft`: depuncture 後の soft を {-1,0,+1} に量子化したビット列
    /// - `dump_vit`: Viterbi 出力ビット列（0/1）
    /// - `dump_byte`: byte デインターリーブ後、RS へ入る直前のバイト列
    pub dump_target: Option<u64>,
    /// 段別ダンプが有効か（`Pipe` 側フラグ、`rs_asm.dump_enabled` と対応）。
    pub dump_enabled: bool,
    pub dump_soft_f32: Vec<f32>,
    /// **原子的な** 1 ブロック分の記録。
    ///
    /// 過去の計測は soft / vit / RS 入力を 3 つの独立したリングから
    /// 取り出しており、切り出しタイミングが合わず「soft は健全なのに
    /// RS は壊れている」という**結論自体が誤り**になっていた。
    ///
    /// ここでは byte ごとに (soft カーソル, vit カーソル) を記録し、
    /// アセンブラが 204 バイトを完成させた時点で、その区間に対応する
    /// soft / vit を**同じブロックから**切り出す。
    pub dump_map: Vec<(usize, usize)>,
    /// ブロック完了時にリング全体をスナップショットした soft。
    pub dump_soft_snap: Vec<f32>,
    /// RS へ入ったバイトの絶対連番（診断用、リセットしない）。
    pub rs_byte_abs: u64,
    /// 各 RS バイトの出所 vit ビット位置（絶対）。
    pub rs_byte_vitpos: Vec<u64>,
    /// Viterbi 出力ビットの絶対連番。
    pub vit_bits_out: u64,
    /// 原子スナップショット時の `ByteDeinterleaver::idx`（絶対）。
    ///
    /// `push()` は `self.idx % BI_I` で分岐を選ぶため、idx=0 から
    /// 始于 offline 再生では online と**別の分岐**にバイトが乗り、
    /// 出力バイト列が一致しない（実測: 先頭 1 バイトだけ 0x47 で
    /// 2 バイト目以降が全滅）。离线侧で idx を合わせる必要がある。
    pub dump_deint_idx: usize,
    /// 対象ブロックの記録（soft, vit, byte が同一ブロック由来）。
    pub dump_atom: Option<(Vec<f32>, Vec<u8>, Vec<u8>)>,
    pub dump_soft: Vec<u8>,
    pub dump_vit: Vec<u8>,
    pub dump_byte: Vec<u8>,
    pub dump_capture: bool,
    /// 現在の TMCC フレーム内でのシンボル数（0..204）。
    pub tmcc_sym_count: usize,
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
    /// -demod/Viterbi 内部状態を捨てて**整列の進行位置は保持したまま**
    /// Pipe を作り直す。
    ///
    /// 状態汚染（実測 2026-09-26）のCES sister として需要的。
    /// ただし `byte_in_idx` / `deint_out_idx` / `block_idx` まで 0 に戻すと
    /// commutator と byte 整列の暖机和 PRBS リセット位置が同時にやり直され、
    /// RS ブロックが二度と完成しなくなる（実測: 総blk=0、出力 0 バイト）。
    /// したがって**消費済みバイト数と PRBS ブロック位置は引き継ぐ**。
    ///
    /// 捨てるのは Viterbi のトレリス状態、バイト詰めの繰り越し
    /// （`bit_acc`/`bit_cnt`）、`block_buf` の部分ブロック。
    /// これらは样本ごとに独立で、保持すると以其整合が壊れる。
    /// 内部状態を部分的に初期化して、長時間走ったときの破綻から復帰する。
    ///
    /// 実測 2026-09-26: 同じ IQ を分割して singly デコードすると 0.3〜1.6%、
    /// 連結すると 24〜58% に劣化する（h1 2回連結で 57.95%）。IQ 側は正常で、
    /// 連続処理の状態だけが汚染される。パイプラインの段落ち込みはない
    /// （`to_rs=1,951,413` に対し `drop_lat=2,303` は一定）ので、破綻は
    /// RS 到達前の内部状態にある。
    ///
    /// **保持するもの**（ここを 0 に戻すと Pipe 全体が死ぬ）:
    /// - `byte_in_idx` / `deint_out_idx`: commutator と byte 整列の進行位置。
    ///   戻すと暖機と byte 境界がずれて RS ブロックが二度と完成しない
    ///   （実測: 総blk=0、出力 0 バイト）。
    /// - `block_idx`: PRBS リセット位置。戻すと PRBS 位相がずれる。
    /// - `block_buf` の部分ブロック: 破棄すると既に溜めたバイトが失われ、
    ///   RS アセンブラと Viterbi 側で不整合が起きて全ブロックが訂正不能に
    ///   なる（実測: 総blk=35 すべて drop、出力 0 バイト）。
    ///
    /// **初期化するもの**（状態汚染の候補）:
    /// - Viterbi のトレリス状態
    /// - バイト詰めの繰り越し（`bit_acc` / `bit_cnt`）
    /// - 診断カウンタ
    fn resync(&mut self) {
        self.vit.reset();
        self.bit_acc = 0;
        self.bit_cnt = 0;
        self.mother.clear();
    }

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
            dbg_degraded_hist: [0u64; 16],
            dbg_degraded_blocks: 0,
            degraded_in_block: 0,
            trace: None,
            dbg_eq_hot: 0,
            dbg_eq_n: 0,
            dbg_eq_hot_pos: [0u64; 432],
            dbg_eq_max_ema: 0.0,
            dbg_y2_all: 0.0,
            dbg_y2_n: 0,
            dbg_h2_all: 0.0,
            dbg_y2_win: 0.0,
            dbg_h2_win: 0.0,
            dbg_win_n: 0,
            dbg_ysp_win: 0.0,
            dbg_hsp_win: 0.0,
            dbg_spn: 0,
            dbg_segsp_win: 0.0,
            dbg_segsp_ph: [0.0; 4],
            dbg_segsp_ph_n: [0; 4],
            dbg_sp_cisum: [0.0; 2],
            dbg_sp_cin: 0,
            dbg_sp_incoh: 0,
            dbg_sp_tot: 0,
            dbg_sp_arg: 0.0,
            dbg_sp_arg_n: 0,
            dbg_sp_bad_idx: [[0; 36]; 4],
            dbg_sp_seen_idx: [[0; 36]; 4],
            dbg_track_h2: [0.0; 8],
            dbg_track_n: 0,
            dbg_hot_l: [0u64; 432],
            dbg_pval_bits: 0,
            dbg_pval_n: 0,
            dbg_pval_hash: 0,
            dbg_pval_hash_set: false,
            dbg_trk_raw: [0.0; 6],
            dbg_ts: [0.0; 6],
            dbg_ts_n: 0,
            dbg_ts_prev: [0.0; 6],
            dbg_ts_last: [0.0; 6],
            dbg_ts_ema: [0.0; 6],
            dbg_abs: [0.0; 21],
            dbg_abs_prev: [0.0; 21],
            dbg_abs_ema: [0.0; 21],
            dbg_prof: [0.0; 11],
            dbg_prof_prev: [0.0; 11],
            dbg_prof_ema: [0.0; 11],
            dbg_prof_n: 0,
            dbg_trk_h: [0.0; 6],
            dbg_sp_band: [0.0; 432],
            dbg_sp_band_n: 0,
            dbg_sp_band1: [0.0; 432],
            dbg_sp_band1_n: 0,
            dbg_sp_band2: [0.0; 432],
            dbg_sp_band2_n: 0,
            dbg_h_hist: [[0; 45]; 4],
            dbg_h_snap: [[[0; 45]; 4]; 12],
            dbg_h_phase: 0,
            dbg_h_sn: 0,
            dbg_h_nphase: 12,
            dbg_trk_n: 0,
            dbg_trk_cnt: [0; 6],
            syms_done: 0,
            rs_lost: 0,
            dbg_depu_at_block: Vec::with_capacity(2048),
        dbg_depu_all: Vec::with_capacity(65536),
            t_sum_us: 0,
            t_max_us: 0,
            t_n: 0,
            dump_target: None,
            dump_enabled: false,
            dump_soft_f32: Vec::new(), dump_map: Vec::new(), dump_soft_snap: Vec::new(), dump_atom: None, rs_byte_abs: 0, rs_byte_vitpos: Vec::new(), vit_bits_out: 0, dump_deint_idx: 0, dump_soft: Vec::new(),
            dump_vit: Vec::new(),
            dump_byte: Vec::new(),
            dump_capture: false,
            tmcc_frame_pos: Vec::new(),
            tmcc_sym_count: 0,
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
        // 診断: 1 シンボルの処理時間を測る。CPU 律速（熱スロットリング含む）か
        // 復調品質低下かを切り分けるため。
        //
        // 実測 2026-09-26: 386MB 連続だと drop 率が 0.2% → 90.4% と
        // ブロック数に比例して劣化するが、原因不明。処理時間も同様に
        // 伸びていれば CPU 律速、フラットなら復調側。
        let _t_start = std::time::Instant::now();
        self.dbg_syms += 1;
        self.syms_done += 1;

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
        // 診断: トレース用の SP 生値を控えておく（トレース有効時のみ）。
        let trace_sp: Vec<(usize, Complex32)> = if self.trace.is_some() {
            self.pilots
                .sp_carriers(sym_mod4)
                .map(|l| (l, seg[l] / self.pilots.values[l]))
                .collect()
        } else {
            Vec::new()
        };
        let eq = equalize(&seg, &h);
        // `data` の元の carrier インデックス（`h[l]` を見るために必要）。
        let eq_idx: Vec<usize> = data_carrier_indices(sym_mod4, &self.pilots);
        let data: Vec<Complex32> = eq_idx.iter().map(|&l| eq[l]).collect();
        self.dbg_carriers_last = data.len() as u64;
        // 診断: 等化後の外挿発散を時系列で追跡する。
        //
        // `estimate_channel` は SP 間を線形補間する��が、帯域両端は
        // `slope` で**線形外挿**する。SP の振幅が定常的に正常範囲に収まって
        // いても、外挿の傾きは両端 SP 2 点の差から求まるので、外挿 carrier で
        // は増幅`|Y| / |H|` が大きく出る。`|H|` が小さいと等化で増幅され、
        // soft が ±8 のクランプに張り付く → RS 訂正限界を超える。
        //
        // これが「時間とともに増える」のか「定常的な弱点」なのかを
        // 切り分けるため、閾値超の**数**と**発生位置**を記録する。
        if crate::stream::eq_stat_enabled() {
            let th = eq_hot_threshold();
            let n = data.len();
            // SP 位置の `|Y|` を直接測る。
            //
            // 注意: `data_carrier_indices` は **SP を除外する**（データキャリア
            // だけ）。そのため `data` の中に SP は無く、`eq_idx` から SP 位置を
            // 探しても必ず `None` になる（実測ミス 2: `Ysp` が常に 0.0000）。
            // SP の `|Y|` は FFT 値 `seg[l]` から直接取る。
            for (sp_k, l) in self.pilots.sp_carriers(sym_mod4).enumerate() {
                if l >= seg.len() {
                    continue;
                }
                // **等化前**の FFT 出力 `|seg[l]|` を別に取る。
                //
                // `|Y_SP| = |seg[l]| / (4/3)` なので `|Y_SP|` の低下は
                // `|seg[l]|` の低下と同義。`|seg[l]|` だけ取り出すと、
                // SP 位置の固有劣化か、FFT 出力全体の低下かを区別できる。
                let raw2 = (seg[l].re as f64) * (seg[l].re as f64) + (seg[l].im as f64) * (seg[l].im as f64);
                self.dbg_segsp_win += raw2;
                // 複素 SP と既知パイロット値の位相差を円平均する。
                // |1 - CI| が 0 に近いほど SP 位相が coherent。
                let pval = self.pilots.values[l] as f64;
                if pval != 0.0 {
                    // `seg[l]` は `Complex32`。`f64` で除算した結果は
                    // `Complex<f64>` だが、手で実部・虚部を取る（`as f64`
                    // 変換だと型が崩れる）。
                    let w = seg[l] / (self.pilots.values[l] as f32);
                    let (wr, wi) = (w.re as f64, w.im as f64);
                    self.dbg_sp_cisum[0] += wr;
                    self.dbg_sp_cisum[1] += wi;
                    self.dbg_sp_cin += 1;
                    // `|1 - w|` が τ 超なら incoherent（完全 coherent なら
                    // `|1-w|` は小さい）。
                    let dev = ((1.0 - wr) * (1.0 - wr) + wi * wi).sqrt();
                    if dev > 0.5 {
                        self.dbg_sp_incoh += 1;
                    }
                    self.dbg_sp_tot += 1;
                    // SP 1 本単位の内訳。`sp_k` は位相内での通し番号
                    // （`l = 3*phase + 12*k`）。36 本のうちどの添字が
                    // 繰り返し異常かを特定する。
                    if sp_k < 36 {
                        let ph4 = sym_mod4 % 4;
                        self.dbg_sp_seen_idx[ph4][sp_k] += 1;
                        if dev > 0.5 {
                            self.dbg_sp_bad_idx[ph4][sp_k] += 1;
                        }
                    }
                    // SP 実測値 `|seg[l]|^2` を追跡。
                    //
                    // 注意: `l=204` は `l mod 12 == 0` なので `symbol%4 == 0`
                    // の時だけ SP、それ以外の位相では**データキャリア**。
                    // `eq_idx` 経由で測ると SP と非 SP が混ざる（実測ミス 4:
                    // `I204 != R204` という物理的にありえない差が出た）。
                    // ここでは SP として確定したシンボルだけを見る。
                    // slot: 0=`l=204`, 2=`l=216`, 3=`l=100`
                    // `values[204]` の実行時ビットパターンを記録。
                    // これが 1 ビットも変わらなければ `R204` の 80% 上昇は
                    // `seg[204]` 側（実測信号）の問題と確定する。
                    if l == 204 {
                        self.dbg_pval_bits = self.pilots.values[204].to_bits();
                        self.dbg_pval_n += 1;
                    }
                    // SP 帯域像（`l` ごとの `|seg[l]|^2` を累積）。
                    //
                    // SP は `l mod 12 == 3*(symbol%4)` なので 4 シンボル
                    // ごとに別の `l` が現れる。36 本 × 4 位相 = 144 本が
                    // 時間とともに平均される。`l` ごとに分けて累積する
                    // ことで、周波数選択性の帯域像が時間とともにどう変
                    // わるかが見える（窓を切らない）。
                    if l < 432 {
                        self.dbg_sp_band[l] += raw2;
                        self.dbg_sp_band_n += 1;
                        // 時系列スナップショット: SP 36 本ごとに 1 回。
                        // 累積値の**差分**を出すので、単調上昇か振動かが
                        // 判る。累積値そのものは単調なので使わない。
                        self.dbg_ts_n += 1;
                        if self.dbg_ts_n >= 36 {
                            self.dbg_ts_n = 0;
                            for t in 0..6 {
                                let d = self.dbg_ts[t] - self.dbg_ts_prev[t];
                                self.dbg_ts_prev[t] = self.dbg_ts[t];
                                self.dbg_ts_last[t] = d;
                                // 36 サンプルごとの差分を ema で平滑化。
                                // 1219 回に 1 回しか出力されないため。
                                self.dbg_ts_ema[t] =
                                    0.999 * self.dbg_ts_ema[t] + 0.001 * d;
                            }
                            // null profile も同様に ema 化する。
                            for t in 0..11 {
                                let d = self.dbg_prof[t] - self.dbg_prof_prev[t];
                                self.dbg_prof_prev[t] = self.dbg_prof[t];
                                self.dbg_prof_ema[t] =
                                    0.999 * self.dbg_prof_ema[t] + 0.001 * d;
                            }
                            // 絶対 bin 502..=522 も同様に ema 化する。
                            for t in 0..21 {
                                let d = self.dbg_abs[t] - self.dbg_abs_prev[t];
                                self.dbg_abs_prev[t] = self.dbg_abs[t];
                                self.dbg_abs_ema[t] =
                                    0.999 * self.dbg_abs_ema[t] + 0.001 * d;
                            }
                        }
                        // SP ループ 1 巡（`l` が先頭に戻った）ごとに
                        // 前半バッファを凍結し、以降は後半バッファに積む。
                        //
                        // 全体を半分で割る近似だが、帯域像の**形状**が
                        // 時間とともにどう変わるかを見るEnough。
                        // **全 `l`** を前半/後半バッファに積む。
                        // 旧実装は `l == 3*(sym%4)` のときだけ積んでいた
                        // ため 4 本しか取れなかった（実測ミス 6）。
                        //
                        // 切替は累積総数 `dbg_sp_band_n` で行う。
                        // SP 1 巡 = 36 回なので 50,000 回 ≒ 1,389 巡。
                        if self.dbg_sp_band_n < 50_000 {
                            self.dbg_sp_band1[l] += raw2;
                            self.dbg_sp_band1_n += 1;
                        } else {
                            self.dbg_sp_band2[l] += raw2;
                            self.dbg_sp_band2_n += 1;
                        }
                    }
                    // 全 432 本のハッシュ（SP ループ 1 巡ごとに更新）。
                    // 1 巡 = 36 回なので、`l == sp_carriers` の先頭だけ
                    // で更新する（`l == 3*(sym%4)` のとき）。
                    if l == 3 * (sym_mod4 % 4) {
                        let mut h: u64 = 1469598103934665603;
                        for v in self.pilots.values.iter() {
                            h ^= v.to_bits() as u64;
                            h = h.wrapping_mul(1099511628211);
                        }
                        if !self.dbg_pval_hash_set {
                            self.dbg_pval_hash = h;
                            self.dbg_pval_hash_set = true;
                        } else if h != self.dbg_pval_hash {
                            eprintln!(
                                "[pval] values が変化した! hash={h:#018x} (初回 {:#018x})",
                                self.dbg_pval_hash
                            );
                        }
                    }
                    // 単一キャリアの窓なし時系列。
                    //
                    // `[spband]` は前半/後半の集計で時間情報が落ちているため、
                    // 単調上昇・振動・階段の判別ができない。累積し、
                    // getter で累積値をそのまま返す（窓を切らない）。
                    //
                    // 対照 Included:
                    //  - `l=204`  DC−1 bin、SP、144 SP 中最大の増加
                    //  - `l=210`  DC+5 bin、SP、最大の減少
                    //  - `l=192`  DC−13 bin、SP、遠い対照
                    //  - `l=216`  DC+11 bin、SP、同じ `l%12==0` の対照
                    //  - `l=100`  DC ��105 bin、中央の健全 SP
                    //  - `l=207`  DC+2 bin、SP、減少側
                    // null profile（11 本、3 間隔）。
                    // `l=195..225` は全部 SP 位相に一致する。
                    // `l=207/210` を中心に単一の谷か、2 本の孤立した
                    // 離散機構かを判別する。
                    const PROF: [(usize, usize); 11] = [
                        (195, 0), (198, 1), (201, 2), (204, 3), (207, 4),
                        (210, 5), (213, 6), (216, 7), (219, 8), (222, 9), (225, 10),
                    ];
                    // 診断: 絶対 spectrum bin 502..=522 を直接見る。
                    //
                    // `seg` は `seg_off` 引き済みなので、絶対 bin `b` の値は
                    // `seg[b - seg_off]`。`seg_off` は capture ごとに 307/308
                    // とズレるため、`l` で較べると同じ `l` でも spectrum
                    // 位置が 1 bin ずれる。**絶対 bin で追う**のが正解。
                    //
                    // ここで見たいのは:
                    //  - 谷の幅が 1 bin 程度か、数 bin 幅か
                    //  - atk1 と late で谷の位置が一致するか
                    for k in 0..21usize {
                        let abs_bin = 502 + k;
                        if abs_bin >= self.seg_off && abs_bin < self.seg_off + 432 {
                            let v = seg[abs_bin - self.seg_off];
                            self.dbg_abs[k] += (v.re as f64) * (v.re as f64)
                                + (v.im as f64) * (v.im as f64);
                        }
                    }
                    for (tl, slot) in PROF.iter() {
                        if l == *tl {
                            self.dbg_prof[*slot] += raw2;
                        }
                    }
                    const TRK_TS: [(usize, usize); 6] =
                        [(204, 0), (210, 1), (192, 2), (216, 3), (100, 4), (207, 5)];
                    for (tl, slot) in TRK_TS.iter() {
                        if l == *tl {
                            self.dbg_ts[*slot] += raw2;
                        }
                    }
                    const TRK_SP: [(usize, usize); 3] = [(204, 0), (216, 2), (100, 3)];
                    for (tl, slot) in TRK_SP.iter() {
                        if l == *tl {
                            self.dbg_trk_raw[*slot] += raw2;
                            let hv = h[l];
                            self.dbg_trk_h[*slot] +=
                                (hv.re as f64) * (hv.re as f64) + (hv.im as f64) * (hv.im as f64);
                            self.dbg_trk_n += 1;
                            self.dbg_trk_cnt[*slot] += 1;
                        }
                    }
                    // 診断: `|h[204]|` / `|h[216]|`（SP 実測）と
                    // `|h[205]|`（補間）の**瞬時分布**を対数ヒストグラムに
                    // 積む。窓平均は分布の形を隠すため、下位パーセンタイル
                    // が見えない。
                    //
                    // slot 0 = `l=204` SP 実測, 1 = `l=216` SP 実測,
                    // 2 = `l=205` 補間 `h`
                    if l == 204 || l == 216 || l == 205 {
                        let hv = h[l];
                        let mag2 = (hv.re as f64) * (hv.re as f64) + (hv.im as f64) * (hv.im as f64);
                        let lnm = 0.5 * mag2.ln();
                        if lnm > -6.0 && lnm < 3.0 {
                            let b = (((lnm + 6.0) * 5.0) as usize).min(44);
                            let slot = if l == 204 {
                                0
                            } else if l == 216 {
                                1
                            } else {
                                2
                            };
                            self.dbg_h_hist[slot][b] += 1;
                            // 4 等分して各期末の累積分布を凍結する。
                            // 凍結 Differencesから下位パーセンタイルの
                            // 時間発展が取れる。
                            self.dbg_h_sn += 1;
                            if self.dbg_h_sn >= 4_000 {
                                self.dbg_h_sn = 0;
                                if self.dbg_h_phase < self.dbg_h_nphase {
                                    self.dbg_h_snap[self.dbg_h_phase] = self.dbg_h_hist;
                                    self.dbg_h_phase += 1;
                                }
                            }
                        }
                    }
                    // 追跡キャリアの `|H|^2`。`[eqpos]` で `k=205/207/210`
                    // （中央の null 候補）と `k=428..431`（両端外挿）が上位
                    // だった。対照に中央の `k=100` を入れる。
                    //
                    // ここでの `h[l]` は `estimate_channel` の**補間済み** `H`
                    // なので、SP 位置の生値ではなく「等化器が見ている値」。
                    // null 深化なら `|H(k)|^2` が時間とともに単調に落ちる。
                    // 注意: ここは **実キャリア番号 `l`（0..432）** で判定する。
                    // `sp_k` は `sp_carriers` の通し番号（0..36）であって
                    // `[eqpos]` の 432 bin インデックスではない（実測ミス 3:
                    // `k=205` を `sp_k` と比較して常に 0 になっていた）。
                    // `[eqpos]` の `k=205/207/210` は `l` そのもの。
                    const TRACK: [(usize, usize); 8] = [
                        (100, 0), // 対照: 中央健全キャリア
                        (205, 1), // null 候補
                        (207, 2), // null 候補
                        (210, 3), // null 候補
                        (428, 4), // 両端（外挿）
                        (429, 5),
                        (430, 6),
                        (431, 7),
                    ];
                    for (tk, slot) in TRACK.iter() {
                        if l == *tk {
                            self.dbg_track_h2[*slot] += raw2 / (pval * pval);
                        }
                    }
                    // 窓を区切る。`sp_carriers` は 36 本なので 400 サンプルで
                    // 十分な平均になる。
                    self.dbg_track_n += 1;
                    if self.dbg_track_n >= 400 {
                        for t in self.dbg_track_h2.iter_mut() {
                            *t = 0.0;
                        }
                        self.dbg_track_n = 0;
                    }
                    // 円平均の引数（未加权の位相平均）。3 回以上回ると
                    // `-PI..PI` に wrap するので累積看不出来になる。
                    // unwrap して累積回転を出す。
                    let ang = wi.atan2(wr);
                    if self.dbg_sp_arg_n == 0 {
                        self.dbg_sp_arg = ang;
                    } else {
                        // 前回との差を `[-PI, PI]` に丸めて足す（unwrap）。
                        let mut d = ang - self.dbg_sp_arg;
                        let two_pi = std::f64::consts::TAU;
                        while d > std::f64::consts::PI { d -= two_pi; }
                        while d < -std::f64::consts::PI { d += two_pi; }
                        self.dbg_sp_arg += d;
                    }
                    self.dbg_sp_arg_n += 1;
                }
                let ph4 = sym_mod4 % 4;
                self.dbg_segsp_ph[ph4] += raw2;
                self.dbg_segsp_ph_n[ph4] += 1;
                let ysp = seg[l] / self.pilots.values[l];
                let ysp2 = (ysp.re as f64) * (ysp.re as f64) + (ysp.im as f64) * (ysp.im as f64);
                // `raw2` = |Y|^2（受信 SP の電力）、`ysp2` = |Y/P|^2 = |H|^2。
                //
                // 旧実装は `dbg_hsp_win += ysp2` としており、**両方に同じ値を
                // 入れていた**。そのため `dbg_eq_sp()` が返す
                // `(Ysp, Hsp)` は全サンプルの行で完全に一致し（実測
                // 0.9994/0.9994、0.9968/0.9968…）、物理的にありえない
                // 「SP の受信電力 == 推定チャネル」になっていた。
                // `Ysp` と `Hsp` の**比**が等化器の Outer 伸長に相当する
                // 量であり、このバグで常に 1.0 になっていた。
                //
                // SP 値は |P| = 4/3 なので |H| = |Y| / (4/3) = |Y| * 3/4、
                // したがって |H|^2 = |Y|^2 * 9/16 = `ysp2`（= |Y|^2 / (4/3)^2）。
                // `ysp2` 本身就��� |H|^2 なので}Hsp には `ysp2`、Ysp には `raw2`。
                self.dbg_ysp_win += raw2;
                self.dbg_hsp_win += ysp2;
                self.dbg_spn += 1;
                if self.dbg_spn >= 400 {
                    self.dbg_ysp_win = 0.0;
                    self.dbg_hsp_win = 0.0;
                    self.dbg_segsp_win = 0.0;
                    self.dbg_spn = 0;
                    for t in self.dbg_segsp_ph.iter_mut() {
                        *t = 0.0;
                    }
                    for t in self.dbg_segsp_ph_n.iter_mut() {
                        *t = 0;
                    }
                    self.dbg_sp_cisum = [0.0; 2];
                    self.dbg_sp_cin = 0;
                    self.dbg_sp_incoh = 0;
                    self.dbg_sp_tot = 0;
                    self.dbg_sp_arg = 0.0;
                    self.dbg_sp_arg_n = 0;
                }
            }
            for (i, v) in data.iter().enumerate() {
                let p2 = (v.re as f64) * (v.re as f64) + (v.im as f64) * (v.im as f64);
                self.dbg_eq_n += 1;
                if p2 > th {
                    self.dbg_eq_hot += 1;
                    // 位置は**実キャリア番号 `l`** で記録する。
                    // 旧実装は `i * 432 / n` で 432 等分していたため
                    // 4 キャリアずつまとめられ、null の位置を特定できな
                    // かった（実測ミス 3）。
                    let l_real = eq_idx[i];
                    if l_real < 432 {
                        self.dbg_hot_l[l_real] += 1;
                    }
                    // 旧 bin（診断用に残す）
                    let b = (i * 432 / n.max(1)).min(431);
                    self.dbg_eq_hot_pos[b] += 1;
                }
                // 追跡キャリアの**補間後** `|h[l]|^2`。
                //
                // 追跡対象（`[hotl]` で発散が集中した実キャリア）:
                //  - `l=204`  SP 実測値（`l%12==0`）
                //  - `l=205`  補間データキャリア（`l%12==1`）
                //  - `l=216`  補間に使う**もう片方の** SP（`l%12==0`）
                //  - `l=100`  対照の健全 SP
                //  - `l=108`,`l=96`  対照 SP（補間区間の他の点）
                //
                // `l=204` と `l=216` を同時に見ることで、補間の両端が
                // 健全かを判定できる。
                // ここでは**非 SP のデータキャリアだけ**を測る。
                // `l=205` は `l mod 12 == 1` なので 4 位相すべてで SP ではない
                // が、`l=204/216/96/108` は位相によって SP になるため
                // ここでは測らない（SP 側は上の `TRK_SP` で実測する）。
                //
                // slot 1 = `l=205`（補間データキャリア、判定対象）
                // slot 3 = `l=100`（`l%12==4` なので常に非 SP、対照）
                const TRK: [(usize, usize); 2] = [(205, 1), (100, 3)];
                let li = eq_idx[i];
                for (tl, slot) in TRK.iter() {
                    if li == *tl {
                        let hv = h[li];
                        self.dbg_trk_h[*slot] +=
                            (hv.re as f64) * (hv.re as f64) + (hv.im as f64) * (hv.im as f64);
                        let sv = seg[li];
                        self.dbg_trk_raw[*slot] +=
                            (sv.re as f64) * (sv.re as f64) + (sv.im as f64) * (sv.im as f64);
                        self.dbg_trk_n += 1;
                        self.dbg_trk_cnt[*slot] += 1;
                    }
                }
                // 窓を切る。SP は 4 シンボル周期、データは毎シンボルなので
                // 共同の分母では安定しない。**スロットごとに** 800 回で切る
                // （SP は 200 シンボル、データは 800 シンボルの平均）。
                // **スロットごと**に窓を切る。SP は 4 シンボル周期で
                // 観測頻度が違うため、全部で 1 つの窓だと分母が狂う。
                for t in 0..6 {
                    if self.dbg_trk_cnt[t] >= 800 {
                        self.dbg_trk_cnt[t] = 0;
                        self.dbg_trk_raw[t] = 0.0;
                        self.dbg_trk_h[t] = 0.0;
                    }
                }
                let a = 0.01f64;
                self.dbg_eq_max_ema = (1.0 - a) * self.dbg_eq_max_ema + a * p2;
                // 分解: `EQ = Y/H` なので、発散の原因は `|H|`（分母）が小さいのか
                // `|Y|`（分子）が大きいのかを切り分ける。
                //
                // `data[l] = Y(l) / H(l)` なので `|Y(l)| = |data(l)| * |H(l)|`。
                // 逆算することで demod 段の値を直接測れる。
                let hl = h[eq_idx[i]];
                let h2 = (hl.re as f64) * (hl.re as f64) + (hl.im as f64) * (hl.im as f64);
                let y2 = p2 * h2; // = |Y|^2
                self.dbg_y2_all += y2;
                self.dbg_y2_n += 1;
                self.dbg_h2_all += h2;
                self.dbg_y2_win += y2;
                self.dbg_h2_win += h2;
                self.dbg_win_n += 1;
                // 区間平均は 20000 サンプルごとに作り直す。累積平均は
                // 199 秒的变化が 1/1000 になって見えないため。
                if self.dbg_win_n >= 20_000 {
                    self.dbg_y2_win = 0.0;
                    self.dbg_h2_win = 0.0;
                    self.dbg_win_n = 0;
                }
                // SP 位置の分子・分母（推定品質の実測）。

            }
        }
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
        let mut trace_soft: Vec<f32> = Vec::new();
        for v in &td {
            let de = self.bdi.push(qpsk_soft(*v)); // [lsb, msb]
            if self.trace.is_some() {
                trace_soft.push(de[0]);
                trace_soft.push(de[1]);
            }
            // 診断: ソフト出力の自信度。理想は ±1.0 で、1.0 から離れるほど
            // 復調の確信度が下がる。Viterbi より**前**の入力なので、ここが
            // 崩れていれば Viterbi の状態は正常でも出力ビットが間違う。
            let abs_sum = (de[0].abs() + de[1].abs()) as f64;
            self.dbg_soft_abs = abs_sum * 0.5;
            let a = 0.0005f64; // 1/2000 の指数移動平均
            self.dbg_soft_abs_avg = (1.0 - a) * self.dbg_soft_abs_avg + a * self.dbg_soft_abs;
            if self.dbg_soft_abs_avg > 0.01 && self.dbg_soft_abs < self.dbg_soft_abs_avg * 0.5 {
                self.dbg_soft_degraded += 1;
                // 診断: 1 RS ブロック（204 バイト = 1 TSD）あたりの degraded 数を
                // ヒストグラムにまとめる。平均だけでは Aguas Forte が見えない。
                // 平均が動かないまま「8 個以上」（RS 訂正限界 t=8 に相当）の
                // ブロック占比だけが増えていれば、バースト性（クラスタリング）が
                // 原因だと分かる。
                self.degraded_in_block += 1;
                if self.degraded_in_block >= TSP {
                    let b = (self.degraded_in_block / 256).min(15);
                    self.dbg_degraded_hist[b] += 1;
                    self.dbg_degraded_blocks += 1;
                    self.degraded_in_block = 0;
                }
            }
            for kept in [de[1], de[0]] {
                if self.dump_enabled {
                    // 直近 1 ブロック分だけ保持する固定長リング。
                    // 4M まで溜めると切り出しが意味を失う（実測）。
                    if self.dump_soft.len() >= DUMP_RING {
                        self.dump_soft.drain(0..DUMP_RING / 4);
                    }
                    // 生の f32 値を保存する（量子化すると情報が失われ、
                    // オフライン再生で原因を判別できなくなる）。
                    // ここでは捕捉**しない**。`kept` は depuncture の
                    // **前**の値で、Viterbi が受け取る `mother`（後）とは
                    // 長さも内容が不一样（実測: kept=60000 に対し
                    // mother=45000）。Viterbi と同一の値を取るには
                    // `self.mother.push(...)` の直後に捕捉する。
                    //
                }
                // order=1
                loop {
                    let pat = PUNCTURE_2_3[self.depu_pos % PUNCTURE_2_3.len()];
                    self.depu_pos += 1;
                    if pat == 1 {
                        self.mother.push(kept);
                        if self.dump_enabled {
                            self.dump_soft_f32.push(kept);
                        }
                        break;
                    } else {
                        self.mother.push(0.0);
                        if self.dump_enabled {
                            // punctured = erasure 0.0。Viterbi に渡す値と同一。
                            self.dump_soft_f32.push(0.0);
                        }
                    }
                }
            }
        }
        // 診断: トレース有効時、このシンボルの各段の生値を控える。
        if let Some(tr) = self.trace.as_mut() {
            tr.syms.push(crate::trace::SymTrace {
                sym_idx: self.syms_done as u64,
                sym_start: 0,
                win_lo: 0,
                win_hi: 0,
                est_symbol_start: 0,
                est_metric: 0.0,
                sym_mod4,
                base: 0,
                sp_hat: trace_sp,
                eq_data: data.clone(),
                freq_deint: fd.clone(),
                time_deint: td.to_vec(),
                soft: trace_soft,
                mother: self.mother.clone(),
            });
            // メモリ無限増を避ける。RS ブロック 1 個ぶんのシンボルだけ保持する。
            // 1 TSD = 1 RS ブロック = 204 バイト ≈ 256 シンボル。
            let cap = 300;
            if tr.syms.len() > cap {
                let over = tr.syms.len() - cap;
                tr.syms.drain(0..over);
            }
        }
        let pairs = self.mother.len() / 2;
        for t in 0..pairs {
            let bit = match self.vit.push(self.mother[2 * t], self.mother[2 * t + 1]) {
                Some(b) => b,
                None => continue,
            };
            if self.dump_enabled {
                if self.dump_vit.len() >= RING_STEPS {
                    let ov = self.dump_vit.len() - RING_STEPS;
                    self.dump_vit.drain(0..ov);
                }
                self.dump_vit.push(bit & 1);
            }
            self.vit_bits_out += 1;
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
            // 診断: RS へ入ったバイトの**絶対連番**。原子スナップショットで
            // 「この 204 バイトの出所」を厳密に特定するための基準点にする。
            self.rs_byte_abs += 1;
            if self.dump_enabled {
                if self.rs_byte_vitpos.len() < 2_000_000 {
                    self.rs_byte_vitpos.push(self.vit_bits_out);
                }
                // このバイトの出所 vit ビット位置（Viterbi 出力の絶対連番）。
                // 1 バイト = 8 ビットなので `bit_acc` の繰り越しを無視して
                // 累(add) カウンタで管理する。
            }
            if self.dump_enabled {
                // このバイトを消費した時点の soft / vit カーソル。
                // 204 バイトが揃ったとき、この範囲が**そのブロックの**
                // soft / vit になる。
                // 固定長リング: 1 RS ブロック分の soft にちょうど
                // なる長さだけ保持する。204 バイト = 1632 情報ビット =
                // 1728 トレリスステップ = **3456 soft 値**。
                // この長さのスライディングウィンドウなら、ブロック完了
                // 時のリング全体が**そのブロックの soft** そのものになる。
                // **各リングを独立に**トリムする。
                // 共有 drain は soft（vit の約 3 倍の速度で増える）を
                // vit と同じ量だけ削り、vit が毎回ほぼ空になっていた（実測:
                // soft=24000 に対し vit=600）。これがオフライン再現不能の
                // 直接原因だった。
                if self.dump_soft_f32.len() > VIT_RING_SOFT {
                    let over = self.dump_soft_f32.len() - VIT_RING_SOFT;
                    self.dump_soft_f32.drain(0..over);
                }
            }
            // 診断: トレース有効時は RS へ入る直前のバイト列を蓄える。
            // 訂正不能ブロックは `rs_asm` から返らない（= 出力が消える）ので、
            // 手元で保持しておかないと「何が RS に入ったか」を後から検証できない。
            if let Some(tr) = self.trace.as_mut() {
                tr.bytes.push(o);
            }
            // 診断: RS ブロック完成時点の depu_pos を記録する。
            // 1 OFDM フレーム = 64 RS ブロックなので、dep u_pos は 64 ブロック
            // ごとに同じ mod PUNCTURE_2_3.len() に戻어야する（ 프레整列）。
            // 戻らなくなれば depuncture の位相がスリップしている。
            // 診断: 復号結果に**かかわらず**全ブロックで depolar 位相を記録する。
            //
            // 既存の `dbg_depu_at_block` は成功ブロックのみ（`for cw in blocks`
            // の内側）に push していたため、drop が 100% になると更新が-stop
            // する。すると「depu は 0 のまま = 整列は正常」という**誤った結論**
            // に到達する（実測でこれに当たった）。劣化域の判定には必ずこちらを使う。
            if dbg_depu_all_enabled() {
                self.dbg_depu_all
                    .push((self.rs_asm.rs_blocks_seen as usize, self.depu_pos % 4));
            }
            // `deint_out_idx` は RS へ出たバイト数の累積。
            // あるバイトが RS に入ったとき、その出所の vit ビットは
            // 現在の vit 位置から (BYTE_LATENCY + block_phase) * 8 だけ前。
            // この量を引いてスナップショットすれば、その 204 バイトの
            // **出所**の vit 区間が得られる。
            let _seen_before = self.rs_asm.rs_blocks_seen;
            let (blocks, burst) = self.rs_asm.feed(&[o]);
            // 原子的な 1 ブロック分の記録。
            //
            // `dump_map` は「バイトを 1 個消費した時点の soft/vit カーソル」の
            // 配列。204 バイトが揃った = アセンブラがブロックを完成させた、
            // 那一刻に、その 204 エントリで囲まれた範囲が**そのブロックの**
            // soft / vit になる。3 つの記録を別々に切り出さずに済む。
            // アセンブラ側でこのブロックの `dump_byte` が取られた直後。
            if self.dump_enabled
                && self.rs_asm.rs_blocks_seen == self.rs_asm.dump_target
                && self.dump_soft_f32.len() >= VIT_RING_SOFT
                && self.dump_vit.len() >= RING_STEPS - 128
                && !self.rs_asm.dump_byte.is_empty()
            {
                // このブロックの 204 バイトは `rs_byte_vitpos` の
                // 直近 204 エントリに対応する。その**先頭**の vit 位置を
                // lo とし、lo から 1632 ビットを取り出す。
                //
                // リングは生きたままなので lo は絶対位置。リング長を
                // 引いた相対位置に変換してから切り出す。
                let n_v = self.rs_byte_vitpos.len();
                if n_v >= rs::N {
                    let v_lo_abs = self.rs_byte_vitpos[n_v - rs::N];
                    let ring = self.dump_vit.len() as u64;
                    // リングは「現在位置まで」のみ保持。v_lo_abs が
                    // リング外なら、そのブロックは保持範囲より古い。
                    if v_lo_abs + (rs::N * 8) as u64 >= self.vit_bits_out {
                        let cur = self.vit_bits_out;
                        let start = v_lo_abs;
                        let s_rel = (cur - start) as usize;   // リング先頭からの距離
                        if s_rel < self.dump_vit.len() {
                            let end = self.dump_vit.len();
                            let atom_vit: Vec<u8> =
                                self.dump_vit[s_rel..end].to_vec();
                            // soft は vit 出力 1 ビット = 1 ステップ = 2 値。
                            // 開始ステップ = (cur - ring) + s_rel ... だが
                            //  Easier: soft の開始 = vit の位置 * 2 相当。
                            let atom_soft: Vec<f32> = self.dump_soft_f32.clone();
                            // 的瞬间の deinterleaver idx（絶対）。
                            self.dump_deint_idx = self.bdeint.idx();
                            self.dump_atom = Some((
                                atom_soft,
                                atom_vit,
                                self.rs_asm.dump_byte.clone(),
                            ));
                        }
                    }
                }
                let (ns, nv) = (self.dump_soft_f32.len(), self.dump_vit.len());
                eprintln!(
                    "[atom] block={} soft={}B vit={}b byte={}B atom={}",
                    self.rs_asm.rs_blocks_seen, ns, nv, self.rs_asm.dump_byte.len(),
                    self.dump_atom.is_some()
                );
                eprintln!(
                    "[atom] block={} soft={}B vit={}b byte={}B",
                    self.rs_asm.rs_blocks_seen, ns, nv, self.rs_asm.dump_byte.len()
                );
            }
            // 診断: `rs_asm` は訂正不能のブロックを返さない（= TS から消える）。
            // 消えたブロックは手元に残した生バイト列でしか追えないので、
            // ここで「消えた」ことを記録し、トレース対象ならダンプする。
            if blocks.is_empty() {
                self.rs_lost += 1;
                if let Some(tr) = self.trace.as_mut() {
                    if tr.wants(self.nblk as u64, None) && tr.bytes.len() >= rs::N {
                        tr.blk = self.nblk as u64;
                        tr.rs_uncorrectable = true;
                        // サマリだけ先頭に出す。バイト列は最後に 64 バイトだけ。
                        let tail: Vec<u8> = tr
                            .bytes
                            .iter()
                            .skip(tr.bytes.len().saturating_sub(64))
                            .copied()
                            .collect();
                        eprint!(
                            "{}",
                            crate::trace::BlockTrace {
                                bytes: tail,
                                ..tr.clone()
                            }
                            .render()
                        );
                        tr.bytes.clear();
                    }
                }
            }
            for cw in blocks {
                // 診断: RS ブロック完成時点の depu_pos。
                // 1 OFDM フレーム = 64 RS ブロックなので、`depu_pos` は 64 ブロック
                // ごとに同じ `mod PUNCTURE_2_3.len()` に戻아야する（フレーム整列）。
                // 戻らなくなれば depuncture の位相がスリップしている。
                self.dbg_depu_at_block.push((self.nblk, self.depu_pos % PUNCTURE_2_3.len()));
                if self.dbg_depu_at_block.len() > 2048 {
                    self.dbg_depu_at_block.remove(0);
                }
                // 診断: トレース対象のブロックなら、ここまでの全段をダンプする。
                if let Some(tr) = self.trace.as_mut() {
                    let nblk_u = self.nblk as u64;
                    if tr.wants(nblk_u, None) {
                        tr.blk = nblk_u;
                        tr.rs = Some(cw.to_vec());
                        tr.rs_uncorrectable = false;
                        eprint!("{}", tr.render());
                        tr.bytes.clear();
                    }
                }
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

        // 診断: TMCC フレーム境界（1 ISDB-T フレーム = 204 シンボル）の
        // RS ブロック位置を記録する。
        //
        // gr-isdbt `tmcc_decoder_1seg_impl.cc` は PRBS リセットの基準を
        // **TMCC 同期語が現れる位置**（`d_frame_end`）から作る。我々は
        // RS ブロック番号の 64 周期でリセットしており、この 2 つが
        // 一致する保証はない。TMCC 境界と 64 周期がずれていれば、
        // 後半で PRBS 位相がずれて RS が壊れる（観測された症状）。
        // 段別ダンプ: 対象ブロックの soft / vit を对齐して切り出す。
        if self.rs_asm.dump_enabled && self.rs_asm.rs_blocks_seen == self.rs_asm.dump_target {
            // Pipe のリングから対象ブロック分だけを取り出す。
            let s_need = rs::N * 8 * 3 / 2;
            let v_need = rs::N * 8;
            let so = self.dump_soft_f32.len().saturating_sub(s_need);
            let vo = self.dump_vit.len().saturating_sub(v_need);
            self.dump_soft_f32 = self.dump_soft_f32[so..].to_vec();
            self.dump_vit = self.dump_vit[vo..].to_vec();
        }
        self.tmcc_sym_count += 1;
        if self.tmcc_sym_count >= TMCC_SYMS_PER_FRAME {
            self.tmcc_sym_count = 0;
            self.tmcc_frame_pos.push(self.rs_asm.rs_blocks_seen as usize);
        }

        // 診断: 1 シンボルの所要時間を集計する。CPU 律速（熱スロットリング含む）
        // なら時間がブロック数に比例して伸びる。復調品質低下ならフラット。
        let dt = _t_start.elapsed().as_micros() as u64;
        self.t_sum_us += dt;
        self.t_n += 1;
        if dt > self.t_max_us {
            self.t_max_us = dt;
        }
    }
}

/// 逐次ストリーミング復調器。`feed` にIQのu8バイトを渡すと、生成されたTSバイトを返す。
pub struct StreamingDecoder {
    demod: OfdmDemod,
    /// ロック時に `demod_and_align` が採用した整列パラメータと評価値。
    ///
    /// `(commutator, reset_off, block_phase, 評価窓のRS復.decode率)`。
    /// 診断時に「93%」と表示されても、これは**ロック判定の短い窓**での値であり、
    /// 連続復調の実運用の性能とは一致しない（実測で 20 倍乖離した）。
    lock_claim: Option<(usize, usize, usize, f32)>,
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
    pub reacq_count: usize,
    /// 診断: 強制再ロックの実行回数。
    pub relock_count: usize,
    /// 再取得試行のログ出力カウンタ（診断用）。
    reacq_log: usize,
    /// 診断: `reacquire` が選んだ `base` のジャンプ履歴。
    ///
    /// 「reacquire が徐々に間違った答えを選ぶ確率が上がっている」なら
    /// 大きくジャンプした回数が blk 数とともに増える。ブロック番号と
    /// ジャンプ量を交互に詰める: `[blk0, jump0, blk1, jump1, ...]`。
    /// `jump` は `(値 + 1000) * 2` で符号化している。
    pub dbg_raq_jump: Vec<u32>,
    /// 前回 `reacquire` が選んだ `base`。
    pub dbg_raq_prev: usize,
    /// 診断: `reacquire` が `None`（peak なし）を返した回数。
    pub dbg_raq_none: u64,
    /// 診断: peak は見つかったが `|delta| >= sym/2` で却下された回数。
    pub dbg_raq_reject: u64,
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
    /// 境界の端数キャリー（サンプル）。[0, 1)。
    ///
    /// 実測 2026-09-27: これが無いと `demod_one_tracked_frac` は
    /// `next = start + sym`（厳密整数）で返すため端数が毎シンボル捨てられ、
    /// 探索の基準が常に整数に戻る。実 fs と指定 fs の差 2.5e-6 サンプル/
    /// シンボルが `next` に反映されず累積し、2000 シンボル（≒7 秒）で
    /// 探索半径 8 を超えて破綻する。`ISDBT_CARRY=1` で有効化。
    pub frac_carry: f32,
    /// 境界の**分数**オフセット（サンプル）。[-0.5, 0.5]。
    ///
    /// 実 fs（1015873.002204）と指定 fs（1015873）の差は 2.5e-6 サンプル/シンボル。
    /// 整数サンプル境界に丸め続けるとこの差が累積し、等化器のタップが合わず
    /// RS は 100% でも H.264 ビットが化ける。FFT 窓を線形補間でずらして吸収する。
    boundary_frac: f32,
    /// Pipe を再生成してからの OFDM フレーム数。
    frames_since_reset: usize,
    /// Pipe 再生成の累計回数（診断用）。
    pub pipe_resets: usize,
    /// SP 位相傾きから推定した残留サンプルタイミング誤差（EMA、-0.5..0.5）。
    timing_frac: f32,
    /// 複数フレームをまたいで SP チャネル推定を coherent 平均する。
    sp_acc: crate::equalize::SpGridAccumulator,
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
            lock_claim: None,
            buf: Vec::new(),
            pending: Vec::new(),
            dc: Complex32::new(0.0, 0.0),
            dc_done: false,
            cur: 0,
            k: 0,
            syms_since_reacq: 0,
            reacq_off: 0,
            reacq_count: 0,
            relock_count: 0,
            reacq_log: 0,
            dbg_raq_jump: Vec::new(),
            dbg_raq_prev: 0,
            dbg_raq_none: 0,
            dbg_raq_reject: 0,
            settle_syms: 0,
            settled: false,
            clean_run: 0,
            pending_discontinuity: false,
            disc_ticks: 0,
            disc_cc: 0,
            disc_count: 0,
            frac_carry: 0.0,
            boundary_frac: 0.0,
            frames_since_reset: 0,
            pipe_resets: 0,
            timing_frac: 0.0,
            sp_acc: crate::equalize::SpGridAccumulator::new(),
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

    /// RS 復号器に渡された**ブロック総数**（= 訂正成功 + 訂正不能）。
    ///
    /// 訂正不能率 `drop / (corrected + drop)` の分母は `rs_corrected + rs_dropped`
    /// だが、RS 復号器が実際に受け取ったブロック数の累積値も欲しい。
    /// `rs_corrected` は「synd が非ゼロだったブロック」なので、
    /// synd がゼロ（訂正不要）だったブロックを含まない。
    pub fn rs_blocks_seen(&self) -> u64 {
        self.pipe.as_ref().map_or(0, |p| p.rs_asm.rs_blocks_seen)
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

    /// `(推定 symbol 誤り数の総和, 総ブロック数)`。
    ///
    /// `rs_error_stats` の第 2 戻り値（`rs_bit_errors`）は 16 で頭打ちになる
    /// ため診断に使えない（`rs_bit_errors` のドキュメント参照）。
    /// これが RS の訂正能力 t=8 と直接比較できる唯一の指標。
    pub fn rs_symbol_error_stats(&self) -> (u64, u64) {
        self.pipe
            .as_ref()
            .map(|p| (p.rs_asm.rs_symbol_errors, p.rs_asm.rs_blocks_seen as u64))
            .unwrap_or((0, 0))
    }

    /// 復号失敗理由ごとの回数。`(多すぎる, 根数不一致, 分母0, 誤訂正)`。
    pub fn rs_fail_reasons(&self) -> [u64; 4] {
        self.pipe
            .as_ref()
            .map(|p| p.rs_asm.rs_fail_reasons)
            .unwrap_or([0; 4])
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

    /// DC オフセットの追従率（0 で追従しない = 旧挙動）。
    ///
    /// 実測 2026-09-26: `self.dc` は `try_lock()` で**一度だけ**推定され、
    /// 以後 50 秒間固定される。ハードウェアの DC ドリフト（温度/AGC 連動）を
    /// 追い跟不上、残差 DC が時間とともに増えて BER が劣化する疑いがある。
    /// 1 で常に追従、0.001 なら 1000 サンプルごとに 1 サンプル分を差し引く。
    fn dc_track() -> f32 {
        match std::env::var("ISDBT_DCTRACK") {
            Ok(v) => v.parse().unwrap_or(0.0),
            Err(_) => 0.0,
        }
    }

    fn append(&mut self, iq: &[u8]) {
        let mut bytes = core::mem::take(&mut self.pending);
        bytes.extend_from_slice(iq);
        let even = bytes.len() & !1;
        for c in u8_iq_to_complex(&bytes[..even]) {
            self.buf.push(if self.dc_done { c - self.dc } else { c });
        }
        // DC 追従（`ISDBT_DCTRACK > 0` のときだけ）。
        //
        // 直近 `TRACK_WIN` サンプルの平均で `self.dc` を.Null  slowly 仄仄更新する。
        // 累積平均だと 50 秒分を毎回走査するので、直近窓に限定する。
        if self.dc_done {
            let a = Self::dc_track();
            if a > 0.0 {
                const TRACK_WIN: usize = 8192;
                let start = self.buf.len().saturating_sub(TRACK_WIN);
                let win = &self.buf[start..];
                if win.len() >= TRACK_WIN {
                    let m = win.iter().sum::<Complex32>() / win.len() as f32;
                    // 既に `self.dc` を引いた値的平均なので、これを引く。
                    self.dc = self.dc * (1.0 - a) + m * a;
                    for v in self.buf[start..].iter_mut() {
                        *v -= m * a;
                    }
                }
            }
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
        let Some((mut seg_off, rs_rate, all)) = select_segment_offset_by_rs(&specs, &cands) else {
            eprintln!("offset 候補 ({cands:?}) すべてで RS 復号 0%: ロック失敗（品質不良）");
            return;
        };
        if seg_off != cands[0] {
            eprintln!(
                "bin offset TMCC 優先={} だが RS 復号率で {} を採用",
                cands[0], seg_off
            );
        }
        // 診断: `ISDBT_SEGOFF` で bin offset を強制固定する。
        //
        // 連続 capture は 307、後半単独 capture は 308 を自動選択した。
        // 1 bin = 992 Hz なので、**`l=207` を比べたのは 992 Hz ずれた
        // 周波数の profile だった可能性がある**。両者を同じ bin に固定して
        // 初めて「同じ `l` = 同じ物理キャリア」を主張できる。
        if let Ok(v) = std::env::var("ISDBT_SEGOFF") {
            if let Ok(f) = v.parse::<usize>() {
                eprintln!("ISDBT_SEGOFF={f} で bin offset を固定（自動選択の {seg_off} を無視）");
                seg_off = f;
            }
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
        // 診断: ロック判定が採用したパラメータと評価値を保存する。
        // 実運用の性能との乖離を調べるため。
        self.lock_claim = Some((commutator, reset_off, block_phase, rs_rate));
        self.locked = Some(Locked {
            gi: est.guard,
            cfo: est.cfo_subcarriers,
            sym,
            phase0,
            seg_off,
        });
        self.pipe = Some(Pipe::new(commutator, reset_off, block_phase, seg_off));
        // 診断: 実運用の RS 符号語率を数える（`ISDBT_RSVPROBE=1`）。
        // ロック判定の `RS復.decode率` との乖離を測るため。
        // 診断: 原子ダンプ（`ISDBT_ATOM=<blk>`）も dump を有効にする。
        if let Ok(b) = std::env::var("ISDBT_ATOM") {
            if let Ok(bb) = b.parse::<u64>() {
                if let Some(p) = self.pipe.as_mut() {
                    p.dump_target = Some(bb);
                    p.dump_enabled = true;
                    p.rs_asm.dump_target = bb;
                    p.rs_asm.dump_enabled = true;
                    eprintln!("[atom] 対象ブロック = {}", bb);
                }
            }
        }
        if std::env::var("ISDBT_RSVPROBE").is_ok() {
            if let Some(p) = self.pipe.as_mut() {
                p.rs_asm.rs_valid_probe = true;
            }
        }
        // `ISDBT_DUMP=<blk>` で段別バイトダンプを有効にする（診断用）。
        if let Ok(v) = std::env::var("ISDBT_DUMP") {
            if let Ok(b) = v.parse::<u64>() {
                if let Some(p) = self.pipe.as_mut() {
                    p.dump_target = Some(b);
                    p.rs_asm.dump_target = b;
                    p.rs_asm.dump_enabled = true;
                    p.rs_asm.dump_span = std::env::var("ISDBT_DUMPSPAN")
                        .ok()
                        .and_then(|v| v.parse().ok())
                        .unwrap_or(1);
                    p.dump_enabled = true;
                    eprintln!("[dump] 対象ブロック = {} (rs_blocks_seen 基準)", b);
                }
            }
        }
        // 診断: `ISDBT_TRACE=<blk>` で指定ブロック（と最初の訂正不能）の
        // 全段トレースを有効にする。
        if let Ok(v) = std::env::var("ISDBT_TRACE") {
            if let Ok(n) = v.parse::<u64>() {
                self.pipe.as_mut().unwrap().trace = Some(crate::trace::BlockTrace {
                    target_blk: n,
                    block_phase,
                    depu_pos_start: 0,
                    ..Default::default()
                });
                eprintln!("[trace] 有効: 対象ブロック {}（±4 と最初の訂正不能も対象）", n);
            }
        }
        self.frames_since_reset = 0;
        self.sp_acc.reset();
        self.timing_frac = 0.0;
        self.frac_carry = 0.0;
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

    /// 診断: ロック時に確定した整列パラメータ（起動時 1 回だけ決まる）。
    /// 後半のブロックで真の整列位置からずれていれば、ここが原因。
    pub fn dbg_align_params(&self) -> (usize, usize, usize) {
        self.pipe
            .as_ref()
            .map_or((usize::MAX, usize::MAX, usize::MAX), |p| {
                (p.commutator, p.reset_off, p.block_phase)
            })
    }

    /// 診断: 1 TSD あたりの degraded 数のヒストグラム（長さ 16）と総ブロック数。
    /// 診断: `(復号できたブロック数, そのうち先頭が 0x47 の数)`。
    pub fn dbg_rs_sync(&self) -> (u64, u64) {
        self.pipe
            .as_ref()
            .map(|p| (p.rs_asm.rs_seen_decoded, p.rs_asm.rs_sync_ok))
            .unwrap_or((0, 0))
    }

    pub fn dbg_degraded_hist(&self) -> ([u64; 16], u64) {
        self.pipe
            .as_ref()
            .map_or(([0u64; 16], 0), |p| (p.dbg_degraded_hist, p.dbg_degraded_blocks))
    }

    /// 診断: `qpsk_soft` の (総数, クランプ飽和数, 非有限数)。
    pub fn dbg_saturation(&self) -> (u64, u64, u64) { crate::demap::dbg_saturation() }

    /// 診断: Viterbi の (ステップ数, metric 乖離, 有限状態数, out キュー長)。
    pub fn dbg_viterbi(&self) -> (u64, f32, usize, usize) {
        self.pipe.as_ref().map_or((0, 0.0, 0, 0), |p| p.vit.dbg)
    }

    /// 診断: `self.dc`（ロック時固定値）の実部。
    pub fn dbg_dc_fixed(&self) -> f64 { self.dc.re as f64 }
    /// 診断: `self.dc`（ロック時固定値）の虚部。
    pub fn dbg_dc_fixed_im(&self) -> f64 { self.dc.im as f64 }
    /// 診断: 直近 8192 サンプルの局所 DC 平均（実部）。
    pub fn dbg_dc_local(&self) -> f64 {
        const W: usize = 8192;
        let s = self.buf.len().saturating_sub(W);
        if self.buf.len() < W { return 0.0; }
        (self.buf[s..].iter().map(|c| c.re as f64).sum::<f64>()) / W as f64
    }
    /// 診断: 直近 8192 サンプルの局所 DC 平均（虚部）。
    pub fn dbg_dc_local_im(&self) -> f64 {
        const W: usize = 8192;
        let s = self.buf.len().saturating_sub(W);
        if self.buf.len() < W { return 0.0; }
        (self.buf[s..].iter().map(|c| c.im as f64).sum::<f64>()) / W as f64
    }
    /// 診断: `self.cur`（絶対サンプルインデックス）。
    pub fn dbg_cur(&self) -> u64 { self.cur as u64 }
    /// 診断: (`|H|^2` 超過数, サンプル数, 位置ヒストグラム, max EMA)。
    /// 診断: (分子 `|Y|` の平均, 分母 `|H|` の平均, EQ の `|H|^2` 平均)。
    /// 区間平均の分子・分母（累積平均ではなく直近 20000 サンプルの窓）。
    /// unwrap 累積した円平均位相回転（ラジアン）。
    pub fn dbg_sp_arg_accum(&self) -> f64 {
        self.pipe.as_ref().map_or(0.0, |p| p.dbg_sp_arg)
    }
    /// 追跡キャリアの窓平均（`(等化前 |seg[l]|^2, 補間後 |h[l]|^2)` の 6 組）。
    /// 添字: 0=`l=204`, 1=`l=205`, 2=`l=216`, 3=`l=100`, 4=`l=96`, 5=`l=108`
    /// `|h[204]|` / `|h[216]|` / `|h[205]|` の累積対数ヒストグラム
    /// （slot 0/1/2）と、4 時点の凍結スナップショット。
    pub fn dbg_h_dist(&self) -> ([[u64; 45]; 4], [[[u64; 45]; 4]; 12]) {
        self.pipe.as_ref().map_or(([[0; 45]; 4], [[[0; 45]; 4]; 12]), |p| (p.dbg_h_hist, p.dbg_h_snap))
    }
    pub fn dbg_track_pairs(&self) -> [f64; 12] {
        self.pipe.as_ref().map_or([0.0; 12], |p| {
            let mut out = [0.0f64; 12];
            for t in 0..6 {
                // スロットごと���分母（SP と非 SP の観測頻度が違う）。
                let n = p.dbg_trk_cnt[t] as f64;
                if n > 0.0 {
                    out[t] = p.dbg_trk_raw[t] / n;   // 等化前
                    out[t + 6] = p.dbg_trk_h[t] / n; // 補間後
                }
            }
            out
        })
    }
    /// `reacquire` の `None`（peak なし）と却下回数、累積値。
    pub fn dbg_raq_miss(&self) -> (u64, u64) {
        (self.dbg_raq_none, self.dbg_raq_reject)
    }
    /// `reacquire` のジャンプ履歴 `[blk0, jump0, blk1, jump1, ...]`。
    /// `jump` は `(値 + 1000) * 2` で符号化。
    pub fn dbg_raq_history(&self) -> &[u32] {
        self.dbg_raq_jump.as_slice()
    }
    /// null profile の 11 キャリア時系列（ema）。
    /// 添字 0..10 = `l=195,198,201,204,207,210,213,216,219,222,225`
    /// 絶対 spectrum bin 502..=522 の `|seg[k]|²`（ema）。
    pub fn dbg_abs_profile(&self) -> [f64; 21] {
        self.pipe.as_ref().map_or([0.0; 21], |p| p.dbg_abs_ema)
    }
    pub fn dbg_profile(&self) -> [f64; 11] {
        self.pipe.as_ref().map_or([0.0; 11], |p| p.dbg_prof_ema)
    }
    /// 単一キャリアの 1 SP 巡あたりの増分（`[204,210,192,216,100,207]`）。
    pub fn dbg_ts_delta(&self) -> [f64; 6] {
        self.pipe.as_ref().map_or([0.0; 6], |p| p.dbg_ts_ema)
    }
    /// 前半・後半の SP 帯域像（前半の `l` ごとの平均と後半の `l` ごとの平均）。
    pub fn dbg_sp_band_split(&self) -> ([f64; 432], u64, [f64; 432], u64) {
        self.pipe.as_ref().map_or(([0.0; 432], 0, [0.0; 432], 0), |p| {
            let mut a = [0.0f64; 432];
            let mut b = [0.0f64; 432];
            // 母数は実観測数そのもの（`l` ごとに同じ回数だけ観測される）。
            let na = p.dbg_sp_band1_n as f64;
            let nb = p.dbg_sp_band2_n as f64;
            for t in 0..432 {
                a[t] = if na > 0.0 { p.dbg_sp_band1[t] / na } else { 0.0 };
                b[t] = if nb > 0.0 { p.dbg_sp_band2[t] / nb } else { 0.0 };
            }
            (a, p.dbg_sp_band1_n, b, p.dbg_sp_band2_n)
        })
    }
    /// SP 帯域像（`l` ごとの平均 `|seg[l]|^2`）。
    pub fn dbg_sp_band(&self) -> ([f64; 432], u64) {
        self.pipe.as_ref().map_or(([0.0; 432], 0), |p| {
            let n = p.dbg_sp_band_n as f64;
            if n <= 0.0 {
                return ([0.0; 432], 0);
            }
            let mut out = [0.0f64; 432];
            for t in 0..432 {
                out[t] = p.dbg_sp_band[t] / n;
            }
            (out, p.dbg_sp_band_n)
        })
    }
    /// `values[204]` の実行時ビットパターンと観測回数。
    pub fn dbg_pval204(&self) -> (u32, u64) {
        self.pipe.as_ref().map_or((0, 0), |p| (p.dbg_pval_bits, p.dbg_pval_n))
    }
    /// `values` 全 432 本のハッシュ。
    pub fn dbg_pval_hash(&self) -> u64 {
        self.pipe.as_ref().map_or(0, |p| p.dbg_pval_hash)
    }
    /// 発散した実キャリア番号 `l` の分布（432 実キャリア単位）。
    pub fn dbg_hot_carriers(&self) -> [u64; 432] {
        self.pipe.as_ref().map_or([0u64; 432], |p| p.dbg_hot_l)
    }
    /// 追跡キャリアの窓平均 `|H(k)|^2`（`k=100,205,207,210,428..431`）。
    pub fn dbg_track_h(&self) -> [f64; 8] {
        self.pipe.as_ref().map_or([0.0; 8], |p| {
            let n = p.dbg_track_n as f64;
            if n <= 0.0 {
                return [0.0; 8];
            }
            let mut out = [0.0f64; 8];
            for t in 0..8 {
                out[t] = p.dbg_track_h2[t] / n;
            }
            out
        })
    }
    /// SP 1 本単位の異常率（`[位相][k] = bad / seen`、`-1.0` は未観測）。
    pub fn dbg_sp_bad_rate(&self) -> [[f64; 36]; 4] {
        self.pipe.as_ref().map_or([[0.0; 36]; 4], |p| {
            let mut out = [[0.0f64; 36]; 4];
            for ph in 0..4 {
                for k in 0..36 {
                    if p.dbg_sp_seen_idx[ph][k] > 0 {
                        out[ph][k] =
                            p.dbg_sp_bad_idx[ph][k] as f64 / p.dbg_sp_seen_idx[ph][k] as f64;
                    } else {
                        out[ph][k] = -1.0;
                    }
                }
            }
            out
        })
    }
    /// incoherent SP の割合（0.0-1.0）。36 本のうち何本が `|1-w| > 0.5`。
    pub fn dbg_sp_incoh_frac(&self) -> f64 {
        self.pipe.as_ref().map_or(0.0, |p| {
            if p.dbg_sp_tot == 0 {
                0.0
            } else {
                p.dbg_sp_incoh as f64 / p.dbg_sp_tot as f64
            }
        })
    }
    /// SP 位相の coherence (`|CI|`, 1.0 = 完全 coherent)。
    /// 0.0 → ランダム散乱、1.0 → 全 SP が同一の余剰位相。
    pub fn dbg_sp_coherence(&self) -> f64 {
        self.pipe.as_ref().map_or(0.0, |p| {
            if p.dbg_sp_cin == 0 {
                return 0.0;
            }
            let n = p.dbg_sp_cin as f64;
            let re = p.dbg_sp_cisum[0] / n;
            let im = p.dbg_sp_cisum[1] / n;
            (re * re + im * im).sqrt()
        })
    }
    /// 位相別（0/1/2/3）の**等化前** `|seg[l]|` 窓平均。
    pub fn dbg_seg_sp_phase(&self) -> [f64; 4] {
        self.pipe.as_ref().map_or([0.0; 4], |p| {
            let mut out = [0.0f64; 4];
            for t in 0..4 {
                if p.dbg_segsp_ph_n[t] > 0 {
                    out[t] = (p.dbg_segsp_ph[t] / p.dbg_segsp_ph_n[t] as f64).sqrt();
                }
            }
            out
        })
    }
    /// SP 位置の**等化前** `|seg[l]|` の窓平均。
    pub fn dbg_seg_sp(&self) -> f64 {
        self.pipe.as_ref().map_or(0.0, |p| {
            if p.dbg_spn == 0 {
                0.0
            } else {
                (p.dbg_segsp_win / p.dbg_spn as f64).sqrt()
            }
        })
    }
    /// SP 位置だけの窓平均 `(|Y_SP|, |H_SP|)`。
    pub fn dbg_eq_sp(&self) -> (f64, f64) {
        self.pipe.as_ref().map_or((0.0, 0.0), |p| {
            if p.dbg_spn == 0 {
                return (0.0, 0.0);
            }
            let n = p.dbg_spn as f64;
            ((p.dbg_ysp_win / n).sqrt(), (p.dbg_hsp_win / n).sqrt())
        })
    }
    pub fn dbg_eq_win(&self) -> (f64, f64, f64, f64) {
        self.pipe.as_ref().map_or((0.0, 0.0, 0.0, 0.0), |p| {
            let n = p.dbg_win_n as f64;
            if n <= 0.0 {
                return (0.0, 0.0, 0.0, 0.0);
            }
            let h2 = p.dbg_h2_win / n;
            let y2 = p.dbg_y2_win / n;
            (y2.sqrt(), h2.sqrt(), h2, y2)
        })
    }
    pub fn dbg_eq_split(&self) -> (f64, f64, f64, f64) {
        self.pipe.as_ref().map_or((0.0, 0.0, 0.0, 0.0), |p| {
            let n = p.dbg_y2_n as f64;
            let h2 = if n > 0.0 { p.dbg_h2_all / n } else { 0.0 };
            let y2 = if n > 0.0 { p.dbg_y2_all / n } else { 0.0 };
            (y2.sqrt(), h2.sqrt(), h2, y2)
        })
    }
    pub fn dbg_eq_hot(&self) -> (u64, u64, [u64; 432], f64) {
        self.pipe.as_ref().map_or((0, 0, [0u64; 432], 0.0), |p| {
            (p.dbg_eq_hot, p.dbg_eq_n, p.dbg_eq_hot_pos, p.dbg_eq_max_ema)
        })
    }
    /// 診断: (`boundary_frac`, `timing_frac`, `frac_carry`)。
    pub fn dbg_boundary_frac(&self) -> (f32, f32, f32) {
        (self.boundary_frac, self.timing_frac, self.frac_carry)
    }
    /// 診断: `cur` を 1 シンボル長で割った余り。FFT 窓境界が一貫していれば常に 0。
    pub fn dbg_cur_rem(&self) -> u64 {
        match self.locked {
            Some(lk) => self.cur as u64 % lk.sym as u64,
            None => 0,
        }
    }
    /// 診断: `cur` を 204 シンボル（1 OFDM フレーム）で割った余り。
    /// frame 境界の drift 検出用。
    pub fn dbg_cur_frame_rem(&self) -> u64 {
        match self.locked {
            Some(lk) => {
                let fr = 204 * lk.sym as u64;
                self.cur as u64 % fr
            }
            None => 0,
        }
    }
    /// 診断: `self.buf.len()`（バッファ長）。
    pub fn dbg_buf_len(&self) -> u64 { self.buf.len() as u64 }
    /// 診断: 全ブロックの `(ブロック番号, depu_pos % 4)`。
    /// 段別バイトダンプ `(soft量子化ビット, Viterbi出力ビット, byte整列後バイト)`。
    /// 連続する複数の RS 入力ブロック（descramble 後）。
    /// 実運用で RS 符号語だったブロック数 / 総数。
    pub fn dbg_rs_valid(&self) -> (u64, u64) {
        self.pipe
            .as_ref()
            .map(|p| (p.rs_asm.rs_valid, p.rs_asm.rs_seen))
            .unwrap_or((0, 0))
    }

    /// ロック時に `demod_and_align` が採用した整列パラメータと、
    /// その**評価窓**で claimed された RS 復.decode 率。
    ///
    /// 実運用の性能とは別物。診断時に「93%」と出ていても、
    /// 連続復調では 5% しか.decode できていない **/
    pub fn dbg_lock_claim(&self) -> Option<(usize, usize, usize, f32)> {
        self.lock_claim
    }

    pub fn dump_blocks(&self) -> Vec<Vec<u8>> {
        self.pipe
            .as_ref()
            .map(|p| p.rs_asm.dump_byte_all.clone())
            .unwrap_or_default()
    }

    /// descramble 前（`buffer` 由来）の生バイト。
    pub fn dump_pre(&self) -> Vec<u8> {
        self.pipe
            .as_ref()
            .map(|p| p.rs_asm.dump_byte_pre.clone())
            .unwrap_or_default()
    }

    /// 原子スナップショット時の `ByteDeinterleaver::idx`。
    /// offline 再生で同じ出力列を得るには、この値まで idx を進めてから
    /// push する（`push` が `idx % BI_I` で分岐を選ぶため）。
    pub fn dump_deint_idx(&self) -> usize {
        self.pipe.as_ref().map(|p| p.dump_deint_idx).unwrap_or(0)
    }

    /// **原子的に**取り出した 1 ブロック分 `(soft f32, vit bits, RS 入力 204B)`。
    ///
    /// 3 つの配列が**同一ブロック由来**であることを保証する。
    pub fn dump_atom(&self) -> Option<(Vec<f32>, Vec<u8>, Vec<u8>)> {
        self.pipe.as_ref().and_then(|p| p.dump_atom.clone())
    }

    /// 生の f32 soft 値（量子化前）。
    pub fn dump_soft_f32(&self) -> Vec<f32> {
        self.pipe.as_ref().map(|p| p.dump_soft_f32.clone()).unwrap_or_default()
    }

    pub fn dump_stages(&self) -> (Vec<u8>, Vec<u8>, Vec<u8>) {
        self.pipe
            .as_ref()
            .map(|p| (p.dump_soft.clone(), p.dump_vit.clone(), p.rs_asm.dump_byte.clone()))
            .unwrap_or((vec![], vec![], vec![]))
    }

    /// PRBS リセットが起きた block_idx の列。
    pub fn dbg_prbs_resets(&self) -> Vec<usize> {
        self.pipe
            .as_ref()
            .map(|p| p.rs_asm.prbs_reset_log.clone())
            .unwrap_or_default()
    }

    /// TMCC フレーム境界の RS ブロック位置。
    pub fn dbg_tmcc_frame_pos(&self) -> Vec<usize> {
        self.pipe.as_ref().map(|p| p.tmcc_frame_pos.clone()).unwrap_or_default()
    }

    pub fn dbg_depu_all(&self) -> Vec<(usize, usize)> {
        self.pipe.as_ref().map(|p| p.dbg_depu_all.clone()).unwrap_or_default()
    }

    /// 診断: 直近 2048 個の (完了ブロック数, `depu_pos % 4`)。
    /// 診断: `(連続訂正不能長の分布, 最長バースト)`。
    pub fn dbg_drop_burst(&self) -> ([u64; 64], u64) {
        self.pipe
            .as_ref()
            .map(|p| (p.rs_asm.drop_burst_hist, p.rs_asm.drop_burst_max))
            .unwrap_or(([0; 64], 0))
    }

    pub fn dbg_depu_phase(&self) -> Vec<(usize, usize)> {
        self.pipe.as_ref().map(|p| p.dbg_depu_at_block.clone()).unwrap_or_default()
    }

    /// 診断: 落下 block の `block_idx % 256` 分布（長さ 256）と合計数。
    pub fn dbg_drop_pos(&self) -> ([u32; 256], usize) {
        self.pipe
            .as_ref()
            .map_or(([0u32; 256], 0), |p| (p.rs_asm.drop_by_mod256, p.rs_asm.drop_seen_blocks))
    }

    /// 診断: (処理シンボル数, 処理時間合計 μs, 最大 μs)。
    ///
    /// CPU 律速（熱スロットリング含む）か復調品質低下かの切り分け用。
    /// 実時間比が 0.267（1 シンボル 3305 μs）なので、1 シンボルがこれ远超えたら
    /// 律速，否则复调侧。
    pub fn dbg_timing(&self) -> (usize, u64, u64) {
        self.pipe.as_ref().map_or((0, 0, 0), |p| (p.t_n, p.t_sum_us, p.t_max_us))
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
        self.process(max_syms_per_call())
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
                // 診断: 強制再ロック（`ISDBT_RELOCK=N` シンボルごと）。
                // `lk` と `Pipe` を丸ごと作り直す。 Pipe::resync() と同じ完全
                // リセットだが、境界の基準（`cur` 付近）も取り直すので、累積
                // したドリフトがゼロからSolving し直される。
                let rl = relock_every();
                if rl > 0 && self.syms_since_reacq >= rl {
                    self.syms_since_reacq = 0;
                    // 現在の境界の少し手前から再ロックする。
                    // 巻き戻しは 0（= `ISDBT_RELOCKBACK` で上書き可）。
                    //
                    // 実測 2026-09-27: 当初 `rl/2 * lk.sym` まで巻き戻していたが、
                    // 進行量（`rl` シンボル = `rl * lk.sym` サンプル）の半分なので、
                    // 区間を半分しか前進せず `cur` が一定値に固定されていた
                    // （relock#1 cur=2303817 / #2 cur=2303793 / #3 cur=2303792、
                    // 53840 ブロック中 499 ブロックしか処理せず）。巻き戻しゼロなら
                    // 区間が実際に前進し、「内部状態の累積か」を判定できる。
                    let rlback = match std::env::var("ISDBT_RELOCKBACK") {
                        Ok(v) => v.parse().unwrap_or(0),
                        Err(_) => 0,
                    };
                    let back = (rl * rlback / 100 * lk.sym).min(self.cur);
                    if back > 0 {
                        self.cur -= back;
                        self.buf.drain(0..back);
                    }
                    self.k = 0;
                    self.locked = None;
                    self.pipe = None;
                    self.need_init = (LOCK_SYMS + 8) * 1280 + 60_000;
                    self.dc_done = false;
                    self.dc = Complex32::new(0.0, 0.0);
                    self.frac_carry = 0.0;
                    self.reacq_count = 0;
                    self.relock_count += 1;
                    eprintln!("[relock] #{} cur={}", self.relock_count, self.cur);
                    break;
                }
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
                    // `reacquire_boundary` が `None`（peak なし）を返した
                    // ケースを数える。
                    //
                    // `metric_curve` は 100,000 サンプルを計算するが peak
                    // 探索は先頭 17 点（`d = 0..16`、=`ISDBT_TOL`）だけ。
                    // 真の境界が `self.cur + 16` を越えると `best = None`
                    // になり境界が更新されない。**これが起きると
                    // `Δ = pos - cur` は 0..4 に収まったように見えながら
                    // 境界が更新されなくなる**。頻度は数える。
                    if r.is_none() {
                        self.dbg_raq_none += 1;
                    }
                    if let Some((pos, metric)) = r {
                        let delta = pos as i64 - self.cur as i64;
                        // 誤 peak を避けるため、半シンボル以上ずれた検出は捨てる。
                        if delta.abs() < lk.sym as i64 / 2 {
                            eprintln!(
                                "再取得: cur={} → peak={} (Δ{}) metric={:.4}",
                                self.cur, pos, delta, metric
                            );
                            // 診断: 前回選んだ `base` からのジャンプ量を記録。
                            //
                            // 「reacquire が徐々に間違った答えを選ぶ確率が
                            // 上がっている」なら、大きくジャンプした回数が
                            // blk 数とともに増える。`Δ` 自体は 0..16 に収まる
                            // ので絶対値は小さく、**変化の倾向**を見る。
                            if self.dbg_raq_prev > 0 {
                                // 負の jump も正確に持���せるため
                                // 16 bit オフセットを掛ける。
                                let jump = (pos as i64 - self.dbg_raq_prev as i64)
                                    .clamp(-1000, 1000) as i32;
                                // ブロック番号は `cur`（サンプル位置）_until
                                // 次の reacquire までの進行で代用する。
                                // `reacquire` 800 シンボル間隔なので、
                                // Provably 単調増加し blk 軸として使える。
                                // `cur` は 32 bit に収まらないので
                                // 1 万サンプル単位で量子化する。単調増加は
                                // 保つのでblk 軸の proxy として使える。
                                self.dbg_raq_jump
                                    .push((self.cur / 10_000) as u32);
                                self.dbg_raq_jump
                                    .push(((jump + 1000) * 2) as u32);
                            }
                            self.dbg_raq_prev = pos;
                        } else {
                            // 境界更新が**却下**された（`delta` が
                            // 半シンボル以上ずれた peak）。
                            //
                            // `metric_curve` は 100,000 サンプルを計算するが
                            // peak 探索は先頭 17 点（`d = 0..16`）だけ。
                            // 真の境界が `self.cur + 16` を越えると
                            // `best = None` になり、境界は更新されない。
                            //
                            // **これが起き Invisible になると `Δ = pos - cur`
                            // は 0..4 に収まったように見えながら、境界が
                            // 更新されなくなっている**。频度は数える。
                            self.dbg_raq_reject += 1;
                        }
                            // 整数シンボル数で丸めると 0.5 サンプル未満の
                            // 残差が捨てられ、実 fs 差（2.5e-6 サンプル/シンボル）
                            // が累積する。その残差を **分数オフセット** として
                            // 保持し、FFT 窓を線形補間でずらす。
                            //
                            // これが無いと整数境界に丸め続け、等化器のタップが
                            // 合わず RS は 100% でも H.264 ビットが化ける
                            // （実測 2026-09-26: MB エラー 1 件/10 秒 →
                            //  70 件/25 秒、境界 metric 0.96 → 0.89）。
                            // `exact` は整数なので `exact - exact.round()` は
                            // 常に 0.0 になる（`pos` と `self.cur` はともに
                            // `usize`）。そのため分数境界は**常に 0** になり、
                            // 境界ドリフトの分数部分が一切補正されない。
                            //
                            // 実測 2026-09-27: `boundary_frac` は前半・後半とも
                            // 全期間 `0.0000`。`ISDBT_SUBPEAK=1` で `metric_curve`
                            // の peak をパラボリック補間し、整数格子を超える
                            // サブサンプル位置を使う。
                            let exact = (pos as f64) - (self.cur as f64);
                            self.boundary_frac = if crate::demod::subpeak_enabled() {
                                self.demod.sub_peak_frac()
                            } else {
                                (exact - exact.round()).clamp(-0.5, 0.5) as f32
                            };
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
                            // 診断: `k`（pilot 配置 = (phase0+k)%4 の基準）の
                            // 経時変化を追う。`k` が 4 ずつ進むなら
                            // sym_mod4 は正しく、ずれた瞬間だけ壊れる。
                            //
                            // 実測の謎: 800 シンボルごとに再取得が走るのに
                            // 訂正不能が 0% → 78% に累積する。再取得で
                            // `k` の補正が効いていない（syms が 0 に丸められる）
                            // なら、pilot 配置がずれたまま以後の復調が
                            // 全方位で 1 象限ずれて RS が全滅する。
                            if std::env::var("ISDBT_DEBUG").is_ok() {
                                eprintln!(
                                    "[k] cur={} k={} kmod4={} Δ={} syms={} adj={} frac={:+.3} metric={:.4}",
                                    self.cur, self.k, self.k % 4, delta, syms, adj,
                                    self.boundary_frac, metric
                                );
                            }
                        } else {
                            // peak は見つかったが `|delta| >= sym/2` で却下。
                            // 半シンボル以上ずれた peak を採用すると pilot
                            // 配置が完全にずれるので捨てる。正解。
                            self.dbg_raq_reject += 1;
                        }
                    }
                // 分数キャリー付き / なしを選択。`ISDBT_CARRY=1` で有効化。
                //
                // なし（既定）: `next = start + sym`（厳密整数）。端数が毎シンボル
                // 捨てられ、探索基準が常に整数サンプルに戻る。
                // あり: `next = start + floor(carry + frac) + sym`、`carry` は
                // 残り端数を次のシンボルへ持ち越す。
                let use_carry = carry_enabled();
                let got = if use_carry {
                    self.demod.demod_one_tracked_frac_carry(
                        &self.buf,
                        self.cur,
                        lk.gi,
                        lk.cfo,
                        8,
                        self.boundary_frac,
                        self.frac_carry,
                    )
                    .map(|(s, n, c)| (s, n, Some(c)))
                } else {
                    self.demod
                        .demod_one_tracked_frac(
                            &self.buf,
                            self.cur,
                            lk.gi,
                            lk.cfo,
                            8,
                            self.boundary_frac,
                        )
                        .map(|(s, n)| (s, n, None))
                };
                let Some((spec, next, new_carry)) = got else {
                    if n < 5 {
                        eprintln!(
                            "[why] demod None: cur={} buflen={} sym={} k={}",
                            self.cur, self.buf.len(), lk.sym, self.k
                        );
                    }
                    break;
                };
                // ファイン・タイミング追跡（SP 位相傾きベース）。
                //
                // `estimate_symbol_sync` は整数サンプルしか返さないので、
                // 0.5 サンプル未満の残差は取り出せない。局所発振器の
                // 周波数誤差により FFT 窓の開始位置が毎シンボル `sym*ε`
                // サンプルずつずれるため、その残差は次の再取得までの
                // 800 シンボルで累積し、等化器のタップが合わなくなって
                // RS が崩れる（実測: 分割 1.34% / 連続 24〜58%）。
                //
                // SP は既知 BPSK なので、`spec` から残留サンプル誤差を
                // 推定し、EMA で追従させる。`next` は 1 シンボル進んだ
                // 位置なので、誤差は `sym` 倍して差し引く。
                if sp_track_enabled() {
                    let seg = crate::equalize::extract_segment(&spec, lk.seg_off);
                    let m4 = (lk.phase0 + self.k) % 4;
                    let pilots = SegmentPilots::center_1seg();
                    // 複数フレームをまたいで coherent 平均する。各 carrier の
                    // 観測が 1 回だけだと ε が -18〜+28 サンプルで暴れるため。
                    self.sp_acc.push(m4, &seg, &pilots);
                    if self.sp_acc.is_complete() {
                        // 位相 0 単体・D=12（同一シンボル内なので Classen の相殺が成立する）
                        if let Some(meas) = crate::equalize::timing_offset_phase0_d12(
                            &self.sp_acc.phase0_grid(),
                        ) {
                            self.timing_frac = if self.timing_frac == 0.0 {
                                meas
                            } else {
                                sp_alpha() * meas + (1.0 - sp_alpha()) * self.timing_frac
                            };
                            self.boundary_frac = self.timing_frac;
                            if self.k % 512 == 0 {
                                let pa = self.sp_acc.phase_mean_angle();
                                let fmt = |v: Option<f32>| match v {
                                    Some(x) => format!("{:+.3}", x),
                                    None => "NaN".into(),
                                };
                                eprintln!(
                                    "[sptrack] k={} D12={:+.4} ema={:+.4} 位相角=[{},{},{},{}] 観測数={}",
                                    self.k, meas, self.timing_frac,
                                    fmt(pa[0]), fmt(pa[1]), fmt(pa[2]), fmt(pa[3]),
                                    self.sp_acc.observations()
                                );
                            }
                        }
                    }
                }
                self.pipe
                    .as_mut()
                    .unwrap()
                    .process(&spec, lk.phase0, self.k, &mut out);
                self.settle_syms += 1;
                // 状態の定期リセット（**既定無効**、`ISDBT_RESETBLK=N` で opt-in）。
                //
                // 実測 2026-09-27: この `Pipe::resync()` は**無効化**。
                //
                // | 設定 | 結果 |
                // |---|---|
                // | 既定 9.6 秒ごと（この経路） | **訂正不能 99.9%**、TS 0 バイト |
                // | `ISDBT_RESETBLK=0`（無効） | 9183/9183 = **100.0%** |
                //
                // 原因は `resync()` が `self.vit.reset()` で Viterbi トレリスを
                // 全消去すること。トレリスは連続した星座状態に制約されており
                // （`docs/DEGRADATION_INVESTIGATION.md` §3 #7/#8 で実測）、
                // 消すと復帰できない。9.6 秒ごとに 1 フレーム分の I フレームが
                // 落ちるため、視聴上は「後方で画像が崩れる」ことになる。
                //
                // `DEGRADATION_INVESTIGATION.md` §5.1 の「warm-up を再利用可能な
                // 設計にする」 requisite が未達のため、この経路は診断用途に残す
                // だけで、既定では**永久に無効**とする。
                self.frames_since_reset += 1;
                if self.frames_since_reset >= reset_frames() {
                    self.frames_since_reset = 0;
                    if let Some(p) = self.pipe.as_mut() {
                        p.resync();
                    }
                    self.pipe_resets += 1;
                }
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
                if let Some(c) = new_carry {
                    self.frac_carry = c;
                }
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
            // `RsBlockAssembler::feed` と同じ並び（203 バイト → 事後 8 bit）。
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

