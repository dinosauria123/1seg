# ワンセグ（ISDB-T 1seg）復調器 RTS_SDR — 仕様と使い方

最終更新: 2026-09-26（ライブ映像＋音声の復号を実測確認）
対象: `/home/dino/oneseg-rs`（Rust workspace）+ DS-DT-305BK（RTL2832U + Fitipower FC0013）

---

## 1. 何をするソフトか

1seg（中央1セグメント）の IQ を入力すると、OFDM 同期 → チャネル等化 → TMCC 復号 →
デインタ → Viterbi → RS 復号 → MPEG-TS 出力までを一パスで行う。出力 TS は
ffplay / VLC でそのまま再生できる。ブラウザ版（WASM + WebUSB）も同じコアを使う。

フルセグ（約5.6MHz）は RTL-SDR の窓に収まらないので対象外。1seg（約700kHz幅）だけが
窓に収まり、かつスクランブルがかかっていないのでそのまま復号できる。

## 2. ハードウェア前提

| 項目 | 値 |
|---|---|
| チューナ | DS-DT-305BK（`0bda:2832`、RTL2832U + Fitipower FC0013） |
| モード | `rtl2832_sdr` / `dvb_usb_rtl28xxu` を blacklist して `rtl_sdr` から使う |
| 受信例 | 札幌 NHK総合 ch15 = 485.142857 MHz（NHK教育 ch13 = 473.142857 MHz） |
| サンプルレート | **1015873 Hz**（実測 exact = 1015873.002204 Hz） |
| 変調パラメータ | Mode 3 / G1/8 / FFT_LEN 1024 / 中央432キャリア |

1seg の中央セグメントは全体8セグメント中1つで、帯域は約700kHz。1015873 Hz の窓に
完全に収まるのが部分受信（partial reception）。TMCC は全帯域共通なので Layer B/C も
読めてしまうが、実際に物理的に持っているのは Layer A だけ。

## 3. ビルド

```bash
cd ~/oneseg-rs
cargo build --release
cargo test -p isdbt-dsp      # 48 + 4 tests
```

## 4. 使い方

### 4.1 ライブ再生（メイン用途）

```bash
# 周波数指定（札幌 NHK総合 ch15）。引数は [周波数Hz] [秒数、0=終了まで]
./scripts/live_play.sh 485142857

# 周波数省略 → 1seg 帯域の既知チャンネルを順に試してロックしたものを再生
./scripts/live_play.sh
```

Piping を手で組むなら:

```bash
cd ~/oneseg-rs
rtl_sdr -f 485142857 -s 1015873 -g 0 -  \
  | ./target/release/examples/stream_decode - - \
  | ~/.hermes/tools/ffmpeg-9.0.1-linux-x64/bin/ffplay - -autoexit
```

**Wayland セッションでは ffplay のウィンドウが出ないことがある。** Wayland のとき
SDL は Xwayland に落ち、`DISPLAY=:0` 経由の grab は真っ黒になる。これは復調の失敗
ではない。映像を客観的に確認したいときは SDL を介さず ffmpeg にフレーム化させる:

```bash
rtl_sdr -f 485142857 -s 1015873 -g 0 - \
  | ./target/release/examples/stream_decode - - \
  | ffmpeg -i pipe:0 -frames:v 6 -f image2 /tmp/live_%02d.png
```

- **bin offset は自動決定される**（`stream_decode` が `bin offset=NNN` を出力する）。
  capture ごとに数百Hz〜一〜二 bin ズレるため、308等の固定値で当てると信号部が全滅して
  RS 0% になる。`select_segment_offset()` が TMCC フレーム同期一致率（主）+ 既知
  キャリア位相スコア（同点割り）で決める。同期一致 95% 未満は偽ロックとして拒否する。

- `-g 0`（自動ゲイン）を使う。**手動ゲインを指定すると 8bit ADC が飽和して
  TMCC が壊れる**（後述 §6）。
- 再生は **ffplay 優先**。VLC は VA-API / X11 converter で数秒停止し、ログ上の終了は
  映像表示成功の証拠にならない。
- `stream_decode` は起動時に PAT/PMT/SPS/PPS/IDR を溜めてから出力するので、
  中途から join した player でも復号起点が取れる。

### 4.2 ファイルからバッチ復号

```bash
./target/release/examples/decode cap.iq 1015873 out.ts 12000
ffmpeg -ss 6 -i out.ts -frames:v 1 frame.png     # 映像を1枚抜く
```

### 4.3 受信状態の確認（最初に行う）

```bash
# 同期 → TMCC → BCH → Layer パラメータ
./target/release/examples/tmcc_probe cur.iq 1015873

# 「本当に1segが乗っているか」の最終判定。偽ロックを明示的に弾く
./target/release/examples/lock_search cur.iq 1015873 2000

# 同期語16bitの一致ヒストグラム。良品=1.00×16、偽ロック=散在
./target/release/examples/tmcc_bitdump cur.iq 1015873
```

## 5. ロックの判定基準（重要）

TMCC 同期語の一致率 **60〜75%（特に 68.8% = 11/16）は偽ロック**であり、
「信号が弱い」ではない。`find_frame_sync_joint` が 204 通りのフレーム位相から
一致ビット数の最大のものを選ぶため、**ノイズだけでも必ず 60〜75% に張り付く**。

`tmcc_probe` の判定は `FrameSync::is_true_lock()` を使う。真のロックは以下の4条件すべてを満たす場合だけ:

1. 同期語16bitが**全フレームで一貫**している（`consistent_sync_bits == 16`）
2. even/odd フレームが交互（`alternates`）
3. BCH(273,191) が OK
4. 一致率 95% 以上

偽ロックは 1・2・3 のいずれかを必ず落とすので区別できる。実測値:

| 入力 | 一致率 | 一貫ビット | alternates | BCH | 真のロック |
|---|---|---|---|---|---|
| 良品 `sapporo/lowgain/g0.iq` | 100% | 16/16 | true | OK | **YES** |
| 良品 `sapporo/connected.iq` | 100% | 16/16 | true | OK | **YES** |
| 熱ノイズだけの取り込み | 70% | 3/16 | false | NG | **NO** |

**偽ロックは復調器側の調整では絶対に直らない。** sync radius 4/8/16 × セグメント
bin offset 41通りを総当たりしても 70% が天井で、信号そのものが無いため demod する
対象が存在しない。C/N を確保するのはアンテナ／受信環境側の問題。

### 偽ロックを見分ける3つの方法

- **複数チャンネルで同じ値が出る** → 偽ロック。本物の1segならチャンネルごとに違う
- **セグメント bin offset がチャンネル毎に変わる** → 偽ロック。本物なら 308 で固定
- **1seg内 と guard帯の電力差** → 本物は約 3.9dB。熱ノイズだけだと 2〜3dB で頭打ち

## 6. 実運用上の落とし穴

- **手動ゲイン禁止**。`-g 17.9` / `-g 19.7` は 8bit ADC を飽和させ、入力の 18.75%
  （0 と 255 に集中）が潰れて TMCC が壊れる。`-g 0`（自動）が唯一正しい。
- **`No E4000 tuner found, aborting` は失敗ではない**。FC0013 成功後の E4000
  専用フォールバックメッセージ。
- **`timeout` で rtl_sdr を止めると終了が不安定**。IQ へ드는 `head -c` でクリーンに
  EOF させる。
- **低ゲイン群 `lowgain/` だけが 100% ロックした記録がある一方、同条件の
  `gain-test/`・`scan/` は全て偽ロック。ゲイン方向の差は ADC 飽和で説明できる。
- ログ上の "ロック→ライブ復码開始" は **`is_true_lock()` を通ってから出すこと**。

## 7. 診断ツール一覧（`examples/`）

| ツール | 用途 |
|---|---|
| `stream_decode` | ライブ/連続 IQ → MPEG-TS（主線） |
| `decode` | バッチ IQ → MPEG-TS |
| `tmcc_probe` | 同期→TMCC→BCH→Layer パラメータ表示 |
| `lock_search` | radius × offset 総当たり。真ロック可否を明示 |
| `tmcc_bitdump` | 同期語16bitの一致ヒストグラム |
| `sync_probe` | GI 推定 / CFO / CP 周期スコア |
| `tmcc_scan` | 204通りのフレーム位相の同期一致率を CSV ダンプ |
| `sp_phase_probe` / `rs_phase_probe` / `fec_lock_probe` | SP位相・RS位相・FECロックの詳細診断 |
| `latm_probe` | 音声 LATM/HE-AAC の構造確認 |
| `rs_diag` / `ts_probe` / `ts_extract` / `ts_crack` / `ts_final` | RS と TS 内部の掘下げ |
| `synth_ts` / `deinterleave_probe` / `viterbi_chain` / `equalize_probe` / `dump_fft` | 合成信号による段ごとの単体確認 |

## 8. 既知の未達 / 制約

- **映像＋音声のライブ復号は達成済み**（2026-09-26 実測）。`stream_decode` は
  実キャプチャ・実ライブいずれでも RS ブロック 100% を復号し、ffmpeg が
  H.264 (Constrained Baseline 320x180 15fps) + HE-AAC 48kHz stereo として認識する。
- TS を**ファイルとして** ffprobe すると `non-existing PPS` / `no frame!` が出る。
  原因はPAT(PID 0) 等の欠損で、SPS/PPS/IDR 自体は TS 内に実在する。映像ESを直接
  `ffmpeg -f h264` に通せばフレームが出る。TS コンテナ経由の再生が壊れるのは
  未修正。
- `stream_decode` のログが「ロック→ライブ復码開始」を出しても、Wayland 上の
  ffplay ウィンドウが出ないことがある。SDL/Xwayland の問題であり復調とは無関係。
- 音声 PES は HE-AAC / LATM で、PES header / CC は正常。ffplay 側で
  `channel element 3.1 is not allocated` 警告が出るが音は出る。
