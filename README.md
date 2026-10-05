# UbuntuでDS-DT308SVをワンセグ再生に使う

https://github.com/Ryujiyasu/1seg.git
のコードをFork元にして、UbuntuでDS-DT308SV（RTL2832U + FC0013）で1seg再生するコードです。
Hermes Agent（space-bunny-alpha）にお任せで開発しています。

以下引用と追記です。

RTL-SDR ドングルで受けた IQ から、**ISDB-T のワンセグ（1セグメント）を自前で復調**する
Rust プロジェクト。復調した MPEG-TS を **ffplay でそのまま再生**でき、
tkinter の **GUI 付き**でチャンネル切替・再生・停止ができる。

実測で到達しているもの:
- **1seg 映像 + 音声のライブ復調**（H.264 320x180 15fps + HE-AAC 48kHz stereo）
- **GUI で 7 チャンネル切替**（札幌・手稲山親局）
- **緑 LED の GPIO 点灯**（ドングルの稼働表示）

## 導入

```bash
git clone https://github.com/dinosauria123/1seg.git ~/oneseg-rs
cd ~/oneseg-rs
./install.sh              # 依存導入 → blacklist → release ビルド → 動作確認
./oneseg_gui.py           # GUI を起動
```

`install.sh --dry-run` で変更せずに何をするか確認できます。
手順の詳細とトラブルシューティングは [INSTALL.md](INSTALL.md)。

---

## 参考、流用した既存ソフトウェア

以下のコードを流用しています。

| ソフトウェア | URL | ライセンス | 参考、流用した内容 |
|---|---|---|---|
|**RTS_SDR — 自作ワンセグ（ISDB-T 1seg）復調器**|<https://github.com/Ryujiyasu/1seg.git>| 記載なし | オリジナルのソース |
| **gr-isdbt** | <https://github.com/git-artes/gr-isdbt> | GPL-3.0 | ISDB-T 固有の処理（TMCC デコード、PRBS エネルギー分散、リセット基準）。ISDB-T 圏で実際に運用されている数少ないオープンソース実装。 |
| **DAB-Radio** | <https://github.com/williamyang98/DAB-Radio> | （upstream のものを参照） | GNU Radio に依存しない**単体実装**の手本。設計判断（ブロック分割、エラー処理、構成の簡潔さ）の参考にした。 |
| **rtl-sdr** | <https://github.com/rtlsdr/rtl-sdr> | GPL-3.0 | `rtlsdr_set_bias_tee_gpio()` などの低レベル API の GPIO レジスタ定義。緑 LED の制御に使った。 |
| **librtlsdr**（同上の一部） | <https://github.com/rtlsdr/rtl-sdr> | GPL-3.0 | Demod レジスタの GPIO（`SYS_GPIO_OUT_VAL` = 0x3001 など）。Linux カーネルの `dvb-usb-v2/rtl28xxu.h` と同じ定義。 |

一次資料（仕様の原文）:

| 資料 | 内容 |
|---|---|
| **ARIB STD-B31** | 地上デジタル伝送方式（ISDB-T）。伝送パラメータ、TMCC の構造、セグメント構成の一次規格。 |
| **ARIB STD-B24** | 音声伝送方式（LATM/HE-AAC の 1seg での運用）。 |

既存ソフトウェアの Clone 方法（`ref/` は `.gitignore` 済み）:

```bash
git clone --depth 1 https://github.com/git-artes/gr-isdbt.git ref/gr-isdbt
git clone --depth 1 https://github.com/williamyang98/DAB-Radio.git ref/DAB-Radio
```

> 参照実装には ISDB-T 圏（ブラジル・日本）で使用されているオープンソースソフトウェアが含まれます。
> 本プロジェクトは **合法な地上デジタル放送の受信**（自分の地域で受信できる放送）を
> 対象としており、復調そのものは暗号解読を含みません。

---

## 主な機能

### ライブ復調（`stream_decode`）

IQ（stdin / ファイル）を単一パスで復調して TS を stdout に出力。
`--follow` でファイルの追記を追従し、`--live` で低遅延モード。

```bash
rtl_sdr -f 509142857 -s 1015873 -g 5 /tmp/iq.fifo &
stdbuf -oL ./target/release/examples/stream_decode /tmp/iq.fifo /tmp/ts.fifo --live --follow &
ffplay -flags low_delay -framedrop -x 640 -window_title "1seg 札幌" -f mpegts -i /tmp/ts.fifo
```

### バッチ復号（`decode`）

IQ ファイルから TS を一括生成。

```bash
./target/release/examples/decode cap.iq 1015873 out.ts 12000
ffmpeg -ss 6 -i out.ts -frames:v 1 frame.png
```

### GUI

tkinter 製。▲▼ でチャンネル切替（循環）、▶/■ で再生/停止。

```bash
~/oneseg-rs/oneseg_gui.py
```

### 緑 LED の点灯

ドングルの緑 LED を GPIO で制御して、ドングルの稼働を表示。

```bash
python3 scripts/led_ctl.py set      # 点灯
python3 scripts/led_ctl.py unset    # 消灯
```

---

## ビルド & テスト

```bash
cargo build --release
cargo test -p isdbt-dsp
```

---

## セットアップ

インストール手順・外部依存・カーネルモジュールの blacklist などは
**[`INSTALL.md`](INSTALL.md)** を参照。

---

## 主な診断ツール

| ツール | 用途 |
|---|---|
| `tmcc_probe` | 同期 → TMCC → BCH → Layer パラメータ表示（**最初に使う**） |
| `lock_search` | 偽ロックと真ロックの明示判定 |
| `tmcc_bitdump` | 同期語 16bit の一致ヒストグラム |
| `rs_diag` / `ts_probe` / `ts_extract` | RS と TS 内部の掘り下げ |
| `snr_constellation.py` | 等化出力の星座解析（C/N 測定） |
| `agc_check.py` | 入力の飽和診断（Kurtosis / 原子 / Rayleigh 比） |
| `seg_offset_survey.py` | bin offset の実測（チャンネルごと 1 bin ずつ） |

### 偽ロックの判定（重要）

TMCC 同期語の一致率 **60〜75%（特に 68.8% = 11/16）は偽ロック**であり、
「信号が弱い」ではありません。`FrameSync::is_true_lock()` で真偽を判定します:

1. 同期語 16bit が全フレームで一貫（`consistent_sync_bits == 16`）
2. even/odd フレームが交互（`alternates`）
3. BCH(273,191) が OK
4. 一致率 95% 以上

**偽ロックは復調器の調整では絶対に直りません**（信号が無いため）。
C/N を確保するのはアンテナ／受信環境側の問題です。

---

## 主な制約・既知の問題

| 項目 | 内容 |
|---|---|
| 受信範囲 | **1seg のみ**。フルセグ（約 5.6 MHz）は 1 本のドングルでは不可 |
| ゲイン | **`-g 5` が最適**。`-g 0`（AGC）は ADC 飽和で復調悪化する（実測） |
| bin offset | チャンネルごとに違う（1 bin = 992 Hz）。`/tmp/isdbt_segoff_<freq>` に保存 |
| 保持量 | `/tmp` は tmpfs 3.6G。IQ は `captures/live/`（ext4）に置くのが安全 |
| H.264 MB エラー | 訂正不能 0.7% の散発に由来。`discontinuity_indicator` では完全には防げない |
| 地域 | 札幌（手稲山親局）のチャンネル一覧をハードコード。他の地域では変更が必要 |

---

## ドキュメント

| ファイル | 内容 |
|---|---|
| [`INSTALL.md`](INSTALL.md) | インストール方法・外部依存・トラブルシューティング |
| [`docs/OPERATION.md`](docs/OPERATION.md) | **仕様と使い方**（最重要。復調の段構成、落とし穴、実測値） |
| [`docs/CONSTELLATION_SNR.md`](docs/CONSTELLATION_SNR.md) | 星座解析と C/N の関係 |
| [`docs/DEGRADATION_INVESTIGATION.md`](docs/DEGRADATION_INVESTIGATION.md) | 復調劣化の調査記録 |

---

## ライセンス

GPL-3.0-or-later（参照実装 `gr-isdbt` が GPL のため）。
