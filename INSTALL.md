# oneseg-rs — インストール方法

自作 ISDB-T 1seg（ワンセグ）復調器 `~/oneseg-rs` のセットアップ手順。
検証環境: **Ubuntu 26.04.1 LTS / x86_64 / rtl-sdr 2.0.2 / Rust 1.93 / FFmpeg 9.0.1**

---

## 1. 必要なもの

### 1.1 ハードウェア

| 項目 | 値 | 備考 |
|---|---|---|
| SDR ドングル | **DS-DT-305BK**（RTL2832U + Fitipower FC0013） | 他社製でも可 |
| アンテナ | 1seg 帯域対応（470〜608 MHz） | 壁面アンテナでも可 |
| OS 環境 | Linux（Ubuntu 26.04 で検証） | macOS/Windows は未検証 |

**1seg 帯域が RTL-SDR の窓に収まる範囲だけ**受信できます（約 700 kHz 幅）。
フルセグ（約 5.6 MHz）は 1 本のドングルでは受信不可。

### 1.2 外部プログラム（必須）

| プログラム | 用途 | 導入方法 | 検証版 |
|---|---|---|---|
| **`rtl-sdr`** | IQ キャプチャ | `sudo apt install rtl-sdr` | 2.0.2 |
| **`librtlsdr0`** | librtlsdr 共有ライブラリ（LED 制御で ctypes から使う） | 同上（依存で入る） | 2.0.2 |
| **`libusb-1.0-0`** | USB アクセス | `sudo apt install libusb-1.0-0` | — |
| **`cargo` / `rustc`** | Rust ビルド | `sudo apt install cargo rustc` または rustup | 1.93.1 |
| **`ffmpeg`** | TS → 画像/音声のデコード確認 | `sudo apt install ffmpeg` | 9.0.1 |
| **`ffplay`** | ライブ映像表示（**再生に必須**） | 同上 | 9.0.1 |
| **`python3`** | 補助スクリプト（`iq_trim.py` / `led_ctl.py` / GUI） | `sudo apt install python3 python3-tk` | 3.14.7 |
| **`python3-tk`** | GUI（`oneseg_gui.py`）用。**Ubuntu では tk が別パッケージ** | `sudo apt install python3-tk` | — |

> **重要**: Ubuntu では `python3-tk` が別パッケージです。入れないと
> `oneseg_gui.py` が `ModuleNotFoundError: tkinter` で起動しません。

### 1.3 外部プログラム（任意）

| プログラム | 用途 | 導入方法 |
|---|---|---|
| `libgpiod-tools` | GPIO デバッグ（Linux _generic gpiochip 用） | `sudo apt install libgpiod-tools` |
| `wat2wasm` / `wasm-pack` | ブラウザ版（WASM+WebUSB）のビルド | `cargo install wasm-pack` |
| `iwd` / `sdrd` 等の SDR ツール | スペクトル確認 | `sudo apt install rtl-sdr` に同梱 |

---

## 2. カーネルモジュールの blacklist（RTL2832U を使うため必須）

Ubuntu の `dvb_usb_rtl28xxu` ドライバが **カーネルから自動認識**するため、
`rtl_sdr` がデバイスを開けなくなることがあります。blacklist が必要です。

```bash
# 1) blacklist を作る（既存ファイルに追加）
echo -e "blacklist dvb_usb_rtl28xxu\nblacklist rtl2832_sdr" \
  | sudo tee /etc/modprobe.d/blacklist-rtlsdr.conf

# 2) 適用（USB を抜差しすると反映）
sudo modprobe -r rtl2832_sdr dvb_usb_rtl28xxu 2>/dev/null || true
```

確認:

```bash
rtl_test
# 期待出力:
#   Found 1 device(s):
#     0:  Generic, RTL2832U, SN: 77771111153705700
#   Found Fitipower FC0013 tuner
```

`Found Fitipower FC0013 tuner` が出れば正常です。
出ない場合は blacklist が効いていないか、USB ケーブルの問題です。

---

## 3. リポジトリのビルド

```bash
git clone <repository-url> ~/oneseg-rs
cd ~/oneseg-rs
cargo build --release
```

### テスト

```bash
cargo test -p isdbt-dsp    # 48 + 4 tests
```

### 生成されるバイナリ

```
target/release/examples/
  stream_decode     # ライブ/連続 IQ → MPEG-TS（主線）
  decode            # バッチ IQ → MPEG-TS
  tmcc_probe        # 同期→TMCC→BCH→Layer パラメータ表示
  lock_search       # 真ロック可否の明示判定
  tmcc_bitdump      # 同期語 16bit の一致ヒストグラム
  ...
```

---

## 4. セットアップ確認（受信できるかの確認）

### 4.1 IQ キャプチャのテスト

```bash
# 札幌 NHK総合 ch15 = 485.142857 MHz
timeout 30 rtl_sdr -f 485142857 -s 1015873 -g 0 /tmp/test.iq
ls -lh /tmp/test.iq      # → 約 60 MB なら成功
```

### 4.2 TMCC 復調の確認

```bash
./target/release/examples/tmcc_probe /tmp/test.iq 1015873
```

**判定の見方**（詳細は `docs/OPERATION.md` §5）:

| 出力 | 意味 |
|---|---|
| `同期語一致: 100%` + `一貫 16/16` + `BCH OK` + `真のロックか: YES` | ✅ 受信可能 |
| `同期語一致: 70%` 前後 + `一貫 2/16` + `BCH NG` | ❌ **偽ロック**（熱ノイズ。信号なし） |

> ⚠️ **偽ロックに注意**: `find_frame_sync_joint` が 204 通りのフレーム位相から
> 一致ビット数の最大のものを選ぶため、**ノイズだけでも必ず 60〜75% に張り付く**。
> 一致率 68.8% = 11/16 は「信号が弱い」ではなく**信号が無い**ことを意味する。
> 復調器の調整では直らない。アンテナ/受信環境の問題。

### 4.3 TS 復号の確認

```bash
./target/release/examples/decode /tmp/test.iq 1015873 /tmp/out.ts 12000
ffprobe -v error -show_entries format=duration /tmp/out.ts
ffmpeg -ss 6 -i /tmp/out.ts -frames:v 1 /tmp/frame.png
```

期待: `duration ≈ 20`（IQ 30 秒から 70% 程度）、
`frame.png` に実際のテレビ映像が出る。

---

## 5. 使い方

### 5.1 GUI（推奨）

```bash
~/oneseg-rs/oneseg_gui.py
```

- **▲ CH / ▼ CH** または **↑↓** キー: チャンネル切替（**循環**。端で反対側へ回る）
- **▶ 再生** または **Space**: 再生開始
- **■ 停止** または **Esc**: 停止
- 映像は ffplay のウィンドウで表示（GUI 内蔵ではない）

チャンネル一覧（札幌・手稲山親局）:

| ch | 周波数 MHz | 局名 | リモコン |
|---|---|---|---|
| 13 | 473.142857 | NHK教育 | 2 |
| 14 | 479.142857 | TVh | 7 |
| 15 | 485.142857 | NHK総合 | 3 |
| 19 | 509.142857 | HBC | 1 |
| 21 | 521.142857 | STV | 5 |
| 23 | 533.142857 | HTB | 6 |
| 25 | 545.142857 | UHB | 8 |

### 5.2 スクリプト（コマンドライン）

```bash
# ライブ再生（チャンネル指定）
~/oneseg-rs/scripts/live_play_direct.sh 509142857     # ch19 HBC

# 停止
~/oneseg-rs/scripts/stop_live.sh

# 30 秒の IQ を TS に録音
~/oneseg-rs/scripts/record30.sh 509142857 30
```

環境変数で動作調整:

| 変数 | 既定 | 説明 |
|---|---|---|
| `GAIN` | `5` | rtl_sdr のゲイン。**`-g 0`（AGC）は飽和して復調悪化する**（実測 2026-10-04） |
| `SEGOFF_RETRY` | `2` | bin offset の ±リトライ。0 で高速化、偽値リスク |
| `IQ_KEEP` | `40000000` | IQ 保持量（40 MB ≒ 20 秒） |
| `IQDIR` | `~/oneseg-rs/captures/live` | IQ の保存先（**ext4 推奨**） |
| `LED` | `1` | 緑 LED の GPIO 点灯。0 で無効 |

---

## 6. トラブルシューティング

### 6.1 `usb_claim_interface error -6`

**カーネルの dvb_usb_rtl28xxu ドライバがデバイスを掴んでいる。**
→ §2 の blacklist を適用して USB を抜差しする。

### 6.2 `Found E4000 tuner found, aborting`

**異常ではない。** FC0013 成功後の E4000 専用フォールバックメッセージ。
上の行に `Found Fitipower FC0013 tuner` があれば正常動作中。

### 6.3 「1seg が映らない」

| 原因 | 判定 | 対処 |
|---|---|---|
| 偽ロック（信号なし） | `tmcc_probe` の `真のロックか: NO` | アンテナ調整。C/N 向上 |
| ADC 飽和 | `agc_check.py` の Kurtosis < 2.7 | `GAIN=5` にする（既定値） |
| bin offset ずれ | TMCC 同期が 0.95 未満で止まる | `SEGOFF_RETRY=2` で再実行 |
| ディスク満杯 | `echo: write error` | `stop_live.sh` → IQ を削除 |

### 6.4 1 分で再生が止まる

**`/tmp/isdbt_collapsed` のチャンネル間汚染**（修正済み `df6a38d`）。
累积値は IQ ファイルごとに分離されている。古いバージョンの場合は再取得。

### 6.5 `/tmp` が小さい

`/tmp` は **tmpfs 3.6G**。`fallocate --collapse-range` が使えない。
IQ の保存先を ext4 にする（`IQDIR` で変更可能）:

```bash
IQDIR=~/captures ~/oneseg-rs/scripts/live_play_direct.sh 509142857
```

---

## 7. ディレクトリ構成

```
~/oneseg-rs/
├── Cargo.toml               # ワークスペース定義
├── crates/
│   ├── isdbt-dsp/           # 復調コア（Rust library + examples）
│   │   ├── src/
│   │   │   ├── iq.rs        # ① RF 入力
│   │   │   ├── sync.rs      # ② OFDM 同期
│   │   │   ├── pilots.rs    # ③ チャネル等化
│   │   │   ├── tmcc.rs      # ④ TMCC 復号・bin offset 選択
│   │   │   ├── deinterleave.rs / demap.rs
│   │   │   ├── viterbi.rs   # ⑤ FEC 前段
│   │   │   ├── rs.rs        # ⑤ RS(204,188) 復号
│   │   │   ├── ts.rs        # ⑥ TS 出力・PTS 正規化
│   │   │   └── stream.rs    # ストリーミング復調器
│   │   └── examples/        # 診断・復号ツール
│   └── isdbt-wasm/          # ブラウザ版（WASM+WebUSB）
├── oneseg_gui.py          # tkinter GUI
├── scripts/
│   ├── live_play_direct.sh  # ライブ再生（GUI が呼ぶ実体）
│   ├── stop_live.sh         # 停止・IQ 削除・LED 消灯
│   ├── iq_trim.py           # IQ 容量管理（collapse + 累積値通知）
│   ├── led_ctl.py           # 緑 LED の GPIO 制御
│   ├── lock_watch.py        # ロック監視
│   └── diag/                # 診断ツール群
├── docs/
│   ├── OPERATION.md         # 仕様と使い方（最重要）
│   ├── CONSTELLATION_SNR.md # 星座解析・C/N
│   └── DEGRADATION_INVESTIGATION.md
└── captures/                # IQ キャプチャ置き場（ext4）
```

---

## 8. ライセンス

GPL-3.0-or-later（参照実装 `gr-isdbt` が GPL のため）。