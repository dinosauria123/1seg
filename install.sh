#!/usr/bin/env bash
# oneseg-rs のセットアップスクリプト（INSTALL.md §1〜§4 の実体）。
#
# 使い方:
#   ./install.sh              # 依存導入 → blacklist → release ビルド → 動作確認
#   ./install.sh --no-apt     # apt を触らない（依存導入済みならビルドのみ）
#   ./install.sh --dry-run    # 何をするか表示するだけ（変更しない）
#   ./install.sh --no-verify  # ビルド後の IQ キャプチャ確認を省く（ドングルを触らない）
#
# 検証環境: Ubuntu 26.04.1 LTS / x86_64 / rtl-sdr 2.0.2 / Rust 1.93 / FFmpeg 9.0.1
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BLACKLIST_CONF=/etc/modprobe.d/blacklist-rtlsdr.conf

DO_APT=1
DRY_RUN=0
DO_VERIFY=1
for arg in "$@"; do
  case "$arg" in
    --no-apt)   DO_APT=0 ;;
    --dry-run)  DRY_RUN=1 ;;
    --no-verify) DO_VERIFY=0 ;;
    -h|--help)  sed -n '3,10p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'; exit 0 ;;
    *) echo "不明な引数: $arg（--help で使い方）" >&2; exit 2 ;;
  esac
done

run() {
  if [ "$DRY_RUN" -eq 1 ]; then
    printf '  [dry-run] %s\n' "$*"
  else
    printf '  $ %s\n' "$*"
    "$@"
  fi
}
# sudo は DRY_RUN でも実行しない（パスワードプロンプトを出さない）。
sudo_run() {
  if [ "$DRY_RUN" -eq 1 ]; then
    printf '  [dry-run] sudo %s\n' "$*"
  else
    printf '  $ sudo %s\n' "$*"
    sudo "$@"
  fi
}
have() { command -v "$1" >/dev/null 2>&1; }
step() { printf '\n== %s ==\n' "$1"; }

cd "$REPO_DIR"

# ---------------------------------------------------------------
step "1/5 依存パッケージ"
# ---------------------------------------------------------------
# python3-tk が無いと oneseg_gui.py が ModuleNotFoundError: tkinter で落ちるので必須。
APT_PKGS=(rtl-sdr libusb-1.0-0 ffmpeg python3 python3-tk)
# libgpiod-tools は任意（GPIO デバッグ）。
if have apt-get; then
  missing=()
  for p in "${APT_PKGS[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
  done
  if [ ${#missing[@]} -eq 0 ]; then
    echo "  すべて導入済み"
  else
    echo "  未導入: ${missing[*]}"
    if [ "$DO_APT" -eq 1 ]; then
      if [ "$DRY_RUN" -eq 1 ]; then
        printf '  [dry-run] sudo apt-get install -y %s\n' "${missing[*]}"
      else
        sudo apt-get update -qq
        sudo apt-get install -y "${missing[@]}"
      fi
    else
      echo "  --no-apt なのでスキップ（未導入のままだと後段で失敗する）" >&2
    fi
  fi
else
  echo "  apt-get が無い（Ubuntu 以外）。手動で rtl-sdr / ffmpeg / python3-tk を入れてください" >&2
fi

if ! have cargo; then
  echo "  cargo がありません。https://rustup.rs を実行してください" >&2
  exit 1
fi
printf '  rustc %s / cargo %s / python %s\n' \
  "$(rustc --version | cut -d' ' -f2)" \
  "$(cargo --version | cut -d' ' -f2)" \
  "$(python3 --version | cut -d' ' -f2)"

# ---------------------------------------------------------------
step "2/5 カーネルドライバの blacklist"
# ---------------------------------------------------------------
# dvb_usb_rtl28xxu が自動認識されると rtl_sdr が usb_claim_interface error -6 で失敗する。
# sudo tee で追記（既存ファイルの blacklisted エントリは保持）。
BLACKLIST_LINES=$'blacklist dvb_usb_rtl28xxu\nblacklist rtl2832_sdr'
# blacklist-rtl2832.conf など別名前の既存ファイルがあっても二重に作らない。
# modprobe は全 conf を読むので、「どこかに 1 つでもあれば十分」。
existing="$(grep -l '^[[:space:]]*blacklist[[:space:]]\+dvb_usb_rtl28xxu' \
             /etc/modprobe.d/*.conf 2>/dev/null || true)"
if [ -n "$existing" ]; then
  echo "  既に設定済み: $existing"
elif [ "$DRY_RUN" -eq 1 ]; then
  printf '  [dry-run] sudo tee %s <<< "blacklist dvb_usb_rtl28xxu / rtl2832_sdr"\n' "$BLACKLIST_CONF"
else
  printf '%s\n' "$BLACKLIST_LINES" | sudo tee "$BLACKLIST_CONF" >/dev/null
  echo "  $BLACKLIST_CONF を作成"
fi
# カーネルモジュールが実際にロード済みなら外す。
# 未ロードなら sudo を呼ばない（パスワード要求が出て邪魔になる。実測 2026-10-05）。
loaded=""
if have lsmod && have modprobe; then
  # pipefail + set -e だと grep の 0 件（exit 1）でスクリプト全体が死ぬので
  # `|| true` で明示的に成功扱いにする（実測 2026-10-05）。
  loaded="$(lsmod | grep -E '^(rtl2832_sdr|dvb_usb_rtl28xxu)\b' | awk '{print $1}' | tr '\n' ' ' || true)"
fi
if [ -n "${loaded// /}" ]; then
  echo "  ロード中モジュール: $loaded"
  sudo_run modprobe -r rtl2832_sdr dvb_usb_rtl28xxu || true
else
  echo "  rtl2832_sdr / dvb_usb_rtl28xxu はロードされていない（blacklist は効いている）"
fi
echo "  ※ blacklist の反映にはドングルの USB 抜差しが必要です"

# ---------------------------------------------------------------
step "3/5 ビルド"
# ---------------------------------------------------------------
run cargo build --release --workspace
echo "  生成バイナリ（examples）:"
# target/release/examples には .d ファイルと <name>-<hash> の dupe が混ざるので除外。
ls -1 target/release/examples/ 2>/dev/null \
  | grep -vE '\.d$|-[0-9a-f]{16}$' | sed 's/^/    /'
echo "  総数: $(ls -1 target/release/examples/ 2>/dev/null | grep -vcE '\.d$|-[0-9a-f]{16}$')"

# ---------------------------------------------------------------
step "4/5 ドングルの認識確認"
# ---------------------------------------------------------------
if have rtl_test; then
  # rtl_test は開けないと非 0 で終わるので set -e を一時的に無効化する。
  set +e
  rtl_test
  rc=$?
  set -e
  if [ $rc -ne 0 ]; then
    echo
    echo "  rtl_test が失敗。"
    # 実測（2026-10-05）: blacklist 済みでも usb_claim_interface error -6 になる。
    # 別の第一因は「他のプロセスがデバイスを掴んだまま」。先にそちらを確かめる。
    # 判定は pgrep -x（完全一致）で行う。`fuser` はパスと PID しか出さないので、
    # プロセス名で grep すると必ず空振りする（実測でつまずいた）。
    holders=""
    for p in rtl_sdr rtl_test rtl_fw; do
      found="$(pgrep -ax "$p" 2>/dev/null || true)"
      [ -n "$found" ] && holders="$holders$found"$'\n'
    done
    if [ -n "$holders" ]; then
      echo "  → 他のプロセスが SDR デバイスを保持しています。先に停止してください:"
      printf '%s' "$holders" | sed 's/^/      /'
      echo "      ./scripts/stop_live.sh   # ライブ再生の残りプロセスを停止"
    else
      echo "  → rtl_sdr プロセスは無し。カーネル側（blacklist 未反映）の可能性が高い。"
      echo "    ドングルの USB を抜差ししてください（§2）。"
    fi
  fi
else
  echo "  rtl_test が無い（rtl-sdr 未導入？）" >&2
fi

# ---------------------------------------------------------------
step "5/5 動作確認（IQ キャプチャ → TMCC）"
# ---------------------------------------------------------------
# 札幌 NHK総合 ch15 = 485.142857 MHz。SDR を持っていない環境では --no-verify で省略する。
FREQ_HZ=485142857
SAMPLE_RATE=1015873
IQ=/tmp/oneseg_install_test.iq
if [ "$DO_VERIFY" -eq 0 ]; then
  echo "  --no-verify なのでスキップ"
elif ! have rtl_sdr; then
  echo "  rtl_sdr が無いのでスキップ"
else
  echo "  30 秒だけ IQ を取り込む（${FREQ_HZ} Hz）"
  if [ "$DRY_RUN" -eq 1 ]; then
    printf '  [dry-run] timeout 30 rtl_sdr -f %s -s %s -g 5 %s\n' "$FREQ_HZ" "$SAMPLE_RATE" "$IQ"
  else
    set +e
    timeout 30 rtl_sdr -f "$FREQ_HZ" -s "$SAMPLE_RATE" -g 5 "$IQ"
    cap_rc=$?
    set -e
    if [ $cap_rc -ne 0 ]; then
      echo "  rtl_sdr が IQ を取れませんでした（rc=$cap_rc）。"
      echo "    多くは 4/5 と同じ原因です（他のプロセスがデバイス保持 / blacklist 未反映）。"
      echo "    上の 4/5 の出力を確認してください。"
      # 不完全な IQ を残すと次の実行を邪魔するので消す。
      rm -f "$IQ"
    else
      ls -lh "$IQ" | sed 's/^/  /'
      echo "  TMCC 復調:"
      set +e
      ./target/release/examples/tmcc_probe "$IQ" "$SAMPLE_RATE" | sed 's/^/  /'
      set -e
      cat <<'EOF'

  判定の見方（docs/OPERATION.md §5）:
    同期語一致 100% + 一貫 16/16 + BCH OK + 真のロックか YES  → 受信できる
    同期語一致 70% 前後 + BCH NG                            → 偽ロック（信号なし）
  偽ロックは調整では直りません。アンテナと受信環境を確認してください。
EOF
    fi
  fi
fi

if [ "$DRY_RUN" -eq 0 ] && [ -f "$IQ" ]; then
  rm -f "$IQ"
  echo "  一時 IQ ($IQ) を削除"
fi

printf '\n完了。次の手順:\n'
printf '  GUI     : %s/oneseg_gui.py\n' "$REPO_DIR"
printf '  ライブ  : %s/scripts/live_play_direct.sh 509142857   # ch19 HBC\n' "$REPO_DIR"
printf '  停止    : %s/scripts/stop_live.sh\n' "$REPO_DIR"
printf '  詳細    : INSTALL.md / docs/OPERATION.md\n'