#!/usr/bin/env bash
# 1seg ライブ視聴: DS-DT-305BK → stream_decode → ffplay
#
#   ./scripts/live_play.sh [周波数Hz] [秒数]
#
# 例: ./scripts/live_play.sh 485142857        # 札幌 NHK総合 ch15
#   ./scripts/live_play.sh 473142857        # 札幌 NHK教育 ch13
#
# 周波数を省略すると 1seg 帯域（473〜518MHz）を走査して TMCC を確実に取るチャンネルを選ぶ。
set -uo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.hermes/tools/ffmpeg-9.0.1-linux-x64/bin:$PATH"

FS=1015873            # 1seg 1チャネルがちょうど収まる実測サンプルレート
DUR=${2:-0}           # 0 = 終了まで
FREQ=${1:-}

RTL=$(command -v rtl_sdr) || { echo "rtl_sdr が見つかりません"; exit 1; }
DEC=./target/release/examples/stream_decode
[ -x "$DEC" ] || { echo "$DEC が無い。先に cargo build --release"; exit 1; }

# ゲイン0=自動。手動ゲインは8bit ADCを飽和させ TMCC が壊れる（OPERATION.md §6）。
run() {
  local f=$1
  echo "=== $((f / 1000000)) MHz ==="
  if [ "$DUR" -gt 0 ]; then
    timeout "$DUR" "$RTL" -f "$f" -s "$FS" -g 0 - | "$DEC" - - | ffplay -autoexit -window_title "1seg ${f}Hz" -
  else
    "$RTL" -f "$f" -s "$FS" -g 0 - | "$DEC" - - | ffplay -autoexit -window_title "1seg ${f}Hz" -
  fi
}

if [ -n "$FREQ" ]; then
  run "$FREQ"
  exit $?
fi

# 周波数未定: 1seg チャンネル群を順に試す（各30秒・1seg帯域の既知チャンネル）
for f in 485142857 473142857 497142857 509142857 525142857; do
  echo "### 試行 $f"
  tmp=$(mktemp)
  ( timeout 30 "$RTL" -f "$f" -s "$FS" -g 0 - | "$DEC" - - > "$tmp" 2>"$tmp.log" )
  if grep -q "ロック→ライブ復码開始" "$tmp.log"; then
    echo "ロック成功: $f（$(stat -c%s "$tmp") バイト）"
    run "$f"
    rm -f "$tmp" "$tmp.log"
    exit 0
  fi
  tail -2 "$tmp.log"
  rm -f "$tmp" "$tmp.log"
done
echo "1seg チャンネルが見つかりません（アンテナ/C/N を確認）"
exit 1
