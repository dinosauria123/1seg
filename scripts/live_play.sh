#!/usr/bin/env bash
# 1seg ライブ視聴: DS-DT-305BK → stream_decode → ffplay / vlc
#
#   ./scripts/live_play.sh [周波数Hz] [秒数] [player]
#
# 例:
#   ./scripts/live_play.sh 485142857            # 札幌 NHK総合 ch15、ffplay、終了まで
#   ./scripts/live_play.sh 485142857 60 vlc      # 60秒だけ VLC で
#   ./scripts/live_play.sh 485142857 0 vlc       # VLC で、VLC を自分で閉じるまで
#   ./scripts/live_play.sh                       # 周波数自動走査、ffplay
#
# 秒数に 0 を渡すとプレイヤーは自分で閉じるまで動き続ける（ウィンドウを閉じるか Ctrl+C）。
#
# 環境変数でも指定できる: PLAYER=vlc FREQ=485142857 DUR=0 ./scripts/live_play.sh
set -uo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.hermes/tools/ffmpeg-9.0.1-linux-x64/bin:$PATH"
# Wayland セッションでも X アプリ（ffplay/VLC）が Xwayland 経由で起動できるようにする。
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

FS=1015873            # 1seg 1チャネルがちょうど収まる実測サンプルレート
FREQ=${1:-${FREQ:-}}
DUR=${2:-${DUR:-0}}   # 0 = プレイヤーが自分で終わるまで
PLAYER=${3:-${PLAYER:-ffplay}}

RTL=$(command -v rtl_sdr) || { echo "rtl_sdr が見つかりません"; exit 1; }
DEC=./target/release/examples/stream_decode
[ -x "$DEC" ] || { echo "$DEC が無い。先に cargo build --release"; exit 1; }

# ゲイン0=自動。手動ゲインは8bit ADCを飽和させ TMCC が壊れる（docs/OPERATION.md §6）。
start_rtl() {
  if [ "$DUR" -gt 0 ]; then
    timeout "$DUR" "$RTL" -f "$1" -s "$FS" -g 0 -
  else
    "$RTL" -f "$1" -s "$FS" -g 0 -
  fi
}

play() {
  local f=$1
  case "$PLAYER" in
    vlc)
      # VA-API は Wayland 環境で vaInitialize が失敗するので CPU デコードさせる。
      # --vout=xcb_x11 で vout を固定（gl は vaapi/vdpau を参照して不安定）。
      #
      # 1seg は PSI を TMCC 経由で運ぶため復調 TS に PAT が無い。
      # stream_decode が PAT/PMT を合成して差し込む（--no-psi で無効化可）ので、
      # ffmpeg での remux は本来不要。実測では
      #   PAT 注入のみ      → ffprobe は program を解決する / VLC は停止
      #   + ffmpeg remux    → VLC も安定再生（66秒 deadlock 0）
      # なので VLC 経路では ffmpeg の remux を挟む（TS の PID/PCR も正規化される）。
      echo "=== vlc: $((f / 1000000)) MHz（VLC を自分で閉じて終了） ==="
      start_rtl "$f" | "$DEC" - - --live \
        | ffmpeg -v error -fflags +genpts -i pipe:0 -c copy -f mpegts pipe:1 \
        | vlc --demux=ts --avcodec-hw=none --vout=xcb_x11 \
              --no-video-title-show --no-osd \
              --clock-jitter=0 --clock-synchro=0 - \
      ;;
    *)
      echo "=== ffplay: $((f / 1000000)) MHz ==="
      start_rtl "$f" | "$DEC" - - \
        | ffplay -autoexit -window_title "1seg $f Hz" -
      ;;
  esac
}

if [ -n "$FREQ" ]; then
  play "$FREQ"
  exit $?
fi

# 周波数未定: 1seg チャンネル群を順に試す（各30秒・1seg帯域の既知チャンネル）
for f in 485142857 473142857 497142857 509142857 525142857; do
  echo "### 試行 $f"
  tmp=$(mktemp)
  ( timeout 30 "$RTL" -f "$f" -s "$FS" -g 0 - | "$DEC" - - > "$tmp" 2>"$tmp.log" )
  if grep -q "ロック→ライブ復調開始" "$tmp.log"; then
    echo "ロック成功: $f（$(stat -c%s "$tmp") バイト）"
    rm -f "$tmp" "$tmp.log"
    play "$f"
    exit 0
  fi
  tail -2 "$tmp.log"
  rm -f "$tmp" "$tmp.log"
done
echo "1seg チャンネルが見つかりません（アンテナ/C/N を確認）"
exit 1
