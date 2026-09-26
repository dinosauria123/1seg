#!/usr/bin/env bash
# 1seg の実放送を録音して TS（MPEG-TS）ファイルに保存する。
#
#   ./scripts/record30.sh                     # 札幌 NHK総合 ch15、30 秒
#   ./scripts/record30.sh 473142857 60        # 周波数(Hz) と秒数を指定
#   ./scripts/record30.sh 485142857 30 out.ts # 出力ファイル名も指定
#
# **IQ をファイルに録ってから一括復調する**のが要点。直接パイプ
# `rtl_sdr | stream_decode` で繋ぐと復調が実時間に追いつかず、TS 出力が
# 実時間の 1.6% に落ちる（実測 2026-09-26: 30 秒録音 → TS 2.07 秒）。
# IQ を中介すれば実時間の 1.5 倍で処理され、30 秒から 22〜33 秒の TS が得られる
# （残差はロックと暖機の処理時間）。
set -uo pipefail
cd "$(dirname "$0")/.."
export PATH="$HOME/.hermes/tools/ffmpeg-9.0.1-linux-x64/bin:$PATH"
export DISPLAY="${DISPLAY:-:0}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"

FREQ=${1:-485142857}          # 札幌 NHK総合 ch15
DUR=${2:-30}                  # 録音秒数
OUT=${3:-oneseg_30s.ts}       # 出力 TS（VLC 互換で remux 済み）
RAW=$(mktemp /tmp/isdbt_raw_XXXXXX.ts)
FS=1015873                    # 1seg 1 チャネルがちょうど収まるサンプルレート
IQ=$(mktemp /tmp/isdbt_rec_XXXXXX.iq)

RTL=$(command -v rtl_sdr) || { echo "rtl_sdr が見つかりません"; exit 1; }
DEC=./target/release/examples/stream_decode
[ -x "$DEC" ] || { echo "$DEC が無い。先に cargo build --release"; exit 1; }
trap 'rm -f "$IQ" "$RAW"' EXIT

echo "=== 1) IQ 録音: $FREQ Hz / ${DUR} 秒 ==="
# ゲイン0=自動。手動ゲインは 8bit ADC を飽和させ TMCC が壊れる（OPERATION.md §6）。
timeout "$DUR" "$RTL" -f "$FREQ" -s "$FS" -g 0 - > "$IQ" 2>/tmp/isdbt_rtl.log
SZ=$(stat -c%s "$IQ")
[ "$SZ" -lt 1000000 ] && { echo "録音が短すぎます（$SZ バイト）"; tail -3 /tmp/isdbt_rtl.log; exit 1; }
echo "IQ: $SZ バイト（$(python3 -c "print(f'{$SZ/2/$FS:.1f}')") 秒相当）"

echo "=== 2) 1seg の真ロック確認 ==="
./target/release/examples/tmcc_probe "$IQ" "$FS" 2>&1 | grep -E "真のロック|同期語一致|判定" | head -3

echo "=== 3) 一括復調 ==="
$DEC "$IQ" "$RAW" --live 2>/tmp/isdbt_dec.log
grep -oE "終了:.*" /tmp/isdbt_dec.log

echo "=== 4) ffmpeg で remux（VLC 互換にする）==="
# demod が出した生 TS は PAT/PMT の CC（continuity counter）が不完全で、
# libdvbpsi が `TS discontinuity (received 3, expected 1) for PID 8136` を出し、
# VLC が `buffer deadlock prevented` で止まる（実測 2026-09-26）。
# ffmpeg で -c copy して remux すると PSI/PCR が正規化され消える。
# -fflags +genpts+igndts: 壊れた DTS を無視して PTS を生成。
# -muxdelay 0: バッファ溜めを無効化し低遅延。
echo "（VLC で開ける TS にするため remux する）"
ffmpeg -v error -y -fflags +genpts+igndts -i "$RAW" -c copy -muxdelay 0 -f mpegts "$OUT" 2>&1 | grep -v "extradata missing" | head -3

echo "=== 5) 検証 ==="
ls -la "$OUT"
echo -n "0x47 sync 異常: "
python3 - "$OUT" <<'PY'
import sys
d=open(sys.argv[1],'rb').read(); n=len(d)//188
print(f"{sum(1 for i in range(n) if d[i*188]!=0x47)}/{n}")
PY
ffprobe -v error -show_entries format=format_name,duration,bit_rate -of default=nw=1 "$OUT"
ffprobe -v error -show_entries stream=index,codec_name,width,height,sample_rate,channels -of csv=p=0 "$OUT" 2>/dev/null | head -4
FR=$(ffprobe -v error -select_streams v:0 -count_frames -show_entries stream=nb_read_frames -of csv=p=0 "$OUT" 2>/dev/null | head -1)
MB=$(ffmpeg -v error -i "$OUT" -f null - 2>&1 | grep -c "error while decoding MB")
echo "映像フレーム数: $FR   MBエラー: $MB"
echo
echo "再生:  ffplay $OUT   /   vlc $OUT   /   mpv $OUT"
echo "  ※ VLC は Wayland 経由（Xwayland）。窓が出ない場合は SDL 経由の ffplay を。"

# VLC なら -c copy の remux 済み TS を使うこと（生 TS だと buffer deadlock で止まる）
