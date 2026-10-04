#!/usr/bin/env bash
# ライブ 1seg 視聴のプロセス一式を停止する。
#
# 2 つの停止方式を使う（どちらも必須）:
#
#  1) 名前ベース: vlc / ffplay / stream_decode / rtl_sdr は `pgrep -x`（完全一致）
#     で拾える。
#
#  2) PID ファイルベース: python スクリプト（lock_watch.py, ts_http_server.py）は
#     プロセス名が python3 なので名前では判別できない。
#     **`pgrep -f` を使ってはいけない。** 実測 2026-10-03 23:51:
#     `bash scripts/live_play_direct.sh` を実行したラッパOSHELL が、
#     コマンドラインに "lock_watch.py" を含むため pgrep にマッチされ、
#     stop 処理の途中で自爆して全体が止まった（復旧不能）。
#     起動側が自分の PID をファイルに書いておけば、
#     それは「自分が起動した PID」に限られるので安全。
#
# 起動側 (live_play_direct.sh) が書くファイル:
#   /tmp/isdbt_launcher.pid   … python スクリプト群の PID（1行ずつ）
set -uo pipefail

self=$$
parent=$PPID
freed=0

# --- 1) 名前ベース（pgrep -x は完全一致なので自分Perc にマッチしない）---
for name in vlc ffplay stream_decode rtl_sdr; do
  for p in $(pgrep -x "$name" 2>/dev/null); do
    [ "$p" = "$self" ] && continue
    [ "$p" = "$parent" ] && continue
    kill -9 "$p" 2>/dev/null
  done
done

# --- 2) PID ファイルベース ---
PIDF=/tmp/isdbt_launcher.pid
if [ -f "$PIDF" ]; then
  while read -r lp; do
    [ -z "$lp" ] && continue
    [ "$lp" = "$self" ] && continue
    [ "$lp" = "$parent" ] && continue
    # PID が枯番して別のプロセスに再利用されていると kill 対象がずれるので、
    # その PID が本当に我々のスクリプトか確認してから kill する。
    exe=$(readlink "/proc/$lp/exe" 2>/dev/null || true)
    case "$exe" in
      *python*)
        cmd=$(tr '\0' ' ' < "/proc/$lp/cmdline" 2>/dev/null || true)
        case "$cmd" in
          *lock_watch.py*|*ts_http_server.py*|*iq_trim.py*)
            kill -9 "$lp" 2>/dev/null
            ;;
          *)
            # python だが我々のスクリプトではない（PID 再利用）。触らない。
            ;;
        esac
        ;;
    esac
  done < "$PIDF"
  rm -f "$PIDF"
fi

# --- 3) IQ キャプチャの逐次削除 ---
#
# 为什么要删（実測 2026-10-04 11:15「NHK総合とHBCが映らない」）：
#   /tmp は tmpfs 3.6G しかない。7 チャンネルの IQ を 400 MB ずつ取ると
#   2.9G を使い切り、`echo: write error: ディスク使用量制限を超過しました` で
#   **起動スクリプトの 113 行目（ALIVE ファイル書き込み）が失敗する**。
#   その結果 stream_decode はライターハンドラを見失って停止し、
#   ffplay には空の FIFO が渡るだけになる = ウィンドウは出るが真っ黒。
#   実害は「再生不出来」であり、IQ の残量ではない。
#
# 必要な IQ 量は 10 MB（5.1 秒）だけ（stream.rs need_init）。ファイルは
# `--follow` が末尾から逐次読むので、**保持量はロックに影響しない**。
# 従って停止時に全消しで問題ない。
#
# 消さないもの: /tmp/isdbt_segoff_*（bin offset の実測値。消すと毎回探索する）
#
# IQ は 2026-10-04 から ext4 上（~/oneseg-rs/captures/live/）に置く。
# `/tmp` のままだと tmpfs で `fallocate --collapse-range` が使えないので
# iq_trim.py の容量制限が効かず、IQ が 400 MB を超えていた。
IQDIR="${IQDIR:-$HOME/oneseg-rs/captures/live}"
for f in "$IQDIR"/isdbt_iq_*.iq /tmp/isdbt_iq_*.iq; do
  [ -f "$f" ] || continue
  sz=$(stat -c%s "$f" 2>/dev/null || echo 0)
  rm -f "$f"
  freed=$((freed + sz))
done
for f in /tmp/isdbt_ts_*.fifo; do
  [ -e "$f" ] && rm -f "$f"
done
if [ "${freed:-0}" -gt 0 ]; then
  echo "IQ キャプチャ削除: $((freed / 1048576)) MB 回収"
fi

sleep 1
echo "停止完了。残存:"
pgrep -x vlc >/dev/null 2>&1 && echo "  vlc: 残りあり" || echo "  vlc: なし"
pgrep -x ffplay >/dev/null 2>&1 && echo "  ffplay: 残りあり" || echo "  ffplay: なし"
pgrep -x stream_decode >/dev/null 2>&1 && echo "  stream_decode: 残りあり" \
  || echo "  stream_decode: なし"
pgrep -x rtl_sdr >/dev/null 2>&1 && echo "  rtl_sdr: 残りあり" \
  || echo "  rtl_sdr: なし"
echo "  python 系: PID ファイル経由で処理済み（残 PID を確認済み）"