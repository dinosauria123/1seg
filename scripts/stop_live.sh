#!/usr/bin/env bash
# ライブ 1seg 視聴のプロセス一式を停止する。
# 起動スクリプト（live_play_http.sh 等）がバックグラウンドで起動した
# プロセスも対象にするため、PID 指定で確実に落とす
# （pkill -f は自分自身のシェルにもマッチして自殺する）。
set -uo pipefail

for name in vlc ffplay stream_decode rtl_sdr; do
  for p in $(pgrep -x "$name" 2>/dev/null); do
    kill -9 "$p" 2>/dev/null
  done
done

# python の ts_http_server.py（プロセス名が python3 なので script 名で特定）
# 注意: `pgrep -f` はこのスクリプト自身にもマッチしうるので、
#   - 引数に "bash scripts/stop_live.sh" を含む PID
#   - 自分の PID と親 PID
# を除外しないと trap で自爆する（実測: SIGKILL でスクリプトが止まる）。
self=$$
parent=$PPID
for p in $(pgrep -f "ts_http_server\.py" 2>/dev/null); do
  [ "$p" = "$self" ] && continue
  [ "$p" = "$parent" ] && continue
  cmdline=$(tr '\0' ' ' < "/proc/$p/cmdline" 2>/dev/null || true)
  case "$cmdline" in
    *stop_live.sh*) continue ;;
  esac
  kill -9 "$p" 2>/dev/null
done

sleep 1
echo "停止完了。残存:"
pgrep -x vlc 2>/dev/null || echo "  vlc: なし"
pgrep -x ffplay 2>/dev/null || echo "  ffplay: なし"
pgrep -x stream_decode 2>/dev/null || echo "  stream_decode: なし"
pgrep -x rtl_sdr 2>/dev/null || echo "  rtl_sdr: なし"
pgrep -f "ts_http_server\.py" 2>/dev/null || echo "  ts_http_server: なし"
