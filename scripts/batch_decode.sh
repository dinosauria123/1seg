#!/usr/bin/env bash
# 1seg IQ を分割してバッチデコードし、TS を連結する。
#
# 実測 2026-09-26: 同じ 80MB の IQ を 1 本で連続処理すると RS 訂正不能率が
# 24% に達し、H.264 の I フレームが欠落して「再生時間とともに画像が崩れる」。
# 一方 20MB ずつに分割して別々にデコードすると合計 1.34%、40MB ずつで 1.90%。
# IQ データ自体は正常で、連続処理の状態だけが汚染される。
# （Viterbi トレリスの連続性が必要なので、内部での再ロックは不可能。
#   ViterbiStreaming::reset() を挟むと 100% 壊れることを実測済み。）
#
# よって IQ を分割して 1 段ずつ独立デコードし、TS を simply 連結する。
# 分割境界は OFDM フレーム境界（204 シンボル = 235008 サンプル）に合わせ、
# 各断片が独立にロック・復調できるようにする。
set -euo pipefail

IQ="${1:?用法: batch_decode.sh <input.iq> [output.ts] [chunk_MB]}"
OUT="${2:-${IQ%.iq}.ts}"
# 既定 64 MB。ロックに必要な初期サンプル数は
# `(LOCK_SYMS + 8) * 1280 + 60_000` = 5,190,240 サンプル = **39.6 MB**。
# これを下回るチャンクでは同期探索が完結せず「暖機のみ」で TS が出ない
# （実測: 20MB チャンク 4 個すべてで TS 0 バイト）。
# 暖機ぶんを丸ごと捨てるので、実質적으로 1 チャンクから 25 MB 程度が TS になる。
CHUNK_MB="${3:-64}"
BIN=./target/release/examples/stream_decode

SYM=1152          # 1seg mode 3: FFT 1024 + GI 1/8 (128)
FRAME_SAMPLES=$((204 * SYM))   # 235008
CHUNK_MB=$((CHUNK_MB * 1024 * 1024))
CHUNK=$(( (CHUNK_MB / FRAME_SAMPLES) * FRAME_SAMPLES ))   # フレーム境界に合わせる

[ -f "$IQ" ] || { echo "入力が見つかりません: $IQ" >&2; exit 1; }
[ -x "$BIN" ] || { echo "バイナリがありません: $BIN（cargo build --release を実行）" >&2; exit 1; }

SIZE=$(stat -c%s "$IQ")
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

echo "入力 : $IQ ($((SIZE / 1024 / 1024)) MB)"
echo "出力 : $OUT"
echo "単位 : $((CHUNK / 1024 / 1024)) MB（OFDM フレーム境界）"
echo

PART=0
OFF=0
: > "$OUT"
rm -f "$TMP"/part*.ts

while [ "$OFF" -lt "$SIZE" ]; do
    PART=$((PART + 1))
    LEN=$CHUNK
    [ $((OFF + LEN)) -gt "$SIZE" ] && LEN=$((SIZE - OFF))

    CHUNK_IQ="$TMP/p$PART.iq"
    # `bs=1` は 1 バイトずつ read/write するので 80MB で数分かかる。
    # `iflag=skip_bytes,count_bytes` を使うとブロックサイズを保ったまま
    # バイト単位で位置指定できる。
    dd if="$IQ" of="$CHUNK_IQ" bs=1M \
       iflag=skip_bytes,count_bytes skip="$OFF" count="$LEN" status=none

    CHUNK_TS="$TMP/part$PART.ts"
    # ISDBT_DEBUG は出さない（診断出力で stderr がmixed になる）。
    # 分割するため連続処理の状態汚染は起きない。
    if ! "$BIN" "$CHUNK_IQ" "$CHUNK_TS" --live 2>"$TMP/log$PART"; then
        echo "  part$PART: デコード失敗（$((OFF / 1024 / 1024)) MB 地点）" >&2
        tail -3 "$TMP/log$PART" >&2
    fi

    if [ -s "$CHUNK_TS" ]; then
        cat "$CHUNK_TS" >> "$OUT"
        BYTES=$(stat -c%s "$CHUNK_TS")
        echo "  part$PART @ $((OFF / 1024 / 1024)) MB : TS $BYTES バイト"
    else
        echo "  part$PART @ $((OFF / 1024 / 1024)) MB : TS なし（ロック失敗 or 暖機のみ）"
    fi

    OFF=$((OFF + LEN))
done

echo
echo "完了: $OUT ($(( $(stat -c%s "$OUT") / 1024 )) KB, part $PART 個を連結)"
