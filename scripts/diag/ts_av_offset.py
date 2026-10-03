#!/usr/bin/env python3
"""MPEG-TS の中で映像と音声の時刻を直接比べ、A/V オフセットを求める。

「ffplay の A-V 表示は小さいのに音が数秒遅く聞こえる」问题时、
デコーダの_Yield_之外で **TS 内部の PTS 自体がずれている**可能性がある。
ffplay の A-V 表示は既にバッファ分で補正した値なので、原始データを見る。

PES ヘッダ構造（p = payload 開始位置）:
    p+0: 0x00
    p+1: 0x00
    p+2: 0x01        ← start_code_prefix
    p+3: stream_id   ← 0xE0-0xEF=映像, 0xC0-0xDF=音声
    p+4: PES_packet_length (2 bytes, big endian)
    p+6: flags1 '10' + flags2
         flags2 の上位 2 bit = PTS_DTS_flags
           2 (0b10) = PTS のみ
           3 (0b11) = PTS + DTS
    p+7: PES_header_data_length (1 byte)
    p+8: PTS (5 bytes, 33 bit / 90 kHz)

stream_id が 0xBC (PES 拡張) の場合は構造が違うのでスキップする。

PTS は 33 bit なので 2^33/90000 ≈ 95443.7 秒で折り返す。差は mod 演算で
最短を取る。
"""
import sys

TS = sys.argv[1] if len(sys.argv) > 1 else "/tmp/av_full.ts"


def parse_ts(b):
    """5 バイトの PTS/DTS フィールド → 90 kHz カウンタ。"""
    return (((b[0] >> 1) & 0x07) << 30 | b[1] << 22 |
            ((b[2] >> 1) & 0x7F) << 15 | b[3] << 7 | (b[4] >> 1) & 0x7F)


def wrap_diff(a, b):
    """33 bit 折り返しを考慮した a - b（秒）。"""
    d = (a - b) & ((1 << 33) - 1)
    if d >= (1 << 32):
        d -= (1 << 33)
    return d / 90000.0


def main():
    data = open(TS, "rb").read()
    n_pkt = len(data) // 188
    print(f"入力: {TS}  {len(data)} B  = {n_pkt} パケット")

    # PID -> [種別, 最初のPTS, 個数, 最後のPTS, 最初のDTS]
    info = {}
    pcr = {}
    adapt = 0
    bad_sync = 0

    for i in range(n_pkt):
        o = i * 188
        if data[o] != 0x47:
            bad_sync += 1
            continue
        b1, b2, b3 = data[o + 1], data[o + 2], data[o + 3]
        pusi = (b1 >> 6) & 1
        pid = ((b1 & 0x1F) << 8) | b2
        afc = (b3 >> 4) & 3
        p = o + 4
        if afc in (2, 3):
            adapt += 1
            af_len = data[o + 4]
            if af_len >= 7 and (data[o + 5] & 0x10):
                b = data[o + 6:o + 12]
                base = (b[0] << 25 | b[1] << 17 | b[2] << 9 |
                        b[3] << 1 | b[4] >> 7)
                ext = (b[4] & 1) << 8 | b[5]
                pcr.setdefault(pid, base + ext / 300.0)
            p += 1 + af_len
        if not pusi or pid in (0x0000, 0x1FFF, 0x1FC8):
            continue
        if p + 9 > o + 188:
            continue
        if data[p] != 0 or data[p + 1] != 0 or data[p + 2] != 0x01:
            continue
        sid = data[p + 3]
        if not (0xC0 <= sid <= 0xEF):
            continue
        flags2 = data[p + 6]
        hdr_len = data[p + 7]
        pts_flags = (flags2 >> 6) & 3
        if pts_flags not in (2, 3):
            continue
        q = p + 8
        if q + 5 > o + 188:
            continue
        pts = parse_ts(data[q:q + 5])
        dts = None
        if pts_flags == 3 and q + 10 <= o + 188:
            dts = parse_ts(data[q + 5:q + 10])
        kind = "video" if sid >= 0xE0 else "audio"
        e = info.setdefault(pid, [kind, None, 0, None, None])
        if e[1] is None:
            e[1], e[4] = pts, dts
        e[3] = pts
        e[2] += 1

    print(f"sync byte 不正: {bad_sync} / {n_pkt}")
    print(f"adaptation field あり: {adapt}")
    print()
    hdr = (f"{'PID':>6s} {'種別':>6s} {'PTS数':>7s} {'最初のPTS':>13s} "
           f"{'最後のPTS':>13s} {'長さ(s)':>9s} {'PCR(s)':>11s}")
    print(hdr)
    for pid in sorted(info):
        kind, first, cnt, last, dts = info[pid]
        span = wrap_diff(last, first) if first is not None else 0.0
        pc = pcr.get(pid)
        print(f"0x{pid:04X} {kind:>6s} {cnt:7d} {first if first is not None else -1:13d} "
              f"{last if last is not None else -1:13d} {span:9.3f} "
              f"{('%.3f' % pc) if pc is not None else '-':>11s}")

    vid = [(p, e) for p, e in info.items() if e[0] == "video"]
    aud = [(p, e) for p, e in info.items() if e[0] == "audio"]
    print()
    if not vid or not aud:
        print("映像または音声の PTS が取れなかった")
        return 1
    vpid, ve = vid[0]
    apid, ae = aud[0]
    print(f"映像 PID 0x{vpid:04X} / 音声 PID 0x{apid:04X}")
    vp, ap = pcr.get(vpid), pcr.get(apid)
    if vp is not None and ap is not None:
        print(f"  PCR 時刻差 (音 - 画) = {ap - vp:+.3f} s")
    print(f"  最初の PTS 差 (音 - 画) = {wrap_diff(ae[1], ve[1]):+.3f} s")
    print(f"  最後の PTS 差 (音 - 画) = {wrap_diff(ae[3], ve[3]):+.3f} s")
    if ve[4] is not None:
        print(f"  最初の DTS 差 (音 - 画) = {wrap_diff(ae[4], ve[4]):+.3f} s")
    print()
    print("判定:")
    print("  差が -4.0 s 付近 → TS 内の PTS 自体が 4 秒ずれている（復調側の問題）")
    print("  差が  0.0 s 付近 → TS は正しい。遅延は再生側/音声バッファの側")
    return 0


if __name__ == "__main__":
    sys.exit(main())