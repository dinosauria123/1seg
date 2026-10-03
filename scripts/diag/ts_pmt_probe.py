#!/usr/bin/env python3
"""PMT を実際にパースして PCR_PID と PID 種別（映像/音声）を確定する。

なぜ必要か
----------
PID を stream_id（0xE0-0xEF=映像 / 0xC0-0xDF=音声）で推測すると誤判定する。
実測 2026-10-04: 同じ放送の 2 つの PID が 3.31 時間離れた PTS を持ち、
「A/V が 11930 秒ずれている」と結論した。。TS 内��� PUSI ペイロードが
PES ヘッダではなく adaptation field の後ろ過ぎて、PES 境界を跨いだ
パケットを誤読した可能性 homotopy がある。

判定は PMT（PID 0x1FC8）の stream_type に委ねる:
    0x1b = H.264/AVC  → 映像
    0x0f = AAC ADTS   → 音声（ただし LATM の場合は 0x11）
    PCR_PID は PMT の program_info 直後に 13 bit で格納されている

使い方:
  python3 ts_pmt_probe.py <ts ファイル>
"""
import sys


def parse_pmt(sec):
    """PMT セクション（table_id=0x02 のペイロード）を(dict) にする。"""
    if len(sec) < 12 or sec[0] != 0x02:
        return None
    program_number = (sec[3] << 8) | sec[4]
    pcr_pid = ((sec[8] & 0x1F) << 8) | sec[9]
    prog_info_len = ((sec[10] & 0x0F) << 8) | sec[11]
    off = 12 + prog_info_len
    streams = []
    while off + 5 <= len(sec):
        stype = sec[off]
        epid = ((sec[off + 1] & 0x1F) << 8) | sec[off + 2]
        es_info_len = ((sec[off + 3] & 0x0F) << 8) | sec[off + 4]
        streams.append((stype, epid, es_info_len))
        off += 5 + es_info_len
    return {"program": program_number, "pcr_pid": pcr_pid, "streams": streams}


STYPE = {
    0x02: "MPEG-2 Video",
    0x0F: "AAC ADTS",
    0x11: "LATM AAC",
    0x1B: "H.264/AVC",
    0x06: "private (PES payload)",
    0x24: "HEVC",
    0x0D: "private (pointer field)",
    0x1C: "loopback",
}


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/live_test.ts"
    data = open(path, "rb").read()
    n_pkt = len(data) // 188
    print(f"入力: {path}  {len(data)} B = {n_pkt} パケット")

    pmt_found = None
    pmts = 0
    for i in range(n_pkt):
        o = i * 188
        if data[o] != 0x47:
            continue
        b1, b2 = data[o + 1], data[o + 2]
        pid = ((b1 & 0x1F) << 8) | b2
        if pid != 0x1FC8:
            continue
        if not ((b1 >> 6) & 1):
            continue
        afc = (data[o + 3] >> 4) & 3
        p = o + 4
        if afc in (2, 3):
            p += 1 + data[o + 4]
        # pointer_field
        if p >= o + 188:
            continue
        ptr = data[p]
        p += 1 + ptr
        if p + 1 >= o + 188:
            continue
        sec_len = ((data[p + 1] & 0x0F) << 8) | data[p + 2]
        sec = data[p:p + 3 + sec_len]
        info = parse_pmt(sec)
        if info:
            pmts += 1
            if pmt_found is None:
                pmt_found = info

    if not pmt_found:
        print("PMT が見つからない")
        return 1
    print(f"PMT を {pmts} 回観測。最初の 1 つを使う。")
    print()
    print(f"  program_number = {pmt_found['program']}")
    print(f"  PCR_PID        = 0x{pmt_found['pcr_pid']:04X}")
    print()
    print("  ES (elementary streams):")
    print(f"  {'PID':>7s}  {'stream_type':>12s}  {'説明':<22s} es_info_len")
    for stype, epid, es_info_len in pmt_found["streams"]:
        name = STYPE.get(stype, f"unknown(0x{stype:02X})")
        print(f"  0x{epid:04X}   0x{stype:02X}        {name:<22s} {es_info_len}")

    # さらに PID ごとの実測（パケット数と PUSI 数）を出す
    print()
    cnt = {}
    pusi = {}
    for i in range(n_pkt):
        o = i * 188
        if data[o] != 0x47:
            continue
        b1, b2 = data[o + 1], data[o + 2]
        pid = ((b1 & 0x1F) << 8) | b2
        cnt[pid] = cnt.get(pid, 0) + 1
        if (b1 >> 6) & 1:
            pusi[pid] = pusi.get(pid, 0) + 1
    print("  PID ごとのパケット数:")
    declared = {pcr_pid for _, pcr_pid, _ in pmt_found["streams"]}
    declared.add(pmt_found["pcr_pid"])
    for pid in sorted(cnt, key=lambda p: -cnt[p])[:10]:
        mark = " ← PMT 宣言" if pid in declared else ""
        print(f"    0x{pid:04X}  {cnt[pid]:7d} 個  PUSI {pusi.get(pid, 0):5d}{mark}")
    return 0


if __name__ == "__main__":
    sys.exit(main())