#!/usr/bin/env python3
"""実際の ISDB-T 1seg チャネル中心周波数で TMCC ロックを確認する。

前版 (`sinr_survey.py`) は 1 MHz グリッドのピーク検出を使っていたが、
1seg は 429 kHz 幅で 6 MHz チャネルの中心に載るため、ピーク＝信道という
仮定が成り立たない（全 17 peak が TMCC ロック NG、しかし 485.142857 MHz は
確実にロックする）。ここは信道中心周波数に直接 Tex して判定する。

日本の地上デジタル: ch n の中心は 473.142857 + 6*(n-15) MHz。
中心は .142857 なので、1 MHz グリッドでは_mat_乗らない。
"""
import subprocess, os, re, sys

FS = 1015873
PROBE = "./target/release/examples/tmcc_probe"
BIN = "./target/release/examples/stream_decode"

# 日本の ISDB-T 1seg 割当（ARIB STD-B31 / 総務省）
# 参考: 実測で確実にロックする 485.142857 MHz（札幌 NHK 総合）も含める。
CHANNELS = {
    25: 525.142857, 26: 531.142857, 27: 537.142857, 28: 543.142857,
    29: 549.142857, 30: 555.142857,
    # 札幌で実際にaviaある 1seg 割当（測定で決めるため候補を並べる）
    "札幌15": 485.142857,
    "札幌16": 491.142857,
    "札幌17": 473.142857,
    "札幌18": 503.142857,
    "札幌19": 509.142857,
    "札幌20": 515.142857,
    "札幌21": 521.142857,
    "札幌22": 479.142857,
}


def capture(freq_mhz, sec):
    hz = int(round(freq_mhz * 1_000_000))
    p = subprocess.run(
        ["timeout", str(sec), "rtl_sdr", "-f", str(hz), "-s", str(FS), "-g", "0", "-"],
        capture_output=True, timeout=sec + 20)
    return p.stdout


def probe_lock(iq, path="/tmp/_lock.iq"):
    with open(path, "wb") as fh:
        fh.write(iq)
    r = subprocess.run([PROBE, path, str(FS)],
                       capture_output=True, text=True, timeout=300)
    yes = "真のロックか          : YES" in r.stdout
    sync = re.search(r"同期語一致\s*:\s*([\d/]+)\s*\(([\d.]+)%\)", r.stdout)
    return yes, (sync.group(2) if sync else "?")


def measure_sind(iq, env):
    """RS の実測値を返す。

    **注意**: 訂正不能率の分母は「RS 復号器が受け取ったブロック総数」
    （訂正不要を含む = `総blk=`）を使う。`訂正blk` は synd が非ゼロだった
    ブロックのみなので、訂正不要のブロックを含んでおらず分母にならない。
    以前のスクリプトは `rs_err` の第 2 値（訂正ビット総数）をブロック数と
    誤読して synd/blk を計算していた。
    """
    with open("/tmp/_lock.iq", "wb") as fh:
        fh.write(iq)
    r = subprocess.run([BIN, "/tmp/_lock.iq", "/dev/null", "--live"],
                       capture_output=True, text=True, env=env, timeout=900)
    corrected = bits = seen = drop = 0
    for ln in r.stderr.splitlines():
        m = re.search(
            r"訂正blk=(\d+) 訂正bit=(\d+) 総blk=(\d+) drop=(\d+)", ln)
        if m:
            corrected, bits, seen, drop = (int(m.group(i)) for i in range(1, 5))
    return corrected, bits, seen, drop


def main():
    env = dict(os.environ, ISDBT_DEBUG="1")
    sec = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    # 485.142857 は確実にロックする周波数のはずなのに、前回の測定で
    # TMCC 同期 100% かつ真ロック NG になった（ロックが不安定）。
    # 環境変化の可能性があるので、この周波数だけ複数回測って安定性を見る。
    repeat = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    if repeat:
        print(f"=== 485.142857 MHz を {repeat} 回反復（ロックの安定性確認）===")
        for i in range(repeat):
            iq = capture(485.142857, sec)
            if len(iq) < 500_000:
                print(f"  run{i+1}: キャプチャ失敗"); continue
            yes, sync = probe_lock(iq)
            tag = "OK" if yes else "NG"
            line = f"  run{i+1}: {tag} 同期={sync}"
            if yes:
                c, b, s, d = measure_sind(iq, env)
                bpb = b / c if c else 0.0
                line += (f"  総blk={s} bit/訂正blk={bpb:.2f}"
                         f" drop/総blk={d/s*100 if s else 0:.1f}%")
            print(line)
        print()

    print(f"=== 1seg 信道中心周波数で TMCC ロック確認（{sec} 秒ずつ）===")
    print(f"{'label':>8s} {'MHz':>12s} {'lock':>5s} {'同期%':>6s} "
          f"{'総blk':>8s} {'bit/訂正blk':>12s} {'drop/総blk':>11s}")
    ok = []
    for label, mhz in CHANNELS.items():
        iq = capture(mhz, sec)
        if len(iq) < 500_000:
            print(f"{str(label):>8s} {mhz:12.6f}  (キャプチャ失敗)")
            continue
        yes, sync = probe_lock(iq)
        if not yes:
            print(f"{str(label):>8s} {mhz:12.6f} {'NG':>5s} {sync:>6s}")
            continue
        corrected, bits, seen, drop = measure_sind(iq, env)
        # 訂正ブロックあたりの synd 数（訂正の深さ）。1〜16。
        # 16 に張り付くのは RS の訂正限界ちょうど。
        bpb = bits / corrected if corrected else 0.0
        # 訂正不能率の正しい分母は「復号器が受け取ったブロック総数」。
        dp = drop / seen * 100 if seen else 0.0
        print(f"{str(label):>8s} {mhz:12.6f} {'OK':>5s} {sync:>6s} "
              f"{seen:8d} {bpb:12.2f} {dp:10.1f}%")
        ok.append((label, mhz, bpb, dp))
    print(f"\nロック成功: {len(ok)}/{len(CHANNELS)}")
    if ok:
        ok.sort(key=lambda x: x[3])
        print("drop/総blk が低い順に:")
        for label, mhz, bpb, dp in ok:
            print(f"  {label:>8s} {mhz:12.6f}  "
                  f"bit/訂正blk={bpb:.2f}  drop/総blk={dp:.1f}%")


if __name__ == "__main__":
    main()
