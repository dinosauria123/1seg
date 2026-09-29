#!/usr/bin/env python3
"""状態汚染の decisive テスト。

同じ IQ を 3 通りでデコードし、`総blk`（RS 復号器が受け取った総数）と
`drop` を厳密に比較する。

  A: h1 単独
  B: h2 単独
  C: h1 + h2 を連結した rec_diag.iq

判定:
  A.drop + B.drop ≈ C.drop  →  状態は独立（IQ 自体に欠陥）
  A.drop + B.drop << C.drop →  状態が h1 から h2 に持ち越されて汚染される

**注意**: `終了: N/N (100.0%)` の N は**成功ブロック数のみ**を数える
（`nblk` は成功分岐でだけ加算される）。訂正不能は `rs_dropped` 側に付く。
よって「100.0%」と表示されても訂正不能は起きている。必ず `総blk` と
`drop` を併せて読むこと。
"""
import os, re, subprocess, sys

BIN = "./target/release/examples/stream_decode"
env = dict(os.environ, ISDBT_DEBUG="1")


def decode(path):
    r = subprocess.run([BIN, path, "/dev/null", "--live"],
                       capture_output=True, text=True, env=env, timeout=2400)
    last = None
    for ln in r.stderr.splitlines():
        m = re.search(r"訂正blk=(\d+) 訂正bit=(\d+) 総blk=(\d+) drop=(\d+)", ln)
        if m:
            last = tuple(int(m.group(i)) for i in range(1, 5))
    return last


for tag, path in [("A h1 単独", "/tmp/h1.iq"),
                  ("B h2 単独", "/tmp/h2.iq"),
                  ("C 連結(rec_diag)", sys.argv[1] if len(sys.argv) > 1
                   else "/tmp/rec_diag.iq")]:
    got = decode(path)
    if not got:
        print(f"{tag:20s} データなし"); continue
    c, b, s, d = got
    print(f"{tag:20s} 総blk={s:6d} 訂正blk={c:6d} drop={d:6d} "
          f"drop/総blk={d/s*100 if s else 0:6.2f}%")
