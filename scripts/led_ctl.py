#!/usr/bin/env python3
"""DS-DT-305BK の緑 LED を GPIO で点灯/消灯する。

実測（2026-10-04、ch19 HBC）:
  全 8 GPIO を Hi に**固定**すると緑 LED が点灯する。1本ずつ Hi/Lo を
  高速で切り替える 방식では観察できなかった（点灯/消灯を繰り返すため）。

  GPIO を Lo に戻すと消灯する（電源ランプではなく GPIO 制御と判定）。

用途:
  1seg 受信機の動作状態を LED で示す。

    点灯  = ドングルが給電され myst/j Pill 動作中
    消灯  = 停止中

注意:
  - librtlsdr の `rtlsdr_set_bias_tee_gpio()` を使い、8bit GPIO の
    出力を直接駆動する。Bias Tee（外部 LNA 給電）用の API だが、
    GPIO の out レジスタを叩くので LED にも効く。
  - デバイスは**排他**。rtl_sdr / stream_decode が掴んでいると
    `open` が失敗する。`stop_live.sh` で解放してから使う。
  - GPIO は 8 本をまとめて Hi にする。1本だけだと点灯しない（実測）。

使い方:
  python3 scripts/led_ctl.py on     # 点灯（保持。Ctrl-C で終了）
  python3 scripts/led_ctl.py off    # 消灯して終了
  python3 scripts/led_ctl.py blink  # 0.5 秒間隔で点滅（動作確認用）
  python3 scripts/led_ctl.py probe  # 1 本ずつ Hi にし、点灯する GPIO を特定
"""
import ctypes
import sys
import time

FREQ = 509_142_857
RATE = 1_015_873
N_GPIO = 8


def open_dev():
    lib = ctypes.CDLL("librtlsdr.so.0")
    lib.rtlsdr_get_device_count.restype = ctypes.c_uint32
    lib.rtlsdr_get_device_name.argtypes = [ctypes.c_uint32]
    lib.rtlsdr_get_device_name.restype = ctypes.c_char_p
    lib.rtlsdr_open.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.rtlsdr_set_center_freq.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.rtlsdr_set_sample_rate.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    lib.rtlsdr_set_bias_tee_gpio.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    lib.rtlsdr_close.argtypes = [ctypes.c_void_p]

    if lib.rtlsdr_get_device_count() == 0:
        return None, None
    dev = ctypes.c_void_p()
    if lib.rtlsdr_open(ctypes.byref(dev), 0) != 0:
        return None, None
    lib.rtlsdr_set_center_freq(dev, FREQ)
    lib.rtlsdr_set_sample_rate(dev, RATE)
    time.sleep(0.3)
    return lib, dev


def set_all(lib, dev, on):
    for g in range(N_GPIO):
        lib.rtlsdr_set_bias_tee_gpio(dev, g, on)


def set_and_release(on, hold=0.5):
    """GPIO を設定して**すぐデバイスを解放**する。

    **デバイスは排他**（実測 2026-10-04）:
    このスクリプトが `open` いている間、`rtl_sdr` / `stream_decode` は
    `usb_claim_interface error -6` で失敗する。だから常駐させられない。

    排他回避の二段構え:
      1. `rtlsdr_open()` で GPIO を Hi/Lo に設定
      2. `rtlsdr_close()` で即解放

    `rtlsdr_close()` は USB ハンドルを閉じるだけで、**demod の GPIO
    レジスタ状態は保持される**。実測 2026-10-04: Hi 設定 → close →
    30 秒後も点灯したまま。

    注意: **rtl_sdr が起動すると demod を再初期化するので GPIO は
    リセットされる**。led_ctl.py を `live_play_direct.sh` の
    rtl_sdr 起動**後**に呼ぶ必要がある。
    """
    lib, dev = open_dev()
    if lib is None or dev is None:
        return False
    set_all(lib, dev, on)
    time.sleep(hold)
    lib.rtlsdr_close(dev)
    return True


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "on"
    if mode not in ("on", "off", "set", "unset", "blink", "probe"):
        print(__doc__)
        return 1

    lib, dev = open_dev()
    if lib is None or dev is None:
        print("ドングルの open に失敗。", file=sys.stderr)
        print("  stop_live.sh で他のプロセスを止めてから再実行。", file=sys.stderr)
        return 1

    try:
        if mode == "on":
            set_all(lib, dev, 1)
            print("全 GPIO=Hi（緑 LED 点灯）。Ctrl-C で Lo に戻して終了。", flush=True)
            while True:
                time.sleep(1)

        if mode == "off":
            set_all(lib, dev, 0)
            print("全 GPIO=Lo（緑 LED 消灯）", flush=True)
            time.sleep(1)

        if mode == "set":
            # Hi に設定して**すぐ解放**。点灯は保持される（実測 30 秒）。
            set_all(lib, dev, 1)
            time.sleep(0.4)
            print("全 GPIO=Hi に設定しデバイスを解放（LED は点いたまま）", flush=True)

        if mode == "unset":
            set_all(lib, dev, 0)
            time.sleep(0.4)
            print("全 GPIO=Lo に設定しデバイスを解放（LED 消灯）", flush=True)

        if mode == "blink":
            print("0.5 秒間隔で点滅（Ctrl-C で終了）", flush=True)
            while True:
                set_all(lib, dev, 1)
                time.sleep(0.5)
                set_all(lib, dev, 0)
                time.sleep(0.5)

        if mode == "probe":
            print("1 本ずつ Hi にします。点灯する GPIO を特定します。\n", flush=True)
            for g in range(N_GPIO):
                set_all(lib, dev, 0)
                time.sleep(0.3)
                lib.rtlsdr_set_bias_tee_gpio(dev, g, 1)
                print(f"  GPIO{g}=Hi (他は Lo)  ← LED は点いた？", flush=True)
                time.sleep(2.0)
            set_all(lib, dev, 0)
            print("\n完了。点灯した GPIO の番号を教えてください。")
    except KeyboardInterrupt:
        print("\n中断。", flush=True)
    finally:
        if mode in ("on", "blink", "probe"):
            set_all(lib, dev, 0)
        lib.rtlsdr_close(dev)
    return 0


if __name__ == "__main__":
    sys.exit(main())