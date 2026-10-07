"""
Theo dõi trực tiếp nhiễu "sóng vuông" của từ kế để tìm thiết bị gây nhiễu.

    python tools/mag_noise_watch.py --port COM4

Đọc DBG_MODE_MAG (mode 21) qua console UART, mỗi giây in độ nhảy của từng
trục thô trong 2 giây gần nhất (khoảng cách giữa mức cao và mức thấp, mG).
Để máy bay NẰM YÊN, rồi rút / bật từng thứ (ELRS, MTF-01P, thẻ SD, GPS, tay
điều khiển...) và xem cột X có tụt về mức nền (< 10 mG) không.

Đo 2026-10-07: X nhảy ~60 mG, chu kỳ 0,78 s, bật/tắt 50 %; Y ~1, Z ~25 mG.
"""
import argparse
import collections
import sys
import time

import numpy as np
import serial

sys.stdout.reconfigure(encoding="utf-8")
LSB_PER_GAUSS = 3000.0          # QMC5883P dải 8 G


def jump_mg(v):
    """Độ nhảy giữa hai mức: trung bình nửa trên trừ nửa dưới."""
    v = np.asarray(v)
    med = np.median(v)
    hi, lo = v[v > med], v[v <= med]
    if len(hi) == 0 or len(lo) == 0:
        return 0.0
    return 1000.0 * (hi.mean() - lo.mean()) / LSB_PER_GAUSS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    a = ap.parse_args()

    ser = serial.Serial(a.port, 921600, timeout=0)
    ser.write(b"mode 0\r\n")
    time.sleep(0.4)
    ser.reset_input_buffer()
    ser.write(b"mode 21\r\n")
    win = collections.deque(maxlen=40)           # 2 s ở 20 dòng/s
    buf = b""
    t0 = time.perf_counter()
    nxt = t0 + 1.0
    print("   t  |  nhảy X    Y     Z (mG)   |  Ctrl+C để dừng")
    try:
        while True:
            buf += ser.read(65536)
            *lines, buf = buf.split(b"\n")
            for ln in lines:
                p = ln.decode("ascii", "replace").split("|")
                if len(p) >= 6:
                    try:
                        raw = [float(x) for x in p[2].split()]
                    except ValueError:
                        continue
                    if len(raw) == 3:
                        win.append(raw)
            if time.perf_counter() >= nxt and len(win) >= 20:
                w = np.array(win)
                jx, jy, jz = (jump_mg(w[:, i]) for i in range(3))
                bar = "#" * int(min(jx, 100) / 2)
                print(f"{time.perf_counter() - t0:5.0f} | {jx:6.1f} {jy:5.1f} {jz:5.1f}            | {bar}", flush=True)
                nxt += 1.0
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b"mode 0\r\n")
        ser.close()


if __name__ == "__main__":
    main()
