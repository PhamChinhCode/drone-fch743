"""
Quay thử TỪNG motor qua các mức ga, thu rung 8 kHz suốt quá trình.

    python tools/vib_motor_sweep.py --port COM23
    python tools/vib_motor_sweep.py --port COM23 --motors 1 3 --pct 4 8 12 --hold 4

CHỈ CHẠY KHI ĐÃ THÁO CÁNH QUẠT VÀ GẮN CỐ ĐỊNH MÁY BAY.

Dùng lệnh quay thử có canh chừng (TLM_MSG_CMD_MOTOR_TEST 0x46): mỗi lệnh chỉ
sống timeout_ms, script gửi lại mỗi 150 ms. Script chết, rút cáp, Ctrl+C...
motor đều tự dừng sau tối đa 0,5 s. Firmware kẹp ga ở DSHOT_TEST_MAX_PERCENT
và từ chối khi đang ARM.

Mỗi đoạn nhận diện được ngay trong dữ liệu (cột motor là giá trị DShot đang
phát), nên file .npz phân tích được bằng vib_analyze.py như file thường.
Đoạn 'nghỉ' giữa hai mức cho motor dừng hẳn để có phổ nền.
"""
import argparse
import struct
import sys
import time
from pathlib import Path

import serial

from vib_capture import (MSG_CLI_LINE, MSG_VIB, VIB_PAYLOAD, Parser, default_out,
                         encode, save_frames, send_cli)

MSG_ACK = 0x0E
MSG_CMD_MOTOR_TEST = 0x46
MOTOR_TEST_STOP = 0xFF
ACK_NAMES = {0: "OK", 1: "UNKNOWN", 2: "LENGTH", 3: "RANGE", 4: "ARMED",
             5: "READONLY", 6: "FAILED", 7: "BUSY"}


def motor_cmd(motor, pct, timeout_ms=500):
    return encode(MSG_CMD_MOTOR_TEST, struct.pack("<BBH", motor, pct, timeout_ms))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM23")
    ap.add_argument("--motors", type=int, nargs="+", default=[1, 2, 3, 4],
                    help="số motor 1..4")
    ap.add_argument("--pct", type=int, nargs="+", default=[4, 8, 12],
                    help="các mức ga quay thử, %% trên mức idle")
    ap.add_argument("--hold", type=float, default=4.0, help="giây mỗi mức")
    ap.add_argument("--rest", type=float, default=1.5, help="giây nghỉ giữa hai motor")
    ap.add_argument("--out")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.01)
    try:
        ser.set_buffer_size(rx_size=1 << 20)
    except Exception:
        pass
    parser = Parser()
    frames = []
    capture = [False]
    rejected = []

    def pump(duration, motor=None, pct=0):
        t_end = time.perf_counter() + duration
        next_cmd = 0.0
        while time.perf_counter() < t_end:
            now = time.perf_counter()
            if motor is not None and now >= next_cmd:
                ser.write(motor_cmd(motor, pct))
                next_cmd = now + 0.15
            data = ser.read(max(1, ser.in_waiting))
            for mid, pl in parser.feed(data):
                if mid == MSG_VIB and capture[0] and len(pl) == VIB_PAYLOAD:
                    frames.append(bytes(pl))
                elif mid == MSG_ACK and len(pl) >= 4:
                    cmd, res, detail = struct.unpack_from("<BBH", pl)
                    if cmd == MSG_CMD_MOTOR_TEST and res != 0:
                        rejected.append((motor, pct, ACK_NAMES.get(res, res)))
                elif mid == MSG_CLI_LINE and len(pl) > 3:
                    line = pl[3:].split(b"\0", 1)[0].decode("ascii", "replace")
                    if line:
                        print("FC>", line)

    try:
        send_cli(ser, "port here")
        pump(0.3)
        send_cli(ser, "vib on")
        pump(0.3)
        capture[0] = True
        t0 = time.perf_counter()

        print("nền: mọi motor dừng")
        pump(args.rest + 1.0)
        for m in args.motors:
            # Tăng ga liền mạch qua các mức, chỉ dừng khi chuyển sang motor khác —
            # để lần theo được đỉnh nào chạy theo tốc độ quay.
            for pct in args.pct:
                print(f"motor {m}  {pct:2d} %")
                pump(args.hold, m - 1, pct)
            ser.write(motor_cmd(MOTOR_TEST_STOP, 0))
            pump(args.rest)
        wall = time.perf_counter() - t0
    finally:
        ser.write(motor_cmd(MOTOR_TEST_STOP, 0))
        send_cli(ser, "vib off")
        capture[0] = False
        pump(0.3)
        ser.close()

    if rejected:
        print("FC TỪ CHỐI lệnh quay thử:", sorted(set(rejected)))
    if not frames:
        print("Không nhận được khung VIB nào.")
        return 1
    save_frames(frames, parser, wall, Path(args.out) if args.out else default_out("sweep"))
    return 0 if not rejected else 2


if __name__ == "__main__":
    sys.exit(main())
