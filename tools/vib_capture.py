"""
Thu luồng rung 8 kHz (TLM_MSG_VIB 0x11) từ FC qua USB CDC.

    python tools/vib_capture.py --port COM17 --seconds 20
    python tools/vib_capture.py --port COM17 --seconds 20 --plot

Script tự gửi 'port here' rồi 'vib on' qua chính cổng USB (bản tin CMD_CLI
0x47), thu dữ liệu, rồi gửi 'vib off'. Kết quả lưu vào Log/vib_<thời gian>.npz.

Đọc liên tục, KHÔNG sleep: Windows sleep ~15 ms là đủ để usbser.sys vứt gói
(đã gặp 2026-09-21 với 'flash dump').

Bố cục khung khớp với App/Telemetry/tlm_messages.h (tlm_vib_t).
"""
import argparse
import binascii
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import serial

sys.stdout.reconfigure(encoding="utf-8")   # console Windows mặc định cp1252

SYNC = b"\xFE\x5A"
MSG_CLI_LINE = 0x10
MSG_VIB = 0x11
MSG_CMD_CLI = 0x47

SAMPLES_PER_FRAME = 16
GYRO_LSB_PER_DPS = 16.0
ACCEL_LSB_PER_G = 2000.0

VIB_HDR = struct.Struct("<IIIHHHHBB")          # idx0 t0_us ring_drops motor[4] n flags
VIB_SAMPLE_BYTES = 12
VIB_PAYLOAD = VIB_HDR.size + SAMPLES_PER_FRAME * VIB_SAMPLE_BYTES


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, giống tlm_crc16() trong firmware."""
    return binascii.crc_hqx(data, 0xFFFF)


def encode(msg_id: int, payload: bytes, seq: int = 0) -> bytes:
    body = bytes([len(payload), msg_id, seq & 0xFF]) + payload
    return SYNC + body + struct.pack("<H", crc16(body))


class Parser:
    """Tách khung từ luồng byte. Làm theo khối cho nhanh, không từng byte."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buf += data
        out = []
        b = self.buf
        i = 0
        while True:
            j = b.find(SYNC, i)
            if j < 0:
                # giữ lại byte cuối phòng khi nó là nửa đầu của SYNC
                i = max(i, len(b) - 1)
                break
            if j + 5 > len(b):
                i = j
                break
            ln = b[j + 2]
            end = j + 5 + ln + 2
            if end > len(b):
                i = j
                break
            body = bytes(b[j + 2: j + 5 + ln])
            crc = b[end - 2] | (b[end - 1] << 8)
            if crc16(body) != crc:
                self.crc_errors += 1
                i = j + 1
                continue
            out.append((body[1], body[3:]))
            i = end
        del b[:i]
        return out


def send_cli(ser, text: str):
    ser.write(encode(MSG_CMD_CLI, text.encode("ascii")))
    ser.flush()


def default_out(prefix="vib"):
    return (Path(__file__).resolve().parents[1] / "Log" /
            f"{prefix}_{datetime.now():%Y%m%d_%H%M%S}.npz")


def save_frames(frames, parser, wall, out):
    """Giải mã các khung VIB, in thống kê mất mẫu, lưu .npz."""
    # --- Giải mã ---
    idx, gyro, accel, motor, armed = [], [], [], [], []
    ring_drops_last = 0
    for f in frames:
        i0, t0u, rd, m0, m1, m2, m3, n, flags = VIB_HDR.unpack_from(f, 0)
        ring_drops_last = rd
        raw = np.frombuffer(f, dtype="<i2", count=SAMPLES_PER_FRAME * 6,
                            offset=VIB_HDR.size).reshape(-1, 6)[:n]
        idx.append(np.arange(i0, i0 + n, dtype=np.int64))
        gyro.append(raw[:, 0:3] / GYRO_LSB_PER_DPS)
        accel.append(raw[:, 3:6] / ACCEL_LSB_PER_G)
        motor.append(np.tile([m0, m1, m2, m3], (n, 1)))
        armed.append(np.full(n, flags & 1, dtype=np.uint8))

    idx = np.concatenate(idx)
    # idx là uint32 bên firmware — mở rộng qua điểm tràn nếu có
    idx = np.concatenate([[0], np.cumsum((np.diff(idx) & 0xFFFFFFFF))]) + idx[0]
    gyro = np.concatenate(gyro)
    accel = np.concatenate(accel)
    motor = np.concatenate(motor)
    armed = np.concatenate(armed)

    # Thời gian từng mẫu: khớp tuyến tính t0_us của từng khung theo idx.
    fr_i0 = np.array([VIB_HDR.unpack_from(f, 0)[0] for f in frames], dtype=np.int64)
    fr_t0 = np.array([VIB_HDR.unpack_from(f, 0)[1] for f in frames], dtype=np.int64)
    fr_i0 = np.concatenate([[0], np.cumsum(np.diff(fr_i0) & 0xFFFFFFFF)])
    fr_t0 = np.concatenate([[0], np.cumsum(np.diff(fr_t0) & 0xFFFFFFFF)])
    slope, icpt = np.polyfit(fr_i0, fr_t0, 1)
    fs = 1e6 / slope
    t_s = (np.polyval([slope, icpt], idx - idx[0])) * 1e-6

    gaps = np.diff(idx)
    lost = int(np.sum(gaps[gaps > 1] - 1))
    expected = int(idx[-1] - idx[0] + 1)

    print()
    print(f"Khung nhận      : {len(frames)}")
    print(f"Mẫu nhận        : {len(idx)} / {expected} kỳ vọng")
    print(f"Mẫu mất         : {lost}  ({100.0 * lost / expected:.4f} %)")
    print(f"  do firmware bỏ: {ring_drops_last}")
    print(f"  do USB/PC     : {max(0, lost - ring_drops_last)}")
    print(f"Lỗi CRC         : {parser.crc_errors}")
    print(f"Tần số lấy mẫu  : {fs:.1f} Hz (theo đồng hồ FC)")
    print(f"Tốc độ trung bình: {len(idx) / wall:.0f} mẫu/s, "
          f"{len(frames) * (VIB_PAYLOAD + 7) / wall / 1024:.1f} KB/s")

    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, t_s=t_s, idx=idx, fs=fs,
                        gyro_dps=gyro.astype(np.float32),
                        accel_g=accel.astype(np.float32),
                        motor=motor.astype(np.uint16), armed=armed)
    print(f"Đã lưu: {out}")
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM17", help="cổng USB CDC của FC")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--out", help="file .npz (mặc định Log/vib_<thời gian>.npz)")
    ap.add_argument("--plot", action="store_true", help="vẽ phổ ngay sau khi thu")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.02)
    try:
        ser.set_buffer_size(rx_size=1 << 20)
    except Exception:
        pass
    ser.reset_input_buffer()

    parser = Parser()

    def pump(duration, on_vib=None):
        """Đọc liên tục trong `duration` giây, in dòng CLI, trả khung VIB."""
        t_end = time.perf_counter() + duration
        while time.perf_counter() < t_end:
            data = ser.read(max(1, ser.in_waiting))
            if not data:
                continue
            for mid, pl in parser.feed(data):
                if mid == MSG_CLI_LINE and len(pl) >= 3:
                    line = pl[3:].split(b"\0", 1)[0].decode("ascii", "replace")
                    if line:
                        print("FC>", line)
                elif mid == MSG_VIB and on_vib is not None:
                    on_vib(pl)

    send_cli(ser, "port here")
    pump(0.3)
    send_cli(ser, "vib on")
    pump(0.3)

    frames = []

    def on_vib(pl):
        if len(pl) == VIB_PAYLOAD:
            frames.append(bytes(pl))

    print(f"Đang thu {args.seconds:.1f} s ...")
    t0 = time.perf_counter()
    pump(args.seconds, on_vib)
    wall = time.perf_counter() - t0

    send_cli(ser, "vib off")
    pump(0.3)
    send_cli(ser, "vib")          # in thống kê phía firmware
    pump(0.5)
    ser.close()

    if not frames:
        print("KHÔNG nhận được khung VIB nào. Kiểm tra 'port' và 'vib' qua COM4.")
        return 1

    out = Path(args.out) if args.out else default_out()
    save_frames(frames, parser, wall, out)

    if args.plot:
        from vib_analyze import analyze
        analyze(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
