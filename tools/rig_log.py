"""
Ghi log trên GIÁ THỬ MỘT TRỤC qua USB, kèm chốt tự ngắt phía máy tính.

    python tools/rig_log.py --port COM23 --seconds 30 --imu-hz 25 --out Log/rig_imu.npz
    python tools/rig_log.py --port COM23 --seconds 120 --kill-deg 40 --out Log/rig_pid1.npz

Bật các luồng telemetry ở nhịp cao (PID 1 ms, ATTITUDE 1 ms, MOTOR 2 ms,
IMU 2 ms, RC 20 ms), ghi lại, rồi trả về nhịp cũ khi thoát.

CHỐT TỰ NGẮT (chỉ có tác dụng với firmware FC_RIG_TEST, nơi có lệnh `kill`):
    |roll| hoặc |pitch| > --kill-deg   khi đang ARM
    |gyro| một trục    > --kill-dps   khi đang ARM
    -> gửi `kill` lặp lại cho tới khi FC báo đã disarm.
Firmware còn tự cắt ở RIG_MAX_TILT_DEG (45°) dù script có chạy hay không.

--imu-hz > 0: hỏi lệnh CLI `imu` liên tục để lấy cả IMU2 (LSM6DSV).
--say "giây:câu" (lặp được): máy tính đọc to câu tiếng Anh tại giây đó.
"""
import argparse
import struct
import sys
import threading
import time
from pathlib import Path

import numpy as np
import serial

from vib_capture import (MSG_CLI_LINE, MSG_VIB, VIB_PAYLOAD, Parser, default_out, encode,
                         save_frames, send_cli)

MSG_ATT, MSG_IMU, MSG_RC, MSG_MOTOR, MSG_PID = 0x02, 0x03, 0x06, 0x07, 0x09
MSG_SET_RATE = 0x40
PERIODS = {MSG_PID: 1, MSG_ATT: 1, MSG_MOTOR: 2, MSG_IMU: 2, MSG_RC: 20}
DEFAULT = {MSG_PID: 0, MSG_ATT: 100, MSG_MOTOR: 0, MSG_IMU: 0, MSG_RC: 500}

FMT = {MSG_ATT: struct.Struct("<6h"), MSG_IMU: struct.Struct("<7hH"),
       MSG_RC: struct.Struct("<8HBbBB"), MSG_MOTOR: struct.Struct("<8HB"),
       MSG_PID: struct.Struct("<9hH")}


def set_rate(ser, mid, ms):
    ser.write(encode(MSG_SET_RATE, struct.pack("<BH", mid, ms)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM23")
    ap.add_argument("--seconds", type=float, default=30)
    ap.add_argument("--imu-hz", type=float, default=0)
    ap.add_argument("--kill-deg", type=float, default=40)
    ap.add_argument("--kill-dps", type=float, default=800)
    ap.add_argument("--say", action="append", default=[])
    ap.add_argument("--vib", action="store_true",
                    help="ghi kèm gyro+accel THÔ 8 kHz (vib on) vào <out>_vib.npz; tắt luồng IMU 500 Hz cho đỡ băng thông")
    ap.add_argument("--out")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.01)
    try:
        ser.set_buffer_size(rx_size=1 << 20)
    except Exception:
        pass
    parser = Parser()
    rec = {k: [] for k in FMT}
    imu_lines = []
    vib_frames = []
    stop = threading.Event()
    state = {"armed": False, "roll": 0.0, "pitch": 0.0, "kill": 0, "gmax": 0.0}
    t0 = time.perf_counter()
    lock = threading.Lock()

    def kill(reason):
        if state["kill"] == 0:
            print(f"\n!!! TỰ NGẮT ({reason}) — gửi kill", flush=True)
        state["kill"] += 1
        with lock:
            send_cli(ser, "kill")

    def reader():
        while not stop.is_set():
            data = ser.read(max(1, ser.in_waiting))
            now = time.perf_counter() - t0
            for mid, pl in parser.feed(data):
                if mid in FMT and len(pl) >= FMT[mid].size:
                    v = FMT[mid].unpack_from(pl)
                    rec[mid].append((now,) + v)
                    if mid == MSG_ATT:
                        state["roll"], state["pitch"] = v[0] / 100, v[1] / 100
                    elif mid == MSG_MOTOR:
                        state["armed"] = bool(v[8] & 1)
                    elif mid == MSG_PID:
                        state["gmax"] = max(abs(x) / 10 for x in v[3:6])
                    if state["armed"] and (abs(state["roll"]) > args.kill_deg or
                                           abs(state["pitch"]) > args.kill_deg):
                        kill(f"nghiêng roll {state['roll']:.0f}° pitch {state['pitch']:.0f}°")
                    elif state["armed"] and state["gmax"] > args.kill_dps:
                        kill(f"gyro {state['gmax']:.0f} °/s")
                elif mid == MSG_VIB and len(pl) == VIB_PAYLOAD:
                    vib_frames.append(bytes(pl))
                elif mid == MSG_CLI_LINE and len(pl) > 3:
                    s = pl[3:].split(b"\0", 1)[0].decode("ascii", "replace")
                    if s.startswith("imu"):
                        imu_lines.append((now, s))
                    elif s:
                        print("FC>", s, flush=True)

    with lock:
        send_cli(ser, "port here")
    time.sleep(0.3)
    for mid, ms in PERIODS.items():
        set_rate(ser, mid, 0 if (args.vib and mid == MSG_IMU) else ms)
    if args.vib:
        with lock:
            send_cli(ser, "vib on")
    th = threading.Thread(target=reader, daemon=True)
    th.start()

    says = sorted((float(s.split(":", 1)[0]), s.split(":", 1)[1]) for s in args.say)
    voice = None
    if says:
        import win32com.client
        voice = win32com.client.Dispatch("SAPI.SpVoice")

    next_imu = 0.0
    next_print = 0.0
    try:
        while (now := time.perf_counter() - t0) < args.seconds:
            if says and now >= says[0][0]:
                voice.Speak(says.pop(0)[1], 1)          # 1 = không chặn
            if args.imu_hz > 0 and now >= next_imu:
                with lock:
                    send_cli(ser, "imu")
                next_imu = now + 1 / args.imu_hz
            if now >= next_print:
                thr = rec[MSG_MOTOR][-1][1:5] if rec[MSG_MOTOR] else (0,) * 4
                print(f"\r t={now:5.1f}s  {'ARM ' if state['armed'] else 'disarm'}  roll {state['roll']:6.1f}°"
                      f"  motor {thr}  ", end="", flush=True)
                next_print = now + 0.5
            time.sleep(0.002)
    except KeyboardInterrupt:
        print("\nCtrl+C")
    finally:
        if args.vib:
            with lock:
                send_cli(ser, "vib off")
        for mid, ms in DEFAULT.items():
            set_rate(ser, mid, ms)
        time.sleep(0.3)
        stop.set()
        th.join(1)
        ser.close()

    names = {MSG_ATT: "att", MSG_IMU: "imu", MSG_RC: "rc", MSG_MOTOR: "motor", MSG_PID: "pid"}
    arrays = {names[k]: np.array(v, dtype=float) for k, v in rec.items() if v}
    if imu_lines:
        rows = []
        for t, s in imu_lines:
            p = s.split()
            try:
                rows.append([t, 1 if p[0] == "imu1" else 2] + [float(x) for x in p[1:10]])
            except (ValueError, IndexError):
                pass
        arrays["imucli"] = np.array(rows)
    out = Path(args.out) if args.out else default_out("rig")
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, **arrays)
    print(f"\nĐã lưu {out}:", {k: v.shape for k, v in arrays.items()}, " kill gửi:", state["kill"])
    if vib_frames:
        save_frames(vib_frames, parser, time.perf_counter() - t0,
                    out.with_name(out.stem + "_vib.npz"))


if __name__ == "__main__":
    sys.exit(main())
