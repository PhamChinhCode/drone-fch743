"""
Hiệu chuẩn từ kế 9 tham số + tự xác định trục, từ console DBG_MODE_MAGAXIS (mode 26).

    python tools/mag_cal9.py --port COM4 --seconds 90      # thu rồi khớp luôn
    python tools/mag_cal9.py --fit Log/magaxis_<...>.npz   # khớp lại file đã thu

Thu: gửi 'mode 26' qua console UART (COM4), đọc từng dòng
    gx gy gz (°/s, hệ thân) | ax ay az (m/s², hệ thân) | msx msy msz (G, THÔ hệ cảm biến) | count
rồi 'mode 0' khi xong. Cột từ kế là raw_gauss — TRƯỚC hiệu chuẩn, nên bộ số
đang lưu trong flash không ảnh hưởng gì tới phép đo.

Khớp:
  1. Ellipsoid đầy đủ (quadric 9 hệ số) -> offset o và ma trận đối xứng S sao cho
     cal = S (raw - o) nằm trên mặt cầu. Đúng công thức trong mag_i2c.c:
     đường chéo S = mag_scale_*, ngoài đường chéo = mag_soft_*. S chuẩn hoá
     det = 1 để |B| giữ đúng độ lớn thô.
  2. Trục: thử cả 24 phép hoán vị có dấu (định thức +1). Đúng trục thì từ
     trường Trái Đất nhìn từ hệ thân thoả  dB/dt = -ω × B  với ω là gyro.
     Kiểm chéo bằng góc chúc từ (giữa B và hướng trọng lực từ accel) — phải
     dương (~20-35° ở Việt Nam) và gần như hằng số.

Cách xoay: lắp máy bay như lúc bay (pin gắn, cánh tháo), xa sắt thép. Xoay
CHẬM và LIÊN TỤC qua mọi hướng — cả lật ngửa, dựng đứng — tới khi phủ 26/26.
Phần thử trục cần có xoay quanh cả ba trục, không chỉ yaw. Thỉnh thoảng
dừng tay yên 2-3 giây ở vài tư thế khác nhau để kiểm được góc chúc.
"""
import argparse
import itertools
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.stdout.reconfigure(encoding="utf-8")

NUM = re.compile(r"[-+]?\d+(?:\.\d+)?")

# 26 hướng: tâm mặt, cạnh, góc của khối lập phương — để đếm độ phủ
DIRS = np.array([d for d in itertools.product((-1, 0, 1), repeat=3) if any(d)], float)
DIRS /= np.linalg.norm(DIRS, axis=1, keepdims=True)
MIN_PER_DIR = 5


def parse_line(s):
    parts = s.split("|")
    if len(parts) != 4:
        return None
    try:
        v = [list(map(float, NUM.findall(p))) for p in parts]
    except ValueError:
        return None
    if [len(x) for x in v] != [3, 3, 3, 1]:
        return None
    return v[0] + v[1] + v[2] + v[3]


def coverage(m):
    """Số hướng đã phủ, dùng tâm thô (max+min)/2."""
    if len(m) < 20:
        return 0
    c = (m.max(0) + m.min(0)) / 2
    d = m - c
    d /= np.linalg.norm(d, axis=1, keepdims=True) + 1e-12
    idx = np.argmax(d @ DIRS.T, axis=1)
    return int(np.sum(np.bincount(idx, minlength=len(DIRS)) >= MIN_PER_DIR))


def capture(port, seconds, out):
    import serial
    ser = serial.Serial(port, 921600, timeout=0.05)
    ser.write(b"mode 0\r\n")
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.write(b"mode 26\r\n")

    rows, t_pc, buf = [], [], b""
    last_cnt = None
    t0 = time.perf_counter()
    next_report = t0 + 1.0
    print(f"Đang thu {seconds:.0f} s — xoay chậm qua MỌI hướng (cả lật ngửa, dựng đứng)...")
    try:
        while time.perf_counter() - t0 < seconds:
            buf += ser.read(4096)
            *lines, buf = buf.split(b"\n")
            for ln in lines:
                r = parse_line(ln.decode("ascii", "replace"))
                if r is None or r[9] == last_cnt:
                    continue          # bỏ dòng tiêu đề và mẫu từ kế lặp lại
                last_cnt = r[9]
                rows.append(r)
                t_pc.append(time.perf_counter() - t0)
            now = time.perf_counter()
            if now >= next_report and rows:
                m = np.array(rows)[:, 6:9]
                print(f"  {now - t0:5.0f} s  mẫu {len(rows):5d}  phủ {coverage(m):2d}/26", flush=True)
                next_report += 1.0
    finally:
        ser.write(b"mode 0\r\n")
        ser.close()

    if len(rows) < 100:
        sys.exit(f"Chỉ được {len(rows)} mẫu — kiểm cổng COM, console có đang ở mode 26 không.")
    d = np.array(rows)
    np.savez(out, t_pc=np.array(t_pc), gyro_dps=d[:, 0:3], acc_mps2=d[:, 3:6],
             mag_raw_g=d[:, 6:9], count=d[:, 9])
    print(f"Đã lưu {out}")
    return out


def fit_ellipsoid(m):
    """Trả về (offset o, S đối xứng det=1, R) với |S(m-o)| ≈ R."""
    x, y, z = m.T
    D = np.c_[x * x, y * y, z * z, 2 * y * z, 2 * x * z, 2 * x * y, 2 * x, 2 * y, 2 * z]
    v, *_ = np.linalg.lstsq(D, np.ones(len(m)), rcond=None)
    a, b, c, f, g, h, p, q, r = v
    A = np.array([[a, h, g], [h, b, f], [g, f, c]])
    o = -np.linalg.solve(A, [p, q, r])
    M = A / (o @ A @ o + 1.0)
    w, V = np.linalg.eigh(M)
    if np.any(w <= 0):
        sys.exit("Khớp ellipsoid suy biến (không xác định dương) — dữ liệu chưa phủ đủ hướng.")
    S1 = V @ np.diag(np.sqrt(w)) @ V.T            # |S1 (m-o)| = 1
    R = np.linalg.det(S1) ** (-1.0 / 3.0)
    return o, S1 * R, R


def fit_diag(m):
    """Khớp trục-thẳng 6 tham số, chỉ để so sánh."""
    x, y, z = m.T
    D = np.c_[x * x, y * y, z * z, 2 * x, 2 * y, 2 * z]
    v, *_ = np.linalg.lstsq(D, np.ones(len(m)), rcond=None)
    a, b, c, p, q, r = v
    o = -np.array([p / a, q / b, r / c])
    k = 1.0 + a * o[0] ** 2 + b * o[1] ** 2 + c * o[2] ** 2
    s = np.sqrt(np.array([a, b, c]) / k)
    s *= np.prod(s) ** (-1 / 3)
    return o, np.diag(s)


def signed_perms():
    """24 ma trận P với field[i] = sign_i * cal[map_i], det(P) = +1."""
    out = []
    for perm in itertools.permutations(range(3)):
        for sg in itertools.product((1, -1), repeat=3):
            P = np.zeros((3, 3))
            for i in range(3):
                P[i, perm[i]] = sg[i]
            if np.linalg.det(P) > 0:
                out.append((perm, sg, P))
    return out


def fit(path):
    d = np.load(path)
    t_pc, g, acc, m, cnt = d["t_pc"], d["gyro_dps"], d["acc_mps2"], d["mag_raw_g"], d["count"]
    n = len(m)
    print(f"\n=== {path}  ({n} mẫu, phủ {coverage(m)}/26) ===")
    if coverage(m) < 20:
        print("!!! Phủ dưới 20/26 hướng — kết quả sắt mềm KHÔNG đáng tin, nên thu lại.")

    raw_n = np.linalg.norm(m, axis=1)
    print(f"|B| thô: TB {raw_n.mean():.3f} G, dao động ±{100 * raw_n.std() / raw_n.mean():.1f} %")

    # Nhiễu lúc đứng yên: các đoạn gyro < 3 °/s kéo dài ≥ 1 s
    wn = np.linalg.norm(g, axis=1)
    noise = []
    i = 0
    while i < n:
        j = i
        while j < n and wn[j] < 3:
            j += 1
        if j - i > 1 and t_pc[j - 1] - t_pc[i] >= 1.0:
            noise.append(m[i:j].std(0))
        i = max(j, i + 1)
    if noise:
        nz = 1000 * np.median(noise, axis=0)
        print(f"Nhiễu lúc đứng yên ({len(noise)} đoạn): {nz[0]:.1f} / {nz[1]:.1f} / {nz[2]:.1f} mG (x/y/z thô)")

    # --- 1. Ellipsoid ---
    o, S, R = fit_ellipsoid(m)
    cal = (m - o) @ S.T
    cn = np.linalg.norm(cal, axis=1)
    od, Sd = fit_diag(m)
    cdn = np.linalg.norm((m - od) @ Sd.T, axis=1)
    print(f"\n[1] Ellipsoid 9 tham số: |B| sau hiệu chuẩn {cn.mean():.3f} G ± {100 * cn.std() / cn.mean():.2f} %"
          f"   (trục-thẳng 6 tham số: ± {100 * cdn.std() / cdn.mean():.2f} %)")
    print(f"    offset  {o[0]:+.4f} {o[1]:+.4f} {o[2]:+.4f} G   (|offset| = {np.linalg.norm(o):.3f} G)")
    print(f"    scale   {S[0, 0]:.4f} {S[1, 1]:.4f} {S[2, 2]:.4f}")
    print(f"    soft    xy {S[0, 1]:+.4f}  xz {S[0, 2]:+.4f}  yz {S[1, 2]:+.4f}")
    bad = []
    if np.any(np.abs(o) > 2.0):
        bad.append("offset vượt ±2 G (giới hạn tham số)")
    if np.any((np.diag(S) < 0.5) | (np.diag(S) > 2.0)):
        bad.append("scale ngoài 0,5-2")
    if np.any(np.abs(S[np.triu_indices(3, 1)]) > 0.5):
        bad.append("soft vượt ±0,5")
    if not 0.25 <= cn.mean() <= 0.65:
        bad.append(f"|B| {cn.mean():.3f} G ngoài dải Trái Đất 0,25-0,65 G (kim loại gần?)")
    if 100 * cn.std() / cn.mean() > 5:
        bad.append("|B| dao động > 5 %")
    if coverage(m) < 20:
        bad.append(f"chỉ phủ {coverage(m)}/26 hướng")
    if noise and nz.max() > 15:
        bad.append(f"nhiễu đứng yên trục {'xyz'[int(np.argmax(nz))]} = {nz.max():.0f} mG (bình thường < 10)"
                   " — có dòng điện bật/tắt gần la bàn?")

    # --- 2. Trục ---
    # Thời gian theo đồng hồ từ kế: count tăng đều ở nhịp lấy mẫu của nó.
    f_mag = (cnt[-1] - cnt[0]) / (t_pc[-1] - t_pc[0])
    t = (cnt - cnt[0]) / f_mag
    w = np.radians(g)
    dt = np.diff(t)
    ok = (dt > 0) & (dt < 0.2)
    dc = (np.diff(cal, axis=0) / dt[:, None])[ok]
    cm = ((cal[1:] + cal[:-1]) / 2)[ok]
    wm = ((w[1:] + w[:-1]) / 2)[ok]
    mov = np.linalg.norm(wm, axis=1) > np.radians(20)
    print(f"\n[2] Trục: nhịp từ kế {f_mag:.1f} Hz, {mov.sum()} cặp mẫu đang xoay (> 20 °/s)")
    if mov.sum() < 50:
        print("!!! Quá ít đoạn xoay — không xác định được trục.")
        return

    # Góc chúc: chỉ lấy lúc gần đứng yên
    still = (np.linalg.norm(w, axis=1) < np.radians(15)) & \
            (np.abs(np.linalg.norm(acc, axis=1) - 9.81) < 0.5)
    down = -acc / np.linalg.norm(acc, axis=1, keepdims=True)

    res = []
    for perm, sg, P in signed_perms():
        db = dc[mov] @ P.T
        pred = -np.cross(wm[mov], cm[mov] @ P.T)
        k = np.sum(db * pred) / np.sum(pred * pred)
        r = np.linalg.norm(db - k * pred) / np.linalg.norm(db)
        b = cal @ P.T
        dip = np.degrees(np.arcsin(np.sum(b * down, axis=1) / np.linalg.norm(b, axis=1)))[still]
        res.append((r, k, perm, sg, dip.mean() if len(dip) else np.nan, dip.std() if len(dip) else np.nan))
    res.sort(key=lambda x: x[0])

    print("    hạng  sai lệch    k   map      sign        chúc TB ± lệch   ({} mẫu đứng yên)".format(still.sum()))
    for i, (r, k, perm, sg, dm, ds) in enumerate(res[:4]):
        print(f"    {i + 1:3d}   {r:7.3f}  {k:5.2f}  {perm}  {sg}  {dm:+6.1f}° ± {ds:4.1f}°")
    r, k, perm, sg, dm, ds = res[0]
    if not (0.7 < k < 1.3):
        bad.append(f"hệ số k = {k:.2f} (phải ≈ 1) — thời gian hoặc trục gyro có vấn đề")
    # Đối thủ thật phải hợp lý về vật lý: k ≈ 1 và góc chúc ổn định. Phương án
    # k âm hay góc chúc lệch hàng chục độ thì sai lệch thấp cỡ nào cũng loại.
    rivals = [x for x in res[1:] if 0.7 < x[1] < 1.3 and (np.isnan(x[5]) or x[5] < 2 * ds + 5)]
    if rivals and rivals[0][0] < 1.15 * r:
        bad.append("có phương án trục khác gần ngang ngửa — xoay thêm quanh cả ba trục")
    if still.sum() < 20:
        bad.append("gần như không có lúc đứng yên — dừng tay vài giây ở vài tư thế để kiểm góc chúc")
    elif not (5 < dm < 50):
        bad.append(f"góc chúc {dm:+.1f}° không hợp lý cho Việt Nam (kỳ vọng ~+20..+35°)")

    # --- 3. Kết quả ---
    print("\n[3] Dán vào console COM4:")
    print(f"set mag_offset_x_g={o[0]:.4f}\nset mag_offset_y_g={o[1]:.4f}\nset mag_offset_z_g={o[2]:.4f}")
    print(f"set mag_scale_x={S[0, 0]:.4f}\nset mag_scale_y={S[1, 1]:.4f}\nset mag_scale_z={S[2, 2]:.4f}")
    print(f"set mag_soft_xy={S[0, 1]:.4f}\nset mag_soft_xz={S[0, 2]:.4f}\nset mag_soft_yz={S[1, 2]:.4f}")
    for i, ax in enumerate("xyz"):
        print(f"set mag_axis_map_{ax}={perm[i]}")
    for i, ax in enumerate("xyz"):
        print(f"set mag_axis_sign_{ax}={sg[i]}")
    print("save")

    if bad:
        print("\n!!! CẢNH BÁO — xem lại trước khi save:")
        for b in bad:
            print("   -", b)
    else:
        print("\nKiểm định: đạt. Sau khi save, khởi động lại FC rồi so yaw với la bàn điện thoại ở 4 hướng.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="cổng console UART (COM4)")
    ap.add_argument("--seconds", type=float, default=90)
    ap.add_argument("--fit", metavar="NPZ", help="chỉ khớp lại file đã thu")
    a = ap.parse_args()
    if a.fit:
        fit(a.fit)
    elif a.port:
        out = Path(__file__).resolve().parents[1] / "Log" / f"magaxis_{datetime.now():%Y%m%d_%H%M%S}.npz"
        fit(capture(a.port, a.seconds, out))
    else:
        ap.error("cần --port hoặc --fit")


if __name__ == "__main__":
    main()
