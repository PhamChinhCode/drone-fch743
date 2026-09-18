"""
Kiem tra lai tham so optical flow (MTF-01P) tren FC qua console COM.

Chay:  python flow_check.py [--port COM4] [--still 10] [--tilt 30]

Ba buoc, moi buoc bam Enter de bat dau:
  1. DUNG YEN (mode 4 + mode 12): tan so goi flow, chat luong, do cao,
     nhieu va lech 0 cua van toc do duoc khi may bay KHONG di chuyen.
  2. LAC NGHIENG TAI CHO (mode 12): khop wx = k*gx va wy = k*gy.
     Dung thi k = -1.00. k lech -> flow_rad_per_count sai ti le |k|.
     Kem uoc luong tre giua flow va gyro (tre lon cung lam POSHOLD lac).
  3. Ket luan + lenh `set` goi y. Script KHONG tu set gi len FC.

Du lieu tho ghi ra Log/flowcheck_<thoi gian>.csv.
"""
import argparse
import csv
import os
import re
import time

import numpy as np
import serial

GYRO_MIN_RAD_S = 0.3     # chi khop mau quay du nhanh, giong lan hieu chuan truoc

# Phai khop FLOW_OFFSET_X_M / FLOW_OFFSET_Z_M dang nap tren FC.
FW_RX = -0.067
FW_RZ = 0.019


def send(s, cmd, wait=0.6):
    s.reset_input_buffer()
    s.write((cmd + "\r\n").encode())
    end, buf = time.time() + wait, b""
    while time.time() < end:
        buf += s.read(4096)
    return buf.decode(errors="replace")


def read_params(s):
    txt = send(s, "get flow_") + send(s, "get est_flow_")
    return {m[0]: float(m[1]) for m in re.findall(r"^(\w+) = (\S+)", txt, re.M)}


def capture(s, mode, secs):
    """Doc cac dong so cua mode trong `secs` giay. Tra ve list token (bo '|')."""
    send(s, f"mode {mode}", 0.2)
    rows, end, pending = [], time.time() + secs, b""
    while time.time() < end:
        pending += s.read(4096)
        *lines, pending = pending.split(b"\n")
        for ln in lines:
            tok = ln.decode(errors="replace").replace("|", " ").split()
            if tok and re.fullmatch(r"-?\d+(\.\d+)?", tok[0]):
                rows.append(tok)
    send(s, "mode 0", 0.2)
    return rows, secs / max(len(rows), 1)   # chu ky dong thuc te


def parse_vel(rows):
    """mode 12: wx gx tong | wy gy tong | vb_x vb_y | range_m | qual | bo | tuoi | tin"""
    out = []
    for t in rows:
        if len(t) < 13:
            continue
        try:
            wx, gx, _, wy, gy, _, vbx, vby, rng = map(float, t[:9])
            qual, bo = int(t[9]), int(t[10])
            age = -1 if t[11] == "--" else int(t[11])
        except ValueError:
            continue
        # Cot gz (rad/s) chi co tu firmware co bu canh tay don tro di.
        gz = float(t[13]) if len(t) >= 14 else float("nan")
        # Van toc sau EKF + bias gia toc (he than) - firmware co cot ek_f tro di.
        ek = [float(x) for x in t[14:18]] if len(t) >= 18 else [float("nan")] * 4
        out.append(dict(wx=wx, gx=gx, wy=wy, gy=gy, vbx=vbx, vby=vby, range_m=rng,
                        qual=qual, rej=bo, age_ms=age, valid=(t[12] == "co"), gz=gz,
                        ek_f=ek[0], ek_r=ek[1], b_f=ek[2], b_r=ek[3]))
    return out


def fresh_samples(vel):
    """
    Console in dong deu dan nhung EKF chi cap nhat wx/gx khi co goi flow DUOC
    CHAP NHAN. Mau bi tu choi (nghieng qua, thap qua, qual thap) de lai so cu,
    nen dong trung lap = khong co mau moi -> bo. Giu chi so dong de uoc luong tre.
    """
    out, last = [], None
    for i, r in enumerate(vel):
        key = (r["wx"], r["gx"], r["wy"], r["gy"])
        if key != last and r["valid"]:
            out.append((i, r))
        last = key
    return out


def fit_axis(idx, w, g, row_dt):
    """Khop w = k*g (qua goc) va w = k*g + c*g' de uoc luong tre tau = -c/k."""
    m = np.abs(g) > GYRO_MIN_RAD_S
    n = int(m.sum())
    if n < 20:
        return None
    k = float(np.sum(w[m] * g[m]) / np.sum(g[m] ** 2))
    res = w[m] - k * g[m]
    r2 = 1.0 - float(np.sum(res ** 2) / np.sum((w[m] - w[m].mean()) ** 2))

    # Tre = do dich (so dong) cho tuong quan cheo w(t) ~ g(t - L) lon nhat, noi
    # suy parabol quanh dinh. Cach dung dao ham gyro truoc day bi lech ~20 ms
    # khi chu ky dong chi 20 ms, bao tre ao.
    tau = None
    if len(idx) and np.all(np.diff(idx) >= 1):
        lags = np.arange(-5, 11)
        cc = []
        for L in lags:
            a = w[L:] if L >= 0 else w[:L]
            b = g[:len(g) - L] if L >= 0 else g[-L:]
            cc.append(-float(np.dot(a, b)) / np.sqrt(np.dot(a, a) * np.dot(b, b)))
        i = int(np.argmax(cc))
        off = 0.0
        if 0 < i < len(cc) - 1:
            den = cc[i - 1] - 2 * cc[i] + cc[i + 1]
            off = 0.5 * (cc[i - 1] - cc[i + 1]) / den if den != 0 else 0.0
        tau = float((lags[i] + off) * row_dt)
    return dict(n=n, k=k, r2=r2, resid=float(np.std(res)), tau=tau)


def beep(freq, ms, times=1):
    import winsound
    for _ in range(times):
        winsound.Beep(freq, ms)


def wait_enter(msg, auto=0):
    print("\n" + msg, flush=True)
    if not auto:
        input("  -> San sang thi bam Enter... ")
        return
    # Che do tu dong: dem nguoc, bip ngan 3 giay cuoi, bip dai = BAT DAU.
    for i in range(auto, 0, -1):
        print(f"  bat dau sau {i} s", flush=True)
        if i <= 3:
            beep(1000, 150)
            time.sleep(0.85)
        else:
            time.sleep(1)
    beep(1500, 700)
    print("  >>> BAT DAU", flush=True)


def run_yaw(s, a):
    """
    Kiem bu canh tay don: xoay YAW quanh tam may, so vb_y voi gz.

    Firmware tra vb_y = vy_cam_bien - (gz*RX - gx*RZ). Cong lai phan da tru
    thi ra van toc tho cua cam bien; do doc cua no theo gz CHINH LA RX that
    (vy_cam_bien = gz*RX khi tam dung yen). Nen khong can doan dau.
    """
    wait_enter(
        "XOAY YAW (%.0f s)\n"
        "  Cam may bay ngang, cao 0.8-1 m, camera nhin xuong san co van.\n"
        "  Xoay YAW qua lai quanh TAM MAY (khong phai quanh cam bien), bien do\n"
        "  ~45-90 do, nhip ~1 lan/giay. Giu tam may dung cho, khong nghieng."
        % a.yaw, a.auto)
    raw, _ = capture(s, 12, a.yaw)
    if a.auto:
        beep(800, 300, 3)
    s.close()

    vel = parse_vel(raw)
    fs = [r for _, r in fresh_samples(vel)]

    fn = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Log",
                      time.strftime("yawcheck_%Y%m%d_%H%M%S.csv"))
    with open(fn, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["gz", "gx", "gy", "vb_x", "vb_y", "range_m", "qual",
                    "ek_f", "ek_r", "b_f", "b_r"])
        # Ghi DU moi dong (khong loc trung): can chuoi thoi gian lien tuc cua ek_*.
        for r in vel:
            w.writerow([r["gz"], r["gx"], r["gy"], r["vbx"], r["vby"], r["range_m"], r["qual"],
                        r["ek_f"], r["ek_r"], r["b_f"], r["b_r"]])
    print("\nDu lieu tho: %s" % os.path.normpath(fn))
    print("\n" + "=" * 64)
    print("KET QUA XOAY YAW   (bu dang dat: RX = %+.3f m, RZ = %+.3f m)" % (FW_RX, FW_RZ))
    print("=" * 64)
    if fs and np.isnan(fs[0]["gz"]):
        print("  Firmware tren FC chua co cot gz o mode 12 - nap firmware moi truoc.")
        return
    gz = np.array([r["gz"] for r in fs])
    gx = np.array([r["gx"] for r in fs])
    vy = np.array([r["vby"] for r in fs])
    vx = np.array([r["vbx"] for r in fs])
    m = np.abs(gz) > 0.5
    print(f"  Mau moi: {len(fs)}, trong do {m.sum()} mau |gz| > 0.5 rad/s"
          f" (gz max {np.abs(gz).max():.1f} rad/s)")
    if m.sum() < 20:
        print("  KHONG DU MAU - xoay nhanh hon.")
        return

    def slope(y):
        return float(np.sum(y[m] * gz[m]) / np.sum(gz[m] ** 2))

    raw_vy = vy + (gz * FW_RX - gx * FW_RZ)
    s_comp, s_raw, s_x = slope(vy), slope(raw_vy), slope(vx)
    r = float(np.corrcoef(raw_vy[m], gz[m])[0, 1])
    print(f"  vb_y ~ gz (DA bu)   : {s_comp:+.4f} m/(rad/s)   <- dung thi ~0")
    print(f"  vb_y ~ gz (CHUA bu) : {s_raw:+.4f} m/(rad/s)   r = {r:+.2f}")
    print(f"  vb_x ~ gz           : {s_x:+.4f} m/(rad/s)   <- lech ngang cua cam bien")
    print(f"\n  => RX THAT (do duoc) = {s_raw:+.3f} m   (ly thuyet -0.067)")
    if abs(s_comp) < 0.015:
        print("  => Bu canh tay don DA DUNG. Khong can doi.")
    else:
        print(f"  => DE NGHI: #define FLOW_OFFSET_X_M ({s_raw:+.3f}f)  trong fc_config.h")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM4")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--still", type=float, default=10.0, help="giay dung yen")
    ap.add_argument("--tilt", type=float, default=30.0, help="giay lac nghieng")
    ap.add_argument("--auto", type=int, default=0,
                    help="dem nguoc N giay + bip thay cho bam Enter")
    ap.add_argument("--yaw-only", action="store_true",
                    help="chi chay buoc xoay yaw (kiem bu canh tay don)")
    ap.add_argument("--yaw", type=float, default=25.0, help="giay xoay yaw")
    a = ap.parse_args()

    s = serial.Serial(a.port, a.baud, timeout=0.05)
    send(s, "mode 0", 0.3)
    p = read_params(s)
    if "flow_rad_per_count" not in p:
        raise SystemExit("Khong doc duoc tham so tu FC - sai cong hoac FC chua chay?")
    cur = p["flow_rad_per_count"]
    tilt_max = p.get("est_flow_max_tilt_deg", 20.0)
    print("Tham so tren FC:")
    for k, v in p.items():
        print(f"  {k:24s} = {v:g}")

    if a.yaw_only:
        run_yaw(s, a)
        return

    # ---------------- 1. DUNG YEN ----------------
    wait_enter(
        "BUOC 1 - DUNG YEN (%.0f s)\n"
        "  Dat may bay tren vat ke cao 0.4-1 m (hoac cam tay that chac), camera\n"
        "  nhin xuong san CO VAN (tham, go, gach co hoa van). Khong dong vao."
        % (a.still + 5), a.auto)
    t0 = time.time()
    raw4, _ = capture(s, 4, 5.0)
    t_flow = time.time() - t0
    raw12_still, _ = capture(s, 12, a.still)
    if a.auto:
        beep(800, 500)                       # het buoc 1

    rate = None
    # mode 4: fx fy | qual | range_mm | rq | v_fwd v_rgt | dt | count | frames | crc | err
    raw4 = [r for r in raw4 if len(r) >= 12 and r[8].isdigit()]
    counts = [int(r[8]) for r in raw4]
    if len(counts) > 2:
        rate = (counts[-1] - counts[0]) / t_flow
    crc = [int(r[10]) for r in raw4]
    qual4 = [int(r[2]) for r in raw4]
    rng4 = [int(r[3]) for r in raw4]

    still = parse_vel(raw12_still)
    fs_still = [r for _, r in fresh_samples(still)]

    # ---------------- 2. LAC NGHIENG ----------------
    wait_enter(
        "BUOC 2 - LAC NGHIENG TAI CHO (%.0f s)\n"
        "  Cam may bay o do cao 0.4-1 m tren cung mat san.\n"
        "  Lac NGHIENG qua lai: ~10 s quanh truc ROLL (trai-phai), ~10 s quanh truc\n"
        "  PITCH (truoc-sau), roi tron ca hai. Nhip ~1-2 lan/giay, du nhanh.\n"
        "  - Goc nghieng KHONG qua %.0f do (qua la FC bo mau).\n"
        "  - Giu camera o cho, CHI xoay, khong dich ngang." % (a.tilt, tilt_max - 5), a.auto)
    raw12_tilt, row_dt = capture(s, 12, a.tilt)
    if a.auto:
        beep(800, 300, 3)                    # het buoc 2
    tilt = parse_vel(raw12_tilt)
    fs = fresh_samples(tilt)
    s.close()

    # ---------------- Luu CSV ----------------
    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Log")
    os.makedirs(log_dir, exist_ok=True)
    fn = os.path.join(log_dir, time.strftime("flowcheck_%Y%m%d_%H%M%S.csv"))
    with open(fn, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["phase", "row", "wx", "gx", "wy", "gy", "vb_x", "vb_y",
                    "range_m", "qual", "rej", "age_ms", "valid"])
        for phase, data in (("still", still), ("tilt", tilt)):
            for i, r in enumerate(data):
                w.writerow([phase, i, r["wx"], r["gx"], r["wy"], r["gy"], r["vbx"],
                            r["vby"], r["range_m"], r["qual"], r["rej"], r["age_ms"],
                            int(r["valid"])])

    # ---------------- 3. KET LUAN ----------------
    print("\n" + "=" * 64)
    print("KET QUA   (du lieu tho: %s)" % os.path.normpath(fn))
    print("=" * 64)

    print("\n[1] Cam bien")
    if rate:
        print(f"  Tan so goi flow   : {rate:5.1f} Hz   (datasheet 100 Hz)")
    if qual4:
        print(f"  Chat luong        : TB {np.mean(qual4):.0f}, min {min(qual4)}"
              f"   (nguong {p.get('flow_quality_min', 64):g})")
    if rng4:
        print(f"  Do cao laser      : {np.mean(rng4):.0f} mm +/- {np.std(rng4):.0f}")
    if len(crc) > 1:
        print(f"  Loi CRC moi       : {crc[-1] - crc[0]}")

    print("\n[2] Dung yen - van toc do duoc (phai ~0)")
    if len(fs_still) < 10:
        print(f"  CHI CO {len(fs_still)} mau hop le - kiem tra do cao >= "
              f"{p.get('est_flow_min_height_m', 0.2):g} m va mat san co van.")
    else:
        vx = np.array([r["vbx"] for r in fs_still])
        vy = np.array([r["vby"] for r in fs_still])
        print(f"  vb_x: TB {vx.mean():+.3f}  do lech {vx.std():.3f} m/s")
        print(f"  vb_y: TB {vy.mean():+.3f}  do lech {vy.std():.3f} m/s")
        print(f"  (est_flow_noise_mps dang = {p.get('est_flow_noise_mps', float('nan')):g})")
        if max(abs(vx.mean()), abs(vy.mean())) > 0.05:
            print("  !! Lech 0 > 5 cm/s khi dung yen -> may se TROI deu theo huong do.")

    print("\n[3] Lac nghieng - flow so voi gyro (phai k = -1.00)")
    rej = [r["rej"] for r in tilt]
    if rej:
        print(f"  Mau moi: {len(fs)} / {len(tilt)} dong, FC tu choi them "
              f"{rej[-1] - rej[0]} mau (nghieng qua / thap / qual kem)")
    idx = np.array([i for i, _ in fs])
    res = {}
    for ax, wk, gk in (("X (roll)", "wx", "gx"), ("Y (pitch)", "wy", "gy")):
        w_ = np.array([r[wk] for _, r in fs])
        g_ = np.array([r[gk] for _, r in fs])
        r = fit_axis(idx, w_, g_, row_dt) if len(fs) else None
        res[ax] = r
        if r is None:
            print(f"  Truc {ax}: KHONG DU MAU (can >= 20 mau |gyro| > {GYRO_MIN_RAD_S} rad/s)"
                  " - lac manh hon.")
            continue
        tau = "n/a" if r["tau"] is None else f"{r['tau'] * 1000:+.0f} ms"
        print(f"  Truc {ax}: k = {r['k']:+.3f}   R2 = {r['r2']:.2f}   "
              f"du {r['resid']:.2f} rad/s   n = {r['n']}   tre flow~gyro = {tau}")

    good = [r for r in res.values() if r is not None]
    print("\n[4] Ket luan")
    if not good:
        print("  Khong du du lieu de ket luan. Lam lai buoc 2, lac manh va deu hon.")
        return
    ks = [r["k"] for r in good]
    k = float(np.average(ks, weights=[r["n"] for r in good]))
    if k > 0:
        print("  !! k DUONG: phep bu quay dang CONG them thay vi tru. Dau truc flow")
        print("     hoac dau gyro sai - dung chinh flow_rad_per_count, xem lai dau truc.")
        return
    new = cur / abs(k)
    err = abs(abs(k) - 1.0) * 100
    print(f"  k gop = {k:+.3f}  -> flow_rad_per_count dang lech {err:.1f}%")
    if len(good) == 2 and abs(ks[0] - ks[1]) > 0.1:
        print("  !! Hai truc lech nhau > 0.1 - nghi mau bi nhieu, lam lai cho chac.")
    if min(r["r2"] for r in good) < 0.8:
        print("  !! R2 < 0.8 - flow bam kem (mat san it van / anh sang yeu).")
    if err > 5:
        print(f"  => DE NGHI:  set flow_rad_per_count={new:.5f}")
        print("               save")
        print("     roi chay lai script, k phai ve ~ -1.00.")
    else:
        print(f"  => flow_rad_per_count = {cur:g} DA DUNG (lech < 5%). Khong can doi.")
    taus = [r["tau"] for r in good if r["tau"] is not None]
    if taus and max(abs(t) for t in taus) > 0.02:
        print(f"  !! Tre flow so voi gyro ~{np.mean(taus) * 1000:.0f} ms: phep bu quay tru")
        print("     sai thoi diem -> van toc ao moi khi may nghieng. Day co the chinh la")
        print("     ly do POSHOLD lac/troi du he so da dung. Can lam tre gyro cho khop.")


if __name__ == "__main__":
    main()
