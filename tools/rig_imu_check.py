"""
Kiểm tra IMU từ log lắc tay trên giá (rig_log.py --imu-hz ...).

    python tools/rig_imu_check.py Log/rig_imu_20261007.npz

So: (1) gyro X với đạo hàm góc roll của EKF, (2) roll EKF với roll tính từ
accel, (3) IMU1 (ICM-42688) với IMU2 (LSM6DSV) cả gyro lẫn accel.
"""
import sys

import numpy as np

sys.stdout.reconfigure(encoding="utf-8")


def fit(a, b):
    """b ≈ k·a + c, trả về k, hệ số tương quan, RMS sai lệch."""
    A = np.c_[a, np.ones_like(a)]
    (k, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    return k, np.corrcoef(a, b)[0, 1], np.sqrt(np.mean((b - (k * a + c)) ** 2))


def main(path):
    d = np.load(path)
    att, imu = d["att"], d["imu"]
    t_a, roll, pitch = att[:, 0], att[:, 1] / 100, att[:, 2] / 100
    rate_est = att[:, 4] / 10
    t_i = imu[:, 0]
    g = imu[:, 1:4] / 10                 # °/s
    acc = imu[:, 4:7] / 1000             # g

    mov = np.abs(np.interp(t_i, t_a, roll) - np.median(roll)) > -1   # cả đoạn
    print(f"Roll EKF: {roll.min():.1f}° .. {roll.max():.1f}°   pitch: {pitch.min():.1f}° .. {pitch.max():.1f}°")

    # (1) gyro X vs d(roll)/dt — làm trơn 20 ms để bớt nhiễu đạo hàm
    r_i = np.interp(t_i, t_a, roll)
    k = 10
    dr = (r_i[k:] - r_i[:-k]) / (t_i[k:] - t_i[:-k])
    gx = (g[k:, 0] + g[:-k, 0]) / 2
    kk, cc, rm = fit(gx, dr)
    print(f"\n(1) d(roll)/dt ≈ {kk:.3f} × gyroX   tương quan {cc:+.3f}   RMS lệch {rm:.1f} °/s")
    print("    -> đúng nếu hệ số ≈ +1 và tương quan ≈ +1 (cùng chiều, cùng thang)")
    print(f"    gyro Y,Z lớn nhất khi lắc roll: {np.abs(g[:,1]).max():.0f}, {np.abs(g[:,2]).max():.0f} °/s "
          f"(so với X {np.abs(g[:,0]).max():.0f}) — giá chỉ cho roll nên Y,Z phải nhỏ")

    # (2) roll từ accel: f = [sinθ, -sinφcosθ, -cosφcosθ]
    roll_acc = np.degrees(np.arctan2(-acc[:, 1], -acc[:, 2]))
    slow = np.abs(g[:, 0]) < 40          # chỉ lấy lúc xoay chậm (ít gia tốc động)
    kk, cc, rm = fit(roll_acc[slow], r_i[slow])
    print(f"\n(2) roll EKF ≈ {kk:.3f} × roll_accel   tương quan {cc:+.3f}   RMS lệch {rm:.2f}°   ({slow.sum()} mẫu xoay chậm)")
    amag = np.linalg.norm(acc, axis=1)
    print(f"    |a| lúc đứng yên: {np.median(amag[np.abs(g).max(1) < 2]):.3f} g (phải ≈ 1,000)")

    # (3) IMU1 vs IMU2 qua lệnh CLI `imu`
    if "imucli" in d.files:
        c = d["imucli"]
        i1, i2 = c[c[:, 1] == 1], c[c[:, 1] == 2]
        n = min(len(i1), len(i2))
        i1, i2 = i1[:n], i2[:n]
        dt = np.abs(i1[:, 0] - i2[:, 0])
        ok = dt < 0.005                  # hai dòng cùng một lần trả lời
        i1, i2 = i1[ok], i2[ok]
        print(f"\n(3) IMU1 vs IMU2 ({len(i1)} cặp, gyro lấy tại cùng một lần hỏi):")
        for j, nm in enumerate(("gyro X", "gyro Y", "gyro Z")):
            a1, a2 = i1[:, 2 + j], i2[:, 2 + j]
            if np.ptp(a1) > 20:
                kk, cc, rm = fit(a1, a2)
                print(f"    {nm}: IMU2 ≈ {kk:.3f} × IMU1   tương quan {cc:+.3f}   RMS lệch {rm:.1f} °/s")
            else:
                print(f"    {nm}: IMU1 {np.ptp(a1):.1f}, IMU2 {np.ptp(i2[:, 2 + j]):.1f} °/s đỉnh-đỉnh (ít chuyển động)")
        for j, nm in enumerate(("accel X", "accel Y", "accel Z")):
            a1, a2 = i1[:, 5 + j], i2[:, 5 + j]
            kk, cc, rm = fit(a1, a2) if np.ptp(a1) > 0.2 else (np.nan, np.nan, np.sqrt(np.mean((a1 - a2) ** 2)))
            print(f"    {nm}: lệch trung bình {np.mean(a2 - a1):+.3f} g, RMS lệch {rm:.3f} g"
                  + (f", IMU2 ≈ {kk:.3f}×IMU1, r {cc:+.3f}" if not np.isnan(kk) else ""))
        e1, e2 = i1[-1, 9] - i1[0, 9], i2[-1, 9] - i2[0, 9]
        print(f"    lỗi đọc trong lúc thử: IMU1 +{e1:.0f}, IMU2 +{e2:.0f};  "
              f"healthy cuối: {int(i1[-1, 10])}/{int(i2[-1, 10])}")


if __name__ == "__main__":
    main(sys.argv[1])
