"""
Phân tích file .npz do vib_capture.py thu: phổ rung và phổ theo thời gian.

    python tools/vib_analyze.py Log/vib_20260929_153000.npz
    python tools/vib_analyze.py Log/vib_....npz --kv 2300 --cells 4

Ra một file PNG cạnh file .npz, gồm:
  1. PSD (Welch) gyro 3 trục          — đỉnh nào, cao bao nhiêu
  2. PSD accel 3 trục
  3. Spectrogram gyro (tổng 3 trục) + đường ga trung bình 4 motor
     -> đỉnh CHẠY THEO GA  = motor/cánh quạt (mất cân bằng, cong, ổ bi)
     -> đỉnh ĐỨNG YÊN      = cộng hưởng khung / cách gắn FC

Muốn biết motor NÀO gây rung: quay thử từng motor một, mỗi lần một file.

--kv và --cells (tuỳ chọn) vẽ thêm đường tần số quay ước tính của motor
f = ga * KV * (3,7 V * cells) / 60 và họa tần bậc 2 lên spectrogram. Chỉ là
ước tính: KV thật dưới tải và điện áp pin sụt đều làm lệch.
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from scipy import signal

sys.stdout.reconfigure(encoding="utf-8")   # console Windows mặc định cp1252

DSHOT_MIN, DSHOT_MAX = 48, 2047
AXES = ("roll (x)", "pitch (y)", "yaw (z)")


def top_peaks(f, p, n=5, fmin=10.0):
    m = f >= fmin
    f, p = f[m], p[m]
    pk, _ = signal.find_peaks(p, distance=max(1, int(5.0 / (f[1] - f[0]))))
    pk = pk[np.argsort(p[pk])[::-1][:n]]
    return sorted(zip(f[pk], p[pk]), key=lambda x: -x[1])


def analyze(path, kv=None, cells=None, show=False):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d = np.load(path)
    fs = float(d["fs"])
    gyro, accel, motor, t = d["gyro_dps"], d["accel_g"], d["motor"], d["t_s"]
    thr = np.clip((motor.astype(float) - DSHOT_MIN) / (DSHOT_MAX - DSHOT_MIN), 0, 1)
    thr[motor < DSHOT_MIN] = 0.0          # 0 = lệnh dừng DShot
    thr_mean = thr.mean(axis=1)

    nper = int(fs / 2)                     # độ phân giải ~2 Hz
    fig, ax = plt.subplots(3, 1, figsize=(13, 13))

    print(f"{path}\n  fs = {fs:.1f} Hz, {len(t)} mẫu, {t[-1] - t[0]:.1f} s")
    for k, name in enumerate(AXES):
        f, p = signal.welch(gyro[:, k] - gyro[:, k].mean(), fs=fs, nperseg=nper)
        ax[0].semilogy(f, p, lw=0.8, label=name)
        pk = ", ".join(f"{fr:.0f} Hz" for fr, _ in top_peaks(f, p))
        rms = np.std(gyro[:, k])
        print(f"  gyro {name:9s}: RMS {rms:7.2f} °/s  | đỉnh: {pk}")
    for k, name in enumerate(AXES):
        f, p = signal.welch(accel[:, k] - accel[:, k].mean(), fs=fs, nperseg=nper)
        ax[1].semilogy(f, p, lw=0.8, label=name)
        pk = ", ".join(f"{fr:.0f} Hz" for fr, _ in top_peaks(f, p))
        rms = np.std(accel[:, k])
        print(f"  accel {name:8s}: RMS {rms:7.3f} g    | đỉnh: {pk}")

    ax[0].set(title="PSD gyro (chưa lọc)", ylabel="(°/s)²/Hz", xlim=(0, fs / 2))
    ax[1].set(title="PSD accel (chưa lọc)", ylabel="g²/Hz", xlim=(0, fs / 2),
              xlabel="Hz")
    for a in ax[:2]:
        a.grid(True, which="both", alpha=0.3)
        a.legend()

    g = gyro - gyro.mean(axis=0)
    S = 0.0
    for k in range(3):
        f, ts, Sk = signal.spectrogram(g[:, k], fs=fs, nperseg=nper // 2,
                                       noverlap=nper // 4)
        S = S + Sk
    ts = ts + t[0]
    im = ax[2].pcolormesh(ts, f, 10 * np.log10(S + 1e-12), shading="auto",
                          cmap="inferno")
    fig.colorbar(im, ax=ax[2], label="dB")
    ax[2].set(title="Spectrogram gyro (tổng 3 trục)", ylabel="Hz", xlabel="s",
              ylim=(0, fs / 2))

    if kv and cells:
        f_rot = thr_mean * kv * 3.7 * cells / 60.0
        ax[2].plot(t, f_rot, "c--", lw=1, label="quay motor (ước tính)")
        ax[2].plot(t, 2 * f_rot, "c:", lw=1, label="× 2")
        ax[2].legend(loc="upper left")

    a2 = ax[2].twinx()
    a2.plot(t, thr_mean * 100, "w", lw=1)
    a2.set_ylabel("ga trung bình 4 motor, %")
    a2.set_ylim(0, 100)

    fig.tight_layout()
    out = Path(path).with_suffix(".png")
    fig.savefig(out, dpi=110)
    print(f"  Đồ thị: {out}")
    if show:
        plt.show()
    return out


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("npz")
    ap.add_argument("--kv", type=float)
    ap.add_argument("--cells", type=int)
    ap.add_argument("--show", action="store_true")
    a = ap.parse_args()
    analyze(a.npz, a.kv, a.cells, a.show)
