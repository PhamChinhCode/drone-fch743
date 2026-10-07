"""
Phân tích log chỉnh PID roll trên giá (rig_log.py).

    python tools/rig_pid_analyze.py Log/rig_pid_run1.npz

Chia log theo mức ga (lúc đang ARM), mỗi mức tính:
  - Đứng yên (setpoint ~0): nhiễu tốc độ góc, dao động góc, tần số dao động trội
  - Gạt cần: độ trễ bám (tương quan chéo setpoint↔gyro), vọt lố, thời gian lên
  - Bão hoà: đầu ra PID roll, motor chạm trần/sàn
Kết quả in ra + ảnh PNG cạnh file log.
"""
import sys
from pathlib import Path

import numpy as np
from scipy import signal

sys.stdout.reconfigure(encoding="utf-8")


GRID = None


def uniform(t, x, fs=1000.0):
    """Nội suy mọi luồng lên CÙNG một lưới thời gian (lấy theo luồng PID)."""
    global GRID
    if GRID is None:
        GRID = np.arange(t[0], t[-1], 1 / fs)
    return GRID, np.interp(GRID, t, x)


def main(path):
    path = Path(path)
    d = np.load(path)
    pid, att, mot = d["pid"], d["att"], d["motor"]
    fs = 1000.0
    t, sp = uniform(pid[:, 0], pid[:, 1] / 10)          # setpoint roll [°/s]
    _, gy = uniform(pid[:, 0], pid[:, 4] / 10)          # gyro roll đã lọc [°/s]
    _, out = uniform(pid[:, 0], pid[:, 7] / 1000)       # đầu ra PID roll (-1..1)
    _, roll = uniform(att[:, 0], att[:, 1] / 100)       # góc roll [°]
    _, armed = uniform(mot[:, 0], (mot[:, 9].astype(int) & 1).astype(float))
    thr = np.c_[[uniform(mot[:, 0], mot[:, 1 + i])[1] for i in range(4)]].T
    tm = thr.mean(1)                                        # ga trung bình (DShot)
    armed = armed > 0.5

    # --- chia mức ga: làm trơn 1 s, lượng tử theo nấc 100 DShot ---
    tms = np.convolve(tm, np.ones(1000) / 1000, mode="same")
    lvl = np.where(armed, np.round(tms / 100) * 100, -1)
    print(f"{path.name}: {t[-1]-t[0]:.0f} s, ARM {armed.mean()*100:.0f} % thời gian")
    if not armed.any():
        print("Không có đoạn nào ARM — không có gì để phân tích.")
        return
    print(f"Góc roll khi ARM: {roll[armed].min():.1f}° .. {roll[armed].max():.1f}°")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(4, 1, figsize=(15, 13), sharex=True)
    ax[0].plot(t, sp, "k", lw=0.8, label="setpoint roll [°/s]")
    ax[0].plot(t, gy, "C0", lw=0.6, label="gyro roll [°/s]")
    ax[1].plot(t, roll, "C1", lw=0.8, label="roll [°]")
    ax[2].plot(t, out, "C2", lw=0.6, label="PID roll out")
    ax[3].plot(t, thr, lw=0.6)
    ax[3].plot(t, armed * 2000, "k:", lw=0.8)
    for A in ax:
        A.grid(alpha=0.3)
        A.legend(loc="upper right", fontsize=8)
    ax[3].set_xlabel("s")
    fig.tight_layout()
    fig.savefig(path.with_suffix(".png"), dpi=80)

    levels = [v for v in np.unique(lvl) if v > 0 and (lvl == v).sum() > 3 * fs]
    for v in levels:
        m = lvl == v
        idx = np.flatnonzero(m)
        seg = slice(idx[0], idx[-1])
        s, g, o, r = sp[seg], gy[seg], out[seg], roll[seg]
        quiet = np.abs(s) < 5
        print(f"\n== Ga ~DShot {v:.0f} ({(v-48)/1999*100:.0f} %), {len(s)/fs:.0f} s")
        if quiet.sum() > fs:
            gq = g[quiet]
            f, p = signal.welch(gq - gq.mean(), fs=fs, nperseg=2048)
            sel = (f > 1) & (f < 200)
            fpk = f[sel][np.argmax(p[sel])]
            print(f"   Đứng yên: gyro RMS {gq.std():5.2f} °/s, roll lệch chuẩn {r[quiet].std():.2f}°, "
                  f"đầu ra PID RMS {o[quiet].std():.3f}, dao động trội {fpk:.1f} Hz")
        # bám setpoint: tương quan chéo trên đoạn có gạt cần
        act = np.abs(s) > 10
        if act.sum() > 0.3 * fs:
            lags = np.arange(-50, 200)
            sc = s - s.mean()
            gc = g - g.mean()
            cc = [np.dot(sc[max(0, -L):len(sc) - max(0, L)], gc[max(0, L):len(gc) - max(0, -L)]) for L in lags]
            lag = lags[int(np.argmax(cc))]
            gain = np.dot(sc, gc) / np.dot(sc, sc)
            err = np.sqrt(np.mean((np.roll(g, -lag) - s)[act] ** 2))
            print(f"   Gạt cần: trễ bám {lag} ms, tỉ lệ biên độ gyro/setpoint {gain:.2f}, "
                  f"sai số bám (bù trễ) {err:.1f} °/s, setpoint đỉnh {np.abs(s).max():.0f} °/s, gyro đỉnh {np.abs(g).max():.0f} °/s")
            # vọt lố từng lần gạt: đỉnh setpoint dương
            pk, _ = signal.find_peaks(s, height=30, distance=int(1.5 * fs))
            ov = []
            for k in pk:
                w = slice(k, min(k + 300, len(g)))
                ov.append((g[w].max() - s[k]) / s[k] * 100)
            if ov:
                print(f"   Vọt lố gyro so với đỉnh setpoint: {np.round(ov, 0)} %")
        sat = (np.abs(o) > 0.95).mean() * 100
        tr = thr[seg]
        print(f"   Bão hoà: PID roll |out|>0,95 {sat:.1f} % thời gian; motor min {tr.min():.0f}, max {tr.max():.0f}")
    print(f"\nĐồ thị: {path.with_suffix('.png')}")


if __name__ == "__main__":
    main(sys.argv[1])
