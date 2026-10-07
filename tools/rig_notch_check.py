"""
Đánh giá notch gyro trên giá: so hai lượt (vd. notch tĩnh vs notch động).

    python tools/rig_notch_check.py "tinh=Log/rig_pid_run7.npz" "dong=Log/rig_pid_run8.npz"

Mỗi lượt cần cả <run>.npz (rig_log.py) và <run>_vib.npz (--vib).
Theo từng mức ga lúc giữ yên (28-36 / 64-72 / 100-108 s):
  - đỉnh rung chính của gyro THÔ (8 kHz) và tần số notch đang đặt
  - biên độ còn lọt tại đỉnh đó trong gyro SAU LỌC (luồng PID, 1 kHz)
  - nhiễu >= 50 Hz trong đầu ra PID roll (thứ đi xuống motor)
Ra thêm ảnh: notch theo thời gian + phổ thô/sau lọc từng mức ga.
"""
import sys
from pathlib import Path

import numpy as np
from scipy import signal

sys.stdout.reconfigure(encoding="utf-8")
HOLD = [(28, 36), (64, 72), (100, 108)]
NAMES = ["ga 1/4", "ga 1/2", "ga 3/4"]


def amp_at(f, p, fc, bw=4.0):
    s = (f > fc - bw) & (f < fc + bw)
    return np.sqrt(p[s].sum() * (f[1] - f[0]))


def analyse(path):
    d = np.load(path)
    v = np.load(Path(path).with_name(Path(path).stem + "_vib.npz"))
    P = d["pid"]
    G = np.arange(P[0, 0], P[-1, 0], 0.001)
    gyf = np.interp(G, P[:, 0], P[:, 4] / 10)          # gyro roll SAU lọc
    out = np.interp(G, P[:, 0], P[:, 7] / 1000)
    notch = np.interp(G, P[:, 0], P[:, 10])
    if not notch.any():                 # firmware trước notch động không báo -> notch tĩnh
        notch[:] = 220.0
    mot = d["motor"]
    mm = np.interp(G, mot[:, 0], mot[:, 1:5].mean(1))

    # gyro thô: căn thời gian bằng tương quan chéo của ga trung bình (vib có motor)
    fs = float(v["fs"])
    vm = v["motor"].astype(float).mean(1)
    tv = v["t_s"] - v["t_s"][0]
    vm_on_G = np.interp(G - G[0], tv, vm, left=np.nan, right=np.nan)
    best, off = -1, 0.0
    for o in np.arange(-3, 3, 0.01):                     # dò lệch thời gian ±3 s
        x = np.interp(G - G[0] - o, tv, vm, left=np.nan, right=np.nan)
        ok = ~np.isnan(x)
        if ok.sum() < 1000:
            continue
        c = np.corrcoef(x[ok], mm[ok])[0, 1]
        if c > best:
            best, off = c, o
    graw = v["gyro_dps"]
    res = []
    for (h0, h1) in HOLD:
        q = (G > h0) & (G < h1)
        # thô
        tsel = (tv + off + G[0] > h0) & (tv + off + G[0] < h1)
        f8, p8 = signal.welch(graw[tsel] - graw[tsel].mean(0), fs=fs, nperseg=4096, axis=0)
        T = p8.sum(1)
        sel = (f8 > 60) & (f8 < 450)
        fpk = f8[sel][np.argmax(T[sel])]
        a_raw = amp_at(f8, p8[:, 0], fpk)                # trục roll
        a_raw_tot = amp_at(f8, T, fpk)
        # sau lọc (1 kHz, roll)
        f1, p1 = signal.welch(gyf[q] - gyf[q].mean(), fs=1000, nperseg=1024)
        a_filt = amp_at(f1, p1, fpk, bw=5)
        fo, po = signal.welch(out[q] - out[q].mean(), fs=1000, nperseg=1024)
        hf = np.sqrt(po[fo >= 50].sum() * (fo[1] - fo[0])) * 1000
        res.append(dict(mm=mm[q].mean(), notch=np.median(notch[q]), fpk=fpk, a_raw=a_raw,
                        a_raw_tot=a_raw_tot, a_filt=a_filt, hf=hf, gstd=gyf[q].std(),
                        f8=f8, T=T, f1=f1, p1=p1))
    return res, (G, notch, mm), best


def main(args):
    runs = [a.split("=", 1) for a in args]
    R = {}
    for n, p in runs:
        R[n], tr, c = analyse(p)
        R[n + "_trace"] = tr
        print(f"{n}: căn thời gian gyro thô với log PID, tương quan ga {c:.3f}")
    for i, nm in enumerate(NAMES):
        print(f"\n{nm}:")
        for n, _ in runs:
            r = R[n][i]
            print(f"  {n:5s}: ga TB {r['mm']:5.0f} | đỉnh thô {r['fpk']:4.0f} Hz ({r['a_raw_tot']:.2f} °/s, roll {r['a_raw']:.2f}) "
                  f"| notch đặt {r['notch']:4.0f} Hz | còn lọt sau lọc (roll) {r['a_filt']:.3f} °/s "
                  f"| gyro sau lọc RMS {r['gstd']:.2f} °/s | nhiễu ≥50 Hz xuống motor {r['hf']:.1f} ‰")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(2, 3, figsize=(18, 8))
    for k, (n, _) in enumerate(runs):
        G, notch, mm = R[n + "_trace"]
        ax[0, 0].plot(G, notch, label=f"notch {n}")
        ax[0, 1].plot(G, mm, label=f"ga TB {n}", alpha=0.7)
    ax[0, 0].set_ylabel("Hz"); ax[0, 0].legend(); ax[0, 0].set_title("Tần số notch theo thời gian")
    ax[0, 1].set_ylabel("DShot"); ax[0, 1].legend(); ax[0, 1].set_title("Ga trung bình 4 motor")
    ax[0, 2].axis("off")
    for i, nm in enumerate(NAMES):
        A = ax[1, i]
        for n, _ in runs:
            r = R[n][i]
            A.semilogy(r["f1"], r["p1"], label=f"{n} sau lọc (roll)")
        r0 = R[runs[0][0]][i]
        A.semilogy(r0["f8"], r0["T"] / r0["T"].max() * max(R[runs[0][0]][i]["p1"].max(), 1e-9), "k:", lw=0.8,
                   label="thô (tỉ lệ tương đối)")
        for n, _ in runs:
            A.axvline(R[n][i]["notch"], ls="--", lw=0.8)
        A.set_xlim(0, 500); A.set_title(nm); A.grid(alpha=0.3); A.legend(fontsize=7); A.set_xlabel("Hz")
    fig.tight_layout()
    out = Path(runs[-1][1]).with_name(Path(runs[-1][1]).stem + "_notch.png")
    fig.savefig(out, dpi=75)
    print("\nĐồ thị:", out)


if __name__ == "__main__":
    main(sys.argv[1:])
