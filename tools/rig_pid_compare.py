"""
So sánh các lượt chỉnh PID roll trên giá (rig_log.py, kịch bản 3 mức ga).

    python tools/rig_pid_compare.py "lan3=Log/rig_pid_run3.npz" "lan4=Log/rig_pid_run4.npz"

Mốc theo kịch bản giọng nói: giữ yên 28-36 / 64-72 / 100-108 s,
gạt cần 37-62 / 73-98 / 109-134 s (ga 1/4, 1/2, 3/4).
Chỉ số mỗi mức ga:
  trễ        : độ trễ setpoint -> gyro (tương quan chéo)
  sai số dạng: RMS(gyro - setpoint dời theo trễ) / RMS(setpoint), lúc |sp|>15
  giữ yên    : lệch chuẩn roll, RMS gyro, và đỉnh phổ gyro 10-150 Hz (dấu
               hiệu sắp dao động khi P/D quá cao)
Ra thêm ảnh so sánh dạng sóng cạnh file cuối cùng.
"""
import sys
from pathlib import Path

import numpy as np
from scipy import signal

sys.stdout.reconfigure(encoding="utf-8")

HOLD = [(28, 36), (64, 72), (100, 108)]
FLICK = [(37, 62), (73, 98), (109, 134)]
NAMES = ["ga 1/4", "ga 1/2", "ga 3/4"]


def load(path):
    d = np.load(path)
    p, a, m = d["pid"], d["att"], d["motor"]
    G = np.arange(p[0, 0], p[-1, 0], 0.001)
    I = lambda t, x: np.interp(G, t, x)
    thr = np.c_[[I(m[:, 0], m[:, 1 + i]) for i in range(4)]].T
    return G, I(p[:, 0], p[:, 1] / 10), I(p[:, 0], p[:, 4] / 10), I(p[:, 0], p[:, 7] / 1000), \
        I(a[:, 0], a[:, 1] / 100), thr


def metrics(path):
    G, sp, gy, out, roll, thr = load(path)
    res = []
    for (h0, h1), (t0, t1) in zip(HOLD, FLICK):
        w = (G > t0) & (G < t1)
        s, g = sp[w], gy[w]
        lags = np.arange(0, 150)
        lag = int(lags[np.argmax([np.dot(s[:len(s) - L], g[L:]) for L in lags])])
        act = (np.abs(s) > 15)[:len(s) - lag]
        e = np.sqrt(np.mean((g[lag:] - s[:len(s) - lag])[act] ** 2)) / np.sqrt(np.mean(s[:len(s) - lag][act] ** 2)) * 100
        q = (G > h0) & (G < h1)
        f, pw = signal.welch(gy[q] - gy[q].mean(), fs=1000, nperseg=1024)
        sel = (f > 10) & (f < 150)
        sat = ((thr[w] >= 2040) | (thr[w] <= 160)).any(1).mean() * 100
        res.append(dict(lag=lag, err=e, rsd=roll[q].std(), grms=gy[q].std(),
                        fpk=f[sel][np.argmax(pw[sel])], ppk=np.sqrt(pw[sel].max() * (f[1] - f[0])),
                        sat=sat, sppk=np.abs(s).max(), outpk=np.abs(out[w]).max()))
    return res


def main(args):
    runs = [a.split("=", 1) for a in args]
    R = {n: metrics(p) for n, p in runs}
    for i, nm in enumerate(NAMES):
        print(f"\n{nm}:")
        for n, _ in runs:
            r = R[n][i]
            print(f"  {n:6s}: trễ {r['lag']:3d} ms | sai số dạng {r['err']:4.0f} % | giữ yên roll sd {r['rsd']:.2f}°, "
                  f"gyro {r['grms']:.2f} °/s, đỉnh phổ {r['fpk']:.0f} Hz ({r['ppk']:.2f} °/s) | "
                  f"bão hoà {r['sat']:4.1f} % | sp đỉnh {r['sppk']:.0f}, out đỉnh {r['outpk']:.2f}")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(len(runs), 3, figsize=(18, 3.4 * len(runs)), sharey=True, squeeze=False)
    for r, (n, p) in enumerate(runs):
        G, sp, gy, *_ = load(p)
        for c, (t0, t1) in enumerate(FLICK):
            w = (G > t0 + 3) & (G < t0 + 13)
            ax[r, c].plot(G[w], sp[w], "k", lw=1)
            ax[r, c].plot(G[w], gy[w], "C0", lw=1)
            ax[r, c].set_title(f"{n} {NAMES[c]}")
            ax[r, c].grid(alpha=0.3)
    fig.tight_layout()
    out = Path(runs[-1][1]).with_name(Path(runs[-1][1]).stem + "_compare.png")
    fig.savefig(out, dpi=70)
    print("\nĐồ thị:", out)


if __name__ == "__main__":
    main(sys.argv[1:])
