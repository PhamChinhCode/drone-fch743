"""
Chỉ số đáp ứng vòng rate roll theo từng lần gạt cần, chia theo mốc thời gian.

    python tools/rig_pid_steps.py Log/rig_pid_run2.npz 26:62 62:98 98:134

Mốc là giây tính từ lúc rig_log.py bắt đầu (trùng với mốc --say).
Mỗi đoạn (một mức ga):
  - giữ yên (2-12 s đầu đoạn): nhiễu gyro roll, lệch chuẩn góc, dao động trội
  - mỗi lần gạt: trễ tới 50 % biên độ, vọt lố gyro so với đỉnh setpoint,
    số lần đổi dấu sai số sau đỉnh (đo độ "rung lắc"), đầu ra PID đỉnh
"""
import sys

import numpy as np
from scipy import signal

sys.stdout.reconfigure(encoding="utf-8")


def main(path, spans):
    d = np.load(path)
    p, a, m = d["pid"], d["att"], d["motor"]
    G = np.arange(p[0, 0], p[-1, 0], 0.001)
    I = lambda t, x: np.interp(G, t, x)
    sp, gy, out = I(p[:, 0], p[:, 1] / 10), I(p[:, 0], p[:, 4] / 10), I(p[:, 0], p[:, 7] / 1000)
    roll = I(a[:, 0], a[:, 1] / 100)
    thr = np.c_[[I(m[:, 0], m[:, 1 + i]) for i in range(4)]].T
    armed = I(m[:, 0], (m[:, 9].astype(int) & 1).astype(float)) > 0.5

    for span in spans:
        t0, t1 = (float(x) for x in span.split(":"))
        w = (G >= t0) & (G < t1) & armed
        if w.sum() < 2000:
            print(f"\n== {span}: không đủ dữ liệu khi ARM")
            continue
        idx = np.flatnonzero(w)
        print(f"\n== {span} s: ga TB {thr[w].mean():.0f} DShot ({(thr[w].mean()-48)/1999*100:.0f} %), "
              f"motor min {thr[w].min():.0f} max {thr[w].max():.0f}, chênh cặp trước/sau "
              f"{(thr[w][:, [2, 3]].mean() - thr[w][:, [0, 1]].mean()):+.0f}")
        q = w & (G >= t0 + 2) & (G < t0 + 11) & (np.abs(sp) < 8)
        if q.sum() > 2000:
            g = gy[q]
            f, pw = signal.welch(g - g.mean(), fs=1000, nperseg=2048)
            sel = (f > 1) & (f < 300)
            print(f"   giữ yên: gyro RMS {g.std():5.2f} °/s, roll lệch chuẩn {roll[q].std():.2f}°, "
                  f"PID out RMS {out[q].std():.4f}, dao động trội {f[sel][np.argmax(pw[sel])]:.1f} Hz")
        pk, _ = signal.find_peaks(np.abs(sp) * w, height=40, distance=1500)
        rows = []
        for k in pk:
            s = np.sign(sp[k])
            s0 = max(0, k - 400)
            i50 = s0 + int(np.argmax(np.abs(sp[s0:k + 1]) > 0.5 * abs(sp[k])))
            j = int(np.argmax(s * gy[i50:i50 + 400] > 0.5 * abs(sp[k])))
            win = slice(k, min(k + 500, len(gy)))
            ov = (s * gy[win]).max() / abs(sp[k]) * 100 - 100
            e = gy[win] - sp[win]
            zc = int(np.sum(np.diff(np.sign(e - e.mean())) != 0))
            rows.append((j, ov, zc, np.abs(out[s0:k + 500]).max(), sp[k]))
        if rows:
            r = np.array(rows)
            print(f"   {len(r)} lần gạt: trễ 50 % trung vị {np.median(r[:,0]):.0f} ms (min {r[:,0].min():.0f}, max {r[:,0].max():.0f}), "
                  f"vọt lố trung vị {np.median(r[:,1]):+.0f} %, đổi dấu sai số/0,5 s {np.median(r[:,2]):.0f}, "
                  f"PID out đỉnh {r[:,3].max():.2f}, setpoint đỉnh {np.abs(r[:,4]).max():.0f} °/s")
        print(f"   roll trong đoạn: {roll[w].min():+.1f}° .. {roll[w].max():+.1f}°")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
