"""Hieu chuan la ban KHONG cam day: bag tren Pi -> npz cho mag_cal9.py --fit.

    # Pi: xoay drone ngoai troi (bag tu ghi), roi
    python3 mag_bag_export.py ~/drone_logs/bag_<...> /tmp/la_ban.csv      (file .mcap da dong)
    # PC: params.txt = ban 'dump' cua FC LUC THU (bo so dang luu, de tinh nguoc so tho)
    python tools/mag_bag_to_npz.py la_ban.csv out.npz HH:MM:SS HH:MM:SS params.txt
    python tools/mag_cal9.py --fit out.npz

/mavros/imu/mag la so DA hieu chuan, he than FLU, Tesla: field_FRD = P S (raw - o) -> raw = S^-1 P^T field + o.
Lan dau 2026-10-27 (IST8310, seq 8): |B| 0,496 G +- 0,73 %, khop tam so tho do qua UART 10-19.
"""
import sys, datetime as dt
import numpy as np
sys.stdout.reconfigure(encoding="utf-8")
rows = {'mag': [], 'imu': [], 'att': [], 'gps': []}
for ln in open(sys.argv[1]):
    p = ln.strip().split(',')
    rows[p[0]].append([float(x) for x in p[1:]])
mag, imu = np.array(rows['mag']), np.array(rows['imu'])
FLU2FRD = np.array([1, -1, -1])
import re
P = dict(re.findall(r"set (mag_\w+)=(\S+)", open(sys.argv[5], encoding="utf-8", errors="replace").read()))
f = lambda k: float(P[k])
o = np.array([f("mag_offset_x_g"), f("mag_offset_y_g"), f("mag_offset_z_g")])
S = np.array([[f("mag_scale_x"), f("mag_soft_xy"), f("mag_soft_xz")],
              [f("mag_soft_xy"), f("mag_scale_y"), f("mag_soft_yz")],
              [f("mag_soft_xz"), f("mag_soft_yz"), f("mag_scale_z")]])
amap = [int(f(f"mag_axis_map_{a}")) for a in "xyz"]
asign = np.array([f(f"mag_axis_sign_{a}") for a in "xyz"])
field = mag[:, 1:4] * 1e4 * FLU2FRD            # T -> G, FLU -> FRD (he than FC)
cal = np.zeros_like(field)
for i in range(3):
    cal[:, amap[i]] = asign[i] * field[:, i]
raw = cal @ np.linalg.inv(S).T + o
tm, ti = mag[:, 0], imu[:, 0]
j = np.clip(np.searchsorted(ti, tm), 1, len(ti) - 1)
j -= (tm - ti[j - 1]) < (ti[j] - tm)
ok = np.abs(ti[j] - tm) < 0.02
g = np.degrees(imu[j, 1:4] * FLU2FRD); a = imu[j, 4:7] * FLU2FRD
def hms(s): h, m, x = map(int, s.split(':')); return h * 3600 + m * 60 + x
t0, t1 = sys.argv[3], sys.argv[4]   # gio dia phuong HH:MM:SS
loc = np.array([(dt.datetime.fromtimestamp(x).hour * 3600 + dt.datetime.fromtimestamp(x).minute * 60 + dt.datetime.fromtimestamp(x).second) for x in tm])
rest = np.linalg.norm(g, axis=1) < 8
g = g - np.median(g[rest], axis=0)   # bias gyro con lai luc dung yen
print('bias gyro (dps)', np.round(np.median(imu[j, 1:4][rest] * FLU2FRD, axis=0) * 57.2958, 2))
sel = ok & (loc >= hms(t0)) & (loc < hms(t1))
# bo mau lap lai
keep = sel.copy(); keep[1:] &= np.any(np.diff(raw, axis=0) != 0, axis=1)
np.savez(sys.argv[2], t_pc=tm[keep], gyro_dps=g[keep], acc_mps2=a[keep], mag_raw_g=raw[keep], count=tm[keep])
print(f"{keep.sum()} mau, {dt.datetime.fromtimestamp(tm[keep][0]):%T}-{dt.datetime.fromtimestamp(tm[keep][-1]):%T}, khop imu {ok.mean()*100:.0f}%")
