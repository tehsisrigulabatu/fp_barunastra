#!/usr/bin/env python3
"""Uji offline FTP + docking TANPA Gazebo: dinamika WAM-V 3-DOF (parameter VRX) + deteksi kamera sintetis
(jarak ber-noise, FOV terbatas) -> peta landmark -> planner -> LOS + PID -> thrust.

Pakai:  python3 tests/sim_ftp_offline.py [--seed 1] [--noise 0.06] [--plot out.png]
Keluar dengan kode 1 kalau ada tabrakan atau gerbang yang tidak dilewati.
"""
import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from asv import config as C  # noqa: E402
from asv.control import BoatState, LOSGuidance, Pilot, StationKeeper  # noqa: E402
from asv.geometry import CourseFrame  # noqa: E402
from asv.planner import LandmarkMap, plan_path  # noqa: E402
from figs import CFG, layout  # noqa: E402


def truth():
    L = layout()
    ox, oy = CFG["W"] / 2, CFG["H"] / 2
    S = C.SCALE
    lm = [("green", (x - ox) * S, (y - oy) * S, C.BUOY_R) for x, y in L["green"]]
    lm += [("red", (x - ox) * S, (y - oy) * S, C.BUOY_R) for x, y in L["red"]]
    lm += [("yellow" if c == "Y" else "black", (x - ox) * S, (y - oy) * S, C.OBS_R) for x, y, c, _ in L["obs"]]
    gates = list(zip([(g[0] - ox) * S for g in L["green"]], L["green"], L["red"]))
    return lm, [((gx - ox) * S, (gy - oy) * S, (rx - ox) * S, (ry - oy) * S) for _, (gx, gy), (rx, ry) in gates]


class Boat:
    m, Iz, d = 230.0, 500.0, 1.03

    def __init__(self, x, y, psi):
        self.x, self.y, self.psi, self.u, self.v, self.r = x, y, psi, 0.0, 0.0, 0.0

    def step(self, L, R, dt):
        X = L + R - (100 * self.u + 150 * self.u * abs(self.u))
        Y = -(100 * self.v + 100 * self.v * abs(self.v))
        N = (R - L) * self.d - (800 * self.r + 800 * self.r * abs(self.r))
        self.u += X / self.m * dt
        self.v += Y / self.m * dt
        self.r += N / self.Iz * dt
        c, s = math.cos(self.psi), math.sin(self.psi)
        self.x += (c * self.u - s * self.v) * dt
        self.y += (s * self.u + c * self.v) * dt
        self.psi += self.r * dt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--noise", type=float, default=0.06, help="sigma relatif estimasi jarak")
    ap.add_argument("--plot", default=None)
    ap.add_argument("--miss", type=float, default=0.15, help="peluang deteksi terlewat per frame")
    a = ap.parse_args()
    random.seed(a.seed)

    lms, gates_true = truth()
    start = (-48.5, 0.0, 0.0)
    boat = Boat(*start)
    frame = CourseFrame(start[:2], start[2])
    dock_xy = frame.to_xy(C.SPOT_TO_SPOT, 0.0)
    lmap = LandmarkMap()
    los, pilot, st = LOSGuidance(), Pilot(), BoatState()
    start_sl, end_sl = (0.0, 0.0), (C.SPOT_TO_SPOT, 0.0)
    dt, t, traj = 0.02, 0.0, []
    last_plan, hold, docked_t = -1.0, None, None
    min_gap = {}
    crossed = {}
    prev = (boat.x, boat.y)
    while t < 400.0:
        st.update(t, boat.x, boat.y, boat.psi)
        # ---- deteksi sintetis 10 Hz (kamera depan: x=0.75 m, FOV 80 derajat, jarak noise)
        if int(t / dt) % 5 == 0:
            cx = boat.x + 0.75 * math.cos(boat.psi)
            cy = boat.y + 0.75 * math.sin(boat.psi)
            seen = set()
            for cls, x, y, rad in lms:
                rng = math.hypot(x - cx, y - cy)
                brg = math.atan2(y - cy, x - cx) - boat.psi
                brg = math.atan2(math.sin(brg), math.cos(brg))
                if abs(brg) > C.FRONT_CAM_HFOV / 2 - 0.03 or not (C.MAP_MIN_RANGE < rng < C.MAP_MAX_RANGE):
                    continue
                r_est = rng * (1 + random.gauss(0, a.noise))
                b_est = brg + random.gauss(0, math.radians(0.4))
                if random.random() < a.miss:
                    continue
                lm = lmap.add(cls, cx + r_est * math.cos(boat.psi + b_est), cy + r_est * math.sin(boat.psi + b_est), r_est, t)
                seen.add(lm.id)
            lmap.negative(cx, cy, boat.psi, C.FRONT_CAM_HFOV, seen)
        # ---- replan 2 Hz
        if hold is None and t - last_plan > 0.5:
            last_plan = t
            lmap.merge()
            wps, gates = plan_path(lmap, frame, start_sl, end_sl)
            xy = [(*frame.to_xy(w["s"], w["l"]), w["kind"]) for w in wps]
            los.set_path(xy, boat.x, boat.y)
        if hold is None:
            psi_d, e, rem, _ = los.step(boat.x, boat.y)
            if psi_d is None:
                psi_d = frame.heading
            u_d = los.speed_for(C.U_CRUISE) if rem > C.DOCK_SLOW_DIST else max(C.U_DOCK_MIN, C.U_CRUISE * rem / C.DOCK_SLOW_DIST)
            if frame.to_sl((boat.x, boat.y))[0] < C.SLOT_EXIT:
                u_d = min(u_d, C.U_EXIT)
            (L, R), _ = pilot.command(st, psi_d, u_d, t)
            if rem < C.DOCK_TOL:
                hold = StationKeeper()
                hold.set(*dock_xy, frame.heading)
                docked_t = t
        else:
            (L, R), e_f, e_psi = hold.command(st)
            if t - docked_t > 20:
                break
        boat.step(L, R, dt)
        t += dt
        traj.append((boat.x, boat.y))
        # ---- metrik: jarak lambung ke tiap benda, pemotongan garis gerbang
        for cls, x, y, rad in lms:
            g = math.hypot(boat.x - x, boat.y - y) - rad - C.HALF_BEAM
            min_gap[(cls, x, y)] = min(min_gap.get((cls, x, y), 1e9), g)
        for i, (gx, gy, rx, ry) in enumerate(gates_true):
            if prev[0] < gx <= boat.x and i not in crossed:
                a_ = (gx - prev[0]) / max(boat.x - prev[0], 1e-9)
                yc = prev[1] + a_ * (boat.y - prev[1])
                crossed[i] = (yc - (gy + ry) / 2, min(gy, ry) < yc < max(gy, ry), abs(gy - ry))
        prev = (boat.x, boat.y)

    ok = True
    print(f"waktu {t:.1f}s, posisi akhir ({boat.x:.2f}, {boat.y:.2f}) psi={math.degrees(boat.psi):.1f}deg, "
          f"galat docking {math.hypot(boat.x - dock_xy[0], boat.y - dock_xy[1]):.2f} m")
    for i in range(len(gates_true)):
        if i not in crossed:
            print(f"  GATE {i + 1}: TIDAK DILEWATI")
            ok = False
            continue
        off, inside, w = crossed[i]
        print(f"  GATE {i + 1} (lebar {w:.1f} m): offset dari tengah {off:+.2f} m {'OK' if inside else 'DI LUAR'}")
        ok &= inside and abs(off) < 0.25 * w
    for (cls, x, y), g in sorted(min_gap.items(), key=lambda kv: kv[1]):
        flag = "TABRAKAN" if g < 0 else ("dekat" if g < 0.5 else "")
        print(f"  {cls:6s} ({x:6.1f},{y:6.1f}) jarak lambung minimum {g:5.2f} m {flag}")
        ok &= g > 0
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(14, 4))
        colors = {"green": "g", "red": "r", "yellow": "y", "black": "k"}
        for cls, x, y, rad in lms:
            ax.add_patch(plt.Circle((x, y), rad, color=colors[cls]))
        for m in lmap.confirmed():
            ax.plot(m.x, m.y, "x", color="m")
        ax.plot(*zip(*traj), "b-", lw=1)
        ax.plot(*zip(*[w[:2] for w in los.wps]), "c.--", lw=0.6)
        ax.set_aspect("equal")
        ax.set_xlim(-60, 60)
        ax.set_ylim(-30, 30)
        fig.savefig(a.plot, dpi=110, bbox_inches="tight")
        print("plot:", a.plot)
    print("HASIL:", "LULUS" if ok else "GAGAL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
