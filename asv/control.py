"""Kendali kapal: LOS path following, PID heading & kecepatan, station keeping. Murni python.

Keluaran selalu (thrust_kiri, thrust_kanan) dalam Newton untuk WAM-V (2 thruster diferensial).
Konvensi: diff > 0 = putar kiri (CCW), kiri = F - diff, kanan = F + diff.
"""
import math

from . import config as C
from .config import clamp, wrap


class BoatState:
    """Estimasi kecepatan dari pose (beda hingga + low-pass)."""

    def __init__(self):
        self.t = None
        self.x = self.y = self.psi = 0.0
        self.u = self.v = self.r = 0.0

    def update(self, t, x, y, psi):
        if self.t is not None and t > self.t + 1e-4:
            dt = t - self.t
            vx, vy = (x - self.x) / dt, (y - self.y) / dt
            c, s = math.cos(psi), math.sin(psi)
            a = min(1.0, dt / 0.25)
            self.u += a * ((c * vx + s * vy) - self.u)
            self.v += a * ((-s * vx + c * vy) - self.v)
            self.r += a * (wrap(psi - self.psi) / dt - self.r)
        if self.t is None or t > self.t + 1e-4:
            self.t, self.x, self.y, self.psi = t, x, y, psi


def mix(F, diff):
    diff = clamp(diff, -C.MAX_DIFF, C.MAX_DIFF)
    left, right = F - diff, F + diff
    # prioritaskan belok: kalau saturasi, kurangi F, bukan diff
    over = max(left, right) - C.THRUST_MAX
    if over > 0:
        left, right = left - over, right - over
    under = C.THRUST_MIN - min(left, right)
    if under > 0:
        left, right = left + under, right + under
    return clamp(left, C.THRUST_MIN, C.THRUST_MAX), clamp(right, C.THRUST_MIN, C.THRUST_MAX)


def drag_ff(u):
    return C.K_FF * 0.5 * (C.DRAG_U * u + C.DRAG_UU * u * abs(u))


class Pilot:
    """Pengendali heading + kecepatan dengan integrator anti-windup."""

    def __init__(self):
        self.iu = 0.0
        self.t = None

    def reset(self):
        self.iu = 0.0
        self.t = None

    def command(self, st, psi_d, u_d, t):
        dt = 0.05 if self.t is None else clamp(t - self.t, 1e-3, 0.3)
        self.t = t
        e_psi = wrap(psi_d - st.psi)
        diff = C.KP_PSI * e_psi - C.KD_PSI * st.r
        # perlambat saat heading masih jauh (cegah lintasan melebar)
        u_cmd = u_d * max(0.25, math.cos(clamp(e_psi, -1.4, 1.4))) if u_d > 0 else u_d
        e_u = u_cmd - st.u
        self.iu = clamp(self.iu + e_u * dt, -2.0, 2.0)
        F = drag_ff(u_cmd) + C.KP_U * e_u + C.KI_U * self.iu
        return mix(F, diff), e_psi


class LOSGuidance:
    """Path following pada polyline (pure pursuit / LOS dengan carrot sepanjang lintasan).

    Posisi kapal diproyeksikan ke polyline (progres monoton), target = titik sejauh LOOKAHEAD di depan
    proyeksi SEPANJANG lintasan, sehingga tikungan di depan sudah diantisipasi. Waypoint bertanda
    (mis. 'gate') dilaporkan saat proyeksi melewatinya.
    """

    def __init__(self, lookahead=None):
        self.wps, self.cum = [], []
        self.prog = 0.0
        self.reported = set()
        self.la = lookahead or C.LOOKAHEAD

    def set_path(self, wps_xy, x=None, y=None):
        """wps_xy: list (x, y, tag). Progres dihitung ulang dari posisi kapal (x, y) bila diberikan."""
        self.wps = list(wps_xy)
        self.cum = [0.0]
        for (ax, ay, _), (bx, by, _) in zip(self.wps, self.wps[1:]):
            self.cum.append(self.cum[-1] + math.hypot(bx - ax, by - ay))
        self.reported = {i for i in self.reported if i < len(self.wps)}
        if x is not None:
            self.prog = self._project(x, y, 0, len(self.wps) - 1)[0]

    def _project(self, x, y, i0, i1):
        best = (0.0, 1e18, 0.0)
        for i in range(max(0, i0), min(i1, len(self.wps) - 1)):
            ax, ay = self.wps[i][:2]
            bx, by = self.wps[i + 1][:2]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-9 else clamp(((x - ax) * dx + (y - ay) * dy) / L2, 0.0, 1.0)
            px, py = ax + t * dx, ay + t * dy
            d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 < best[1]:
                L = math.sqrt(L2)
                e = 0.0 if L < 1e-9 else (-(x - ax) * dy + (y - ay) * dx) / L
                best = (self.cum[i] + t * L, d2, e)
        return best[0], best[2]

    def point_at(self, s):
        s = clamp(s, 0.0, self.cum[-1])
        for i in range(len(self.cum) - 1):
            if self.cum[i + 1] >= s:
                L = self.cum[i + 1] - self.cum[i]
                a = 0.0 if L < 1e-9 else (s - self.cum[i]) / L
                ax, ay = self.wps[i][:2]
                bx, by = self.wps[i + 1][:2]
                return ax + a * (bx - ax), ay + a * (by - ay)
        return self.wps[-1][:2]

    def turn_ahead(self, dist):
        """Total perubahan arah lintasan (rad) dari proyeksi kapal sampai dist di depan."""
        if len(self.wps) < 3:
            return 0.0
        pts = [self.point_at(self.prog + d) for d in (0.0, dist / 3, 2 * dist / 3, dist)]
        hs = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(pts, pts[1:])
              if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-3]
        return sum(abs(wrap(h1 - h0)) for h0, h1 in zip(hs, hs[1:]))

    def speed_for(self, u_max):
        """Kecepatan dikurangi di bagian lintasan yang berbelok tajam."""
        k = self.turn_ahead(2.0 * self.la)
        return u_max * clamp(1.0 - 0.9 * k, 0.45, 1.0)

    def step(self, x, y):
        """-> (psi_d, cross_track(+ = kapal di kiri jalur), sisa_jarak, list tag waypoint yang baru dilewati)."""
        if len(self.wps) < 2:
            return None, 0.0, 0.0, []
        i = 0
        while i < len(self.cum) - 1 and self.cum[i + 1] < self.prog - 1e-6:
            i += 1
        s_new, e = self._project(x, y, i, i + 12)
        self.prog = max(self.prog, s_new)
        reached = []
        for j, c in enumerate(self.cum):
            if c <= self.prog + 1e-6 and j not in self.reported:
                self.reported.add(j)
                reached.append(self.wps[j][2])
        tx, ty = self.point_at(self.prog + self.la)
        rem = self.cum[-1] - self.prog
        if math.hypot(tx - x, ty - y) < 0.3:
            return None, e, rem, reached
        return math.atan2(ty - y, tx - x), e, rem, reached


class StationKeeper3DOF:
    """Tahan x, y, yaw dengan dua mesin azimuth. PID di kerangka kapal -> (Fx, Fy, Mz) -> alokasi
    pseudo-invers ke gaya (Fx_i, Fy_i) tiap mesin -> thrust & sudut mesin."""

    def __init__(self):
        self.ref = None
        self.ix = self.iy = 0.0
        self.t = None
        (x1, y1), (x2, y2) = C.THRUSTER_XY
        import numpy as np
        B = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [-y1, x1, -y2, x2]], dtype=float)
        self.Bp = B.T @ np.linalg.inv(B @ B.T)
        self.prev_az = [0.0, 0.0]

    def set(self, x, y, psi):
        self.ref = (x, y, psi)
        self.ix = self.iy = 0.0

    def command(self, st, t):
        x0, y0, psi0 = self.ref
        dt = 0.05 if self.t is None else clamp(t - self.t, 1e-3, 0.3)
        self.t = t
        c, s = math.cos(st.psi), math.sin(st.psi)
        ex = c * (x0 - st.x) + s * (y0 - st.y)            # galat di kerangka kapal
        ey = -s * (x0 - st.x) + c * (y0 - st.y)
        epsi = wrap(psi0 - st.psi)
        self.ix = clamp(self.ix + ex * dt, -10, 10)
        self.iy = clamp(self.iy + ey * dt, -10, 10)
        Fx = clamp(C.SK_KP[0] * ex + C.SK_KI[0] * self.ix - C.SK_KD[0] * st.u, -C.SK_MAX[0], C.SK_MAX[0])
        Fy = clamp(C.SK_KP[1] * ey + C.SK_KI[1] * self.iy - C.SK_KD[1] * st.v, -C.SK_MAX[1], C.SK_MAX[1])
        Mz = clamp(C.SK_KP[2] * epsi - C.SK_KD[2] * st.r, -C.SK_MAX[2], C.SK_MAX[2])
        f = self.Bp @ [Fx, Fy, Mz]
        out = []
        for i in range(2):
            fx, fy = f[2 * i], f[2 * i + 1]
            T = math.hypot(fx, fy)
            prev = self.prev_az[i]
            if T < 2.0:                                   # gaya kecil: pertahankan sudut lama (mesin tidak berputar-putar)
                az = prev
                T = fx * math.cos(az) + fy * math.sin(az)
            else:
                # dua cara menghasilkan gaya yang sama: (T, a) atau (-T, a +- pi). Pilih yang paling dekat
                # dengan sudut mesin sekarang -> tidak ada lompatan 180 derajat saat gaya ~ menyamping.
                a = math.atan2(fy, fx)
                cands = [(a, T), (a - math.pi, -T), (a + math.pi, -T)]
                az, T = min(((c, tt) for c, tt in cands if abs(c) <= C.AZ_MAX),
                            key=lambda ct: abs(ct[0] - prev))
                # mesin berputar pelan: selama belum sampai sudut target, kurangi thrust (gaya ke arah salah)
                T *= max(0.0, math.cos(az - prev)) ** 2
            # batasi laju perubahan sudut perintah (~1.5 rad/s) supaya estimasi arah gaya tetap benar
            az = prev + clamp(az - prev, -0.08, 0.08)
            self.prev_az[i] = az
            out.append((clamp(T, C.THRUST_MIN, C.THRUST_MAX), az))
        return out, (ex, ey, epsi)


class StationKeeper:
    """Tahan posisi sepanjang sumbu kapal + heading. Lateral tidak bisa dikendalikan (2 thruster),
    tapi di dalam slot dermaga lateral dibatasi dinding."""

    def __init__(self):
        self.ref = None

    def set(self, x, y, psi):
        self.ref = (x, y, psi)

    def command(self, st):
        x0, y0, psi0 = self.ref
        c, s = math.cos(st.psi), math.sin(st.psi)
        e_f = c * (x0 - st.x) + s * (y0 - st.y)
        e_psi = wrap(psi0 - st.psi)
        F = clamp(C.HOLD_KP_X * e_f - C.HOLD_KD_X * st.u, -C.HOLD_MAX_F, C.HOLD_MAX_F)
        diff = clamp(C.HOLD_KP_PSI * e_psi - C.HOLD_KD_PSI * st.r, -C.HOLD_MAX_DIFF, C.HOLD_MAX_DIFF)
        return mix(F, diff), e_f, e_psi
