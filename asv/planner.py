"""Peta landmark + perencanaan lintasan FTP (murni python, bisa diuji tanpa Gazebo).

Aturan FTP dari guidebook: hijau di KIRI, merah di KANAN (arah Spot 1 -> Spot 2), kapal wajib lewat
tengah tiap gerbang, rintangan kuning/hitam ada di dalam lintasan dan harus dihindari.

Strategi:
  1. Setiap deteksi kamera -> posisi dunia -> digabung ke landmark (rata-rata berbobot 1/jarak^2,
     karena estimasi jarak makin akurat saat dekat).
  2. Pasangan hijau-merah yang sejajar dengan arah lintasan -> GERBANG, waypoint = titik tengahnya
     (MUST-PASS). Kapal dianggap lulus gerbang saat memotong garis hijau-merah.
  3. Rintangan yang terlalu dekat dengan jalur di antara dua gerbang -> titik hindar di sisi yang
     lebih lapang, tetap di dalam koridor (tidak menabrak baris buoy).
"""
import math

import numpy as np

from . import config as C


class Landmark:
    __slots__ = ("id", "cls", "x", "y", "w", "hits", "last_t", "min_rng", "misses")

    def __init__(self, lid, cls, x, y, w, t, rng=1e9):
        self.id, self.cls, self.x, self.y, self.w, self.hits, self.last_t = lid, cls, x, y, w, 1, t
        self.min_rng = rng
        self.misses = 0

    def update(self, x, y, w, t, rng=1e9):
        self.min_rng = min(self.min_rng, rng)
        tot = self.w + w
        self.x = (self.x * self.w + x * w) / tot
        self.y = (self.y * self.w + y * w) / tot
        self.w = min(tot, 1e6)
        self.hits += 1
        self.last_t = t

    @property
    def xy(self):
        return (self.x, self.y)


class LandmarkMap:
    def __init__(self):
        self.lms = []
        self._next = 0

    def add(self, cls, x, y, rng, t):
        w = 1.0 / max(rng, 1.0) ** 2
        best, bd = None, max(C.MAP_ASSOC_R, C.MAP_ASSOC_K * rng)
        for lm in self.lms:
            if lm.cls != cls:
                continue
            d = math.hypot(lm.x - x, lm.y - y)
            if d < bd:
                best, bd = lm, d
        if best is None:
            best = Landmark(self._next, cls, x, y, w, t, rng)
            self._next += 1
            self.lms.append(best)
        else:
            best.update(x, y, w, t, rng)
            best.misses = max(0, best.misses - 1)
        return best

    def negative(self, cam_x, cam_y, cam_yaw, hfov, seen_ids, max_rng=C.MAP_NEG_RANGE):
        """Landmark yang berada jelas di dalam FOV dan dekat tapi TIDAK terdeteksi di frame ini -> miss.
        Landmark palsu (dari deteksi jauh yang meleset) akan terhapus sendiri."""
        for lm in list(self.lms):
            if lm.id in seen_ids:
                continue
            dx, dy = lm.x - cam_x, lm.y - cam_y
            rng = math.hypot(dx, dy)
            brg = math.atan2(math.sin(math.atan2(dy, dx) - cam_yaw), math.cos(math.atan2(dy, dx) - cam_yaw))
            if C.MAP_MIN_RANGE * 2 < rng < max_rng and abs(brg) < hfov / 2 - 0.12:
                lm.misses += 1
                if lm.misses > 6 and lm.misses > 0.6 * lm.hits:
                    self.lms.remove(lm)

    def merge(self):
        """Gabungkan landmark kelas sama yang sudah konvergen ke titik yang sama (duplikat dari deteksi jauh)."""
        changed = True
        while changed:
            changed = False
            for i, a in enumerate(self.lms):
                for b in self.lms[i + 1:]:
                    if a.cls == b.cls and math.hypot(a.x - b.x, a.y - b.y) < C.MAP_ASSOC_R:
                        keep, drop = (a, b) if a.w >= b.w else (b, a)
                        keep.update(drop.x, drop.y, drop.w, max(a.last_t, b.last_t), drop.min_rng)
                        keep.hits += drop.hits - 1
                        self.lms.remove(drop)
                        changed = True
                        break
                if changed:
                    break

    def confirmed(self, cls=None, min_hits=C.MAP_CONFIRM_HITS):
        return [m for m in self.lms if m.hits >= min_hits and m.min_rng <= C.MAP_CONFIRM_RANGE
                and (cls is None or m.cls == cls)]


class Gate:
    __slots__ = ("green", "red", "mid", "s", "l", "width", "passed", "pass_offset")

    def __init__(self, g, r, frame):
        self.green, self.red = g, r
        self.mid = ((g.x + r.x) / 2.0, (g.y + r.y) / 2.0)
        self.s, self.l = frame.to_sl(self.mid)
        self.width = math.hypot(g.x - r.x, g.y - r.y)
        self.passed = False
        self.pass_offset = None

    @property
    def key(self):
        return (self.green.id, self.red.id)


def pair_gates(lmap, frame, direction=+1):
    """Pasangkan buoy hijau-merah menjadi gerbang. direction=+1 Spot1->Spot2 (hijau kiri), -1 pulang."""
    greens = [g for g in lmap.confirmed("green") if abs(frame.to_sl(g.xy)[1]) < C.CORRIDOR_HALF]
    reds = [r for r in lmap.confirmed("red") if abs(frame.to_sl(r.xy)[1]) < C.CORRIDOR_HALF]
    cands = []
    for g in greens:
        sg, lg = frame.to_sl(g.xy)
        for r in reds:
            sr, lr = frame.to_sl(r.xy)
            w = math.hypot(sg - sr, lg - lr)
            if not (C.GATE_W_MIN <= w <= C.GATE_W_MAX):
                continue
            if abs(sg - sr) / w > C.GATE_MAX_SKEW:        # pasangan harus melintang terhadap lintasan
                continue
            if (lg - lr) * direction <= 0:                # hijau harus di kiri arah gerak
                continue
            cands.append((abs(sg - sr), w, g, r))
    cands.sort(key=lambda c: (c[0], c[1]))
    used_g, used_r, gates = set(), set(), []
    for _, _, g, r in cands:
        if g.id in used_g or r.id in used_r:
            continue
        used_g.add(g.id)
        used_r.add(r.id)
        gates.append(Gate(g, r, frame))
    gates.sort(key=lambda gt: gt.s * direction)
    return gates


def _interp_row(row, s):
    """row: list (s, l) terurut s. Interpolasi linear, ekstrapolasi konstan."""
    if not row:
        return None
    if s <= row[0][0]:
        return row[0][1]
    if s >= row[-1][0]:
        return row[-1][1]
    for (s0, l0), (s1, l1) in zip(row, row[1:]):
        if s0 <= s <= s1:
            a = (s - s0) / max(s1 - s0, 1e-6)
            return l0 + a * (l1 - l0)
    return row[-1][1]


def corridor_bounds(lmap, frame, s):
    """Batas lateral aman (l_min, l_max) untuk garis tengah kapal di posisi s."""
    gr = sorted(frame.to_sl(m.xy) for m in lmap.confirmed("green"))
    rd = sorted(frame.to_sl(m.xy) for m in lmap.confirmed("red"))
    lg, lr = _interp_row(gr, s), _interp_row(rd, s)
    hi = (lg - C.BUOY_CLEAR) if lg is not None else +C.CORRIDOR_HALF
    lo = (lr + C.BUOY_CLEAR) if lr is not None else -C.CORRIDOR_HALF
    return lo, hi


def _choose_sides(gates, obs_sl, lmap, frame, s_grid, l_init):
    """Tentukan sisi lewat tiap rintangan (+1 kiri, -1 kanan) dari jalur awal (garis lurus antar gerbang)."""
    sides = []
    for so, lo_, o in obs_sl:
        lp = float(np.interp(so, s_grid, l_init))
        if abs(lp - lo_) >= C.OBS_CLEAR + 0.5:
            continue                                   # jauh dari jalur, abaikan
        l_cmin, l_cmax = corridor_bounds(lmap, frame, so)
        left_ok = lo_ + C.OBS_CLEAR <= l_cmax
        right_ok = lo_ - C.OBS_CLEAR >= l_cmin
        if left_ok and right_ok:
            side = +1 if lp >= lo_ else -1             # sisi tempat jalur sudah berada = belokan terkecil
        elif left_ok or right_ok:
            side = +1 if left_ok else -1
        else:
            side = +1 if (l_cmax - lo_) > (lo_ - l_cmin) else -1
        sides.append((so, lo_, side, o))
    return sides


def plan_path(lmap, frame, start_sl, end_sl, direction=+1, ds=1.0, iters=1500,
              start_straight=None, end_straight=None):
    """Elastic band: lintasan lateral l(s) yang mulus dengan kendala

      - KERAS : lewat TEPAT di titik tengah tiap gerbang, lurus sepanjang +/- GATE_STRAIGHT (memotong tegak lurus)
      - rintangan : jarak ke pusat rintangan >= OBS_CLEAR, di sisi yang dipilih
      - koridor   : tidak mendekati baris buoy hijau/merah (BUOY_CLEAR)
      - awal/akhir: posisi tetap dan sejajar sumbu lintasan

    Hasil: list waypoint dict(s, l, kind, ref) (rapat, tiap ~2 m) dan list Gate.
    """
    gates = pair_gates(lmap, frame, direction)
    s0, s1 = start_sl[0], end_sl[0]
    n = max(8, int(abs(s1 - s0) / ds) + 1)
    sg = np.linspace(s0, s1, n)                         # urut sesuai arah tempuh
    lo_s, hi_s = min(s0, s1), max(s0, s1)
    gates_in = [g for g in gates if lo_s + C.GATE_STRAIGHT < g.s < hi_s - C.GATE_STRAIGHT]

    # jalur awal: garis lurus antar titik keras
    hs = [s0] + [g.s for g in sorted(gates_in, key=lambda g: g.s * direction)] + [s1]
    hl = [start_sl[1]] + [g.l for g in sorted(gates_in, key=lambda g: g.s * direction)] + [end_sl[1]]
    order = np.argsort(hs)
    l = np.interp(sg, np.array(hs)[order], np.array(hl)[order])

    obs = [m for m in lmap.confirmed() if m.cls in ("yellow", "black")]
    obs_sl = [frame.to_sl(o.xy) + (o,) for o in obs]
    sides = _choose_sides(gates_in, obs_sl, lmap, frame, sg if direction > 0 else sg[::-1],
                          l if direction > 0 else l[::-1])

    # batas koridor per titik grid
    bounds = np.array([corridor_bounds(lmap, frame, s_) for s_ in sg])
    lo_b, hi_b = bounds[:, 0], bounds[:, 1]
    # topeng kendala
    fixed = np.zeros(n, bool)
    fixed_val = np.zeros(n)
    ss = C.SLOT_EXIT if start_straight is None else start_straight
    es = C.DOCK_STRAIGHT if end_straight is None else end_straight
    m = np.abs(sg - s0) <= max(ss, 1.5 * ds)          # keluar slot Spot 1 lurus
    fixed[m], fixed_val[m] = True, start_sl[1]
    m = np.abs(sg - s1) <= max(es, 1.5 * ds)          # masuk slot Spot 2 lurus
    fixed[m], fixed_val[m] = True, end_sl[1]
    for g in gates_in:
        m = np.abs(sg - g.s) <= C.GATE_STRAIGHT
        fixed[m], fixed_val[m] = True, g.l
    obs_masks = []
    for so, lo_, side, o in sides:
        dsv = sg - so
        m = np.abs(dsv) < C.OBS_CLEAR
        need = lo_ + side * np.sqrt(np.maximum(C.OBS_CLEAR ** 2 - dsv ** 2, 0.0))
        obs_masks.append((m, need, side))

    a = 0.05                                            # langkah biharmonik (stabil < 1/16)
    for _ in range(iters):
        d4 = np.zeros(n)
        d4[2:-2] = l[:-4] - 4 * l[1:-3] + 6 * l[2:-2] - 4 * l[3:-1] + l[4:]
        l = l - a * d4
        l = np.minimum(np.maximum(l, lo_b), hi_b)
        for m, need, side in obs_masks:
            if side > 0:
                l[m] = np.maximum(l[m], need[m])
            else:
                l[m] = np.minimum(l[m], need[m])
        l[fixed] = fixed_val[fixed]

    out = []
    step = max(1, int(round(2.0 / ds)))
    gate_s = {round(g.s, 3): g for g in gates_in}
    for i in range(0, n, step):
        out.append(dict(s=float(sg[i]), l=float(l[i]), kind="path", ref=None))
    if out[-1]["s"] != float(sg[-1]):
        out.append(dict(s=float(sg[-1]), l=float(l[-1]), kind="path", ref=None))
    for g in gates_in:                                  # sisipkan titik gerbang persis
        out.append(dict(s=g.s, l=g.l, kind="gate", ref=g))
    out.sort(key=lambda w: w["s"] * direction)
    out[0]["kind"], out[-1]["kind"] = "start", "end"
    _ = gate_s
    return out, gates


def min_clearance(path_xy, lmap):
    """Jarak terdekat polyline jalur ke tiap landmark (untuk logging / uji)."""
    res = {}
    P = np.asarray(path_xy, dtype=float)
    for m in lmap.confirmed():
        best = 1e9
        for a, b in zip(P, P[1:]):
            ab = b - a
            t = np.clip(np.dot(np.array(m.xy) - a, ab) / max(np.dot(ab, ab), 1e-9), 0, 1)
            best = min(best, float(np.linalg.norm(a + t * ab - np.array(m.xy))))
        res[(m.cls, m.id)] = best
    return res
