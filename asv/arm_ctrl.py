"""Kendali lengan crane (yaw, slide, lift) di atas WAM-V + urutan pick & place berbasis visi.

Perbaikan utama dibanding versi lama:
  - IK analitik (silindris) dari titik dunia -> sendi, bukan servo piksel coba-coba dengan tanda +-1.
  - Setiap gerakan = trajektori ramp terbatas kecepatan, lalu TUNGGU joint_state mencapai target
    (dengan toleransi & timeout), bukan sleep buta.
  - Posisi barang/zona dari kamera gripper diproyeksikan ke bidang z yang diketahui (sinar x bidang),
    diulang sampai koreksi < REFINE_TOL (closed loop), baru turun dan pegang.
  - Tinggi lift dihitung dari z gripper aktual, bukan daftar angka tetap.
"""
import math
import time

from . import config as C
from .config import clamp
from .geometry import boat_to_world, pose_T, world_to_boat


class ArmIK:
    @staticmethod
    def boat_xy_to_joints(bx, by, prev_yaw=0.0):
        rx, ry = bx - C.ARM_PIVOT[0], by - C.ARM_PIVOT[1]
        yaw = math.atan2(ry, rx)
        r = math.hypot(rx, ry)
        # sendi yaw dibatasi +-pi (tidak bisa melingkar). Titik di belakang kapal (yaw ~ +-pi) bisa dicapai
        # dari dua sisi; kalau sisi seberang hanya beda sedikit, tetap di sisi sekarang (hindari putar 360 derajat).
        if abs(prev_yaw) > 2.5 and yaw * prev_yaw < 0:
            alt = math.copysign(C.ARM_YAW_LIM, prev_yaw)
            err = r * abs(math.atan2(math.sin(yaw - alt), math.cos(yaw - alt)))
            if err < 0.12:
                yaw = alt
        if abs(yaw) > C.ARM_YAW_LIM:
            yaw = math.copysign(C.ARM_YAW_LIM, prev_yaw if abs(prev_yaw) > 0.3 else yaw)
        slide = math.hypot(rx, ry) - C.ARM_CARR0
        return yaw, slide

    @staticmethod
    def joints_to_boat_xy(yaw, slide):
        r = C.ARM_CARR0 + slide
        return C.ARM_PIVOT[0] + r * math.cos(yaw), C.ARM_PIVOT[1] + r * math.sin(yaw)

    @staticmethod
    def reachable(bx, by):
        rx, ry = bx - C.ARM_PIVOT[0], by - C.ARM_PIVOT[1]
        s = math.hypot(rx, ry) - C.ARM_CARR0
        return C.ARM_SLIDE_MIN <= s <= C.ARM_SLIDE_MAX - 0.1


class ArmController:
    def __init__(self, arm, perception, poses, logger, killed):
        """arm: lengan.Arm (gz transport), poses: PoseSource, killed: callable -> bool."""
        self.arm, self.per, self.poses, self.log, self.killed = arm, perception, poses, logger, killed
        self.q = {"yaw": 0.0, "slide": 0.0, "lift": 0.0}       # perintah terakhir
        self.holding = None

    # ------------------------------------------------------------------ sendi
    def joint(self, name):
        for k, v in self.arm.joints.items():
            if k == name or k.endswith("::" + name) or k.endswith("/" + name):
                return v
        return None

    def sync(self):
        """Mulai dari posisi sendi aktual (kalau joint_state tersedia)."""
        for k in self.q:
            v = self.joint(k)
            if v is not None:
                self.q[k] = v

    def move(self, yaw=None, slide=None, lift=None, timeout=25.0):
        tgt = dict(self.q)
        if yaw is not None:
            tgt["yaw"] = clamp(yaw, -C.ARM_YAW_LIM, C.ARM_YAW_LIM)
        if slide is not None:
            tgt["slide"] = clamp(slide, C.ARM_SLIDE_MIN, C.ARM_SLIDE_MAX)
        if lift is not None:
            tgt["lift"] = clamp(lift, C.ARM_LIFT_MIN, C.ARM_LIFT_MAX)
        # kecepatan putar diturunkan saat lengan menjulur (kecepatan ujung dibatasi ~0.8 m/s)
        reach = C.ARM_CARR0 + max(self.q["slide"], tgt["slide"])
        vmax = dict(C.ARM_VMAX)
        vmax["yaw"] = min(vmax["yaw"], 0.8 / reach)
        T = max(abs(tgt[k] - self.q[k]) / vmax[k] for k in tgt)
        start = dict(self.q)
        dt = 0.05
        n = max(1, int(math.ceil(T / dt)))
        for i in range(1, n + 1):
            while self.killed():
                time.sleep(0.1)
            a = i / n
            a = a * a * (3 - 2 * a)                           # smoothstep: tanpa sentakan di awal/akhir
            for k in tgt:
                self.q[k] = start[k] + (tgt[k] - start[k]) * a
            self.arm.cmd(**self.q)
            time.sleep(dt)
        return self.wait_settled(tgt, timeout)

    def wait_settled(self, tgt, timeout):
        t_end = time.time() + timeout
        while time.time() < t_end:
            self.arm.cmd(**tgt)
            ok = True
            have = False
            for k, v in tgt.items():
                j = self.joint(k)
                if j is None:
                    continue
                have = True
                if abs(j - v) > C.ARM_TOL[k]:
                    ok = False
            if not have:                       # joint_state tidak ada: beri waktu settle
                time.sleep(0.6)
                return True
            if ok:
                time.sleep(0.15)
                return True
            time.sleep(0.05)
        errs = {k: (None if self.joint(k) is None else round(self.joint(k) - v, 3)) for k, v in tgt.items()}
        self.log.warn(f"[ARM] belum mencapai target dalam {timeout:.0f}s, galat {errs}")
        return False

    # ------------------------------------------------------------------ geometri
    def gripper_z_world(self):
        """z gripper di dunia saat ini (dari pose Gazebo link gripper relatif model lengan)."""
        ap = self.poses.get("asv_arm") or self.poses.get("wamv")
        g = self.poses.get("gripper")
        if ap is None or g is None:
            return None
        return float((pose_T(ap) @ pose_T(g))[2, 3])

    def lift_for_world_z(self, z_grip):
        """Nilai sendi lift supaya gripper berada di z dunia z_grip."""
        zg = self.gripper_z_world()
        lift_now = self.joint("lift")
        if zg is None or lift_now is None:
            wamv = self.poses.get("wamv")
            z0 = (wamv[2] if wamv else 0.0) + C.ARM_GRIP_Z0
            return clamp(z_grip - z0, C.ARM_LIFT_MIN, C.ARM_LIFT_MAX)
        return clamp(lift_now + (z_grip - zg), C.ARM_LIFT_MIN, C.ARM_LIFT_MAX)

    def goto_world_xy(self, x, y, lift=None):
        boat = self.poses.get("wamv")
        bx, by = world_to_boat(boat, (x, y))
        yaw, slide = ArmIK.boat_xy_to_joints(bx, by, self.q["yaw"])
        return self._goto_joints(yaw, slide, lift)

    def goto_boat_xy(self, bx, by, lift=None):
        yaw, slide = ArmIK.boat_xy_to_joints(bx, by, self.q["yaw"])
        return self._goto_joints(yaw, slide, lift)

    def _goto_joints(self, yaw, slide, lift):
        # putaran besar: tarik slide dulu (momen inersia kecil, kapal tidak ikut berputar)
        if abs(yaw - self.q["yaw"]) > 0.35 and self.q["slide"] > 1.5:
            self.move(slide=1.0)
        self.move(yaw=yaw)
        self.move(slide=slide)
        if lift is not None:
            self.move(lift=lift)

    def carry_lift(self, surface_z):
        """Lift aman saat mengayun: alas barang di atas (a) permukaan tujuan + SAFE_CARRY_DZ,
        (b) puncak barang lain di dermaga, (c) puncak lidar/kamera WAM-V (barang ikut lewat atas kapal)."""
        w = self.poses.get("wamv")
        boat_z = w[2] if w else 0.0
        bottom = max(surface_z + C.SAFE_CARRY_DZ, C.DOCK_TOP_Z + C.ITEM + 0.15,
                     boat_z + C.WAMV_SENSOR_TOP + C.CARRY_CLEAR, self._deck_wall_world_z() + C.SAFE_CARRY_DZ)
        return self.lift_for_world_z(bottom + C.ITEM / 2 + C.HANG)

    def _deck_wall_world_z(self):
        w = self.poses.get("wamv")
        return (w[2] if w else 0.0) + C.CONTAINER_WALL_TOP

    def gripper_world_xy(self):
        try:
            gx, gy, _ = self.arm.gripper_world()
            return gx, gy
        except Exception:
            return None

    # ------------------------------------------------------------------ visi
    def refine(self, want, guess_xy, z_plane, lift_view=0.0, max_jump=1.2):
        """Posisikan kamera di atas guess_xy, deteksi `want`, lompat ke hasil proyeksi; ulang sampai
        koreksi < REFINE_TOL. -> (xy, cls) atau (None, None)."""
        xy, cls = guess_xy, None
        for it in range(C.REFINE_ITERS):
            self.goto_world_xy(xy[0], xy[1], lift=lift_view)
            time.sleep(0.4)                                # tunggu frame baru setelah diam
            dets = []
            for _ in range(4):
                dets = self.per.detect(want, z_plane)
                if dets:
                    break
                time.sleep(0.25)
            if not dets:
                self.log.warn(f"[ARM-VIS] {want} tidak terlihat di sekitar ({xy[0]:.2f},{xy[1]:.2f}) iter {it}")
                return (None, None) if it == 0 else (xy, cls)
            # pilih deteksi terdekat ke tebakan (bukan barang tetangga)
            dets.sort(key=lambda d: math.hypot(d[2][0] - xy[0], d[2][1] - xy[1]))
            c, conf, p, _ = dets[0]
            jump = math.hypot(p[0] - xy[0], p[1] - xy[1])
            if jump > max_jump and it == 0:
                self.log.warn(f"[ARM-VIS] deteksi {c} melompat {jump:.2f} m dari tebakan, diabaikan")
                return None, None
            xy, cls = p, c
            self.log.info(f"[ARM-VIS] {want}: iter {it} -> {c} ({p[0]:.2f},{p[1]:.2f}) koreksi {jump:.3f} m")
            if jump < C.REFINE_TOL:
                break
        self.goto_world_xy(xy[0], xy[1], lift=lift_view)
        return xy, cls

    # ------------------------------------------------------------------ pick & place
    def pick(self, item, xy, surface_z):
        """Turun ke barang di (x, y) dunia yang alasnya di surface_z, lalu pegang (DetachableJoint)."""
        item_center_z = surface_z + C.ITEM / 2
        self.goto_world_xy(*xy, lift=self.carry_lift(surface_z))
        g = self.gripper_world_xy()
        if g is not None:
            err = math.hypot(g[0] - xy[0], g[1] - xy[1])
            self.log.info(f"[PICK] {item}: galat horizontal gripper-target {err:.3f} m")
            if err > C.GRAB_TOL:
                self.goto_world_xy(*xy)
        self.move(lift=self.lift_for_world_z(item_center_z + C.HANG + 0.02), timeout=8.0)
        boat = self.poses.get("wamv")
        it = self.poses.get(f"item_{item}")
        g = self.gripper_world_xy()
        if it is not None and g is not None:
            d = math.hypot(it[0] - g[0], it[1] - g[1])
            self.log.info(f"[PICK] {item}: jarak gripper ke barang (cek Gazebo) {d:.3f} m")
            if d > C.GRAB_TOL:
                self.log.error(f"[PICK] {item} terlalu jauh dari gripper ({d:.2f} m), batal grab")
                self.move(lift=self.carry_lift(surface_z))
                return False
        ok = self.arm.grab(item, offset=self._grab_offset())
        # Nilai balik service 'create' tidak bisa dipercaya (bisa timeout padahal barang sudah terpasang),
        # jadi keberhasilan ditentukan dari pose: barang harus ikut naik bersama gripper.
        attached = self.is_attached(item, timeout=4.0)
        if not attached:
            self.log.error(f"[PICK] {item}: grab gagal (service={ok}), detach untuk keamanan")
            self.arm.detach(item)
            self.move(lift=self.carry_lift(surface_z))
            return False
        self.holding = item
        self.move(lift=self.carry_lift(surface_z))
        if not self.is_attached(item, timeout=1.0):
            self.log.error(f"[PICK] {item} terlepas saat diangkat")
            self.holding = None
            return False
        _ = boat
        return True

    def is_attached(self, item, timeout=2.0, tol=0.6):
        """True bila pusat barang berada ~HANG di bawah gripper (dicek dari pose Gazebo)."""
        t_end = time.time() + timeout
        while True:
            it = self.poses.get(f"item_{item}")
            try:
                g = self.arm.gripper_world()
            except Exception:
                g = None
            if it is not None and g is not None:
                d = math.dist((it[0], it[1], it[2] + C.HANG), g)
                if d < tol:
                    return True
            if time.time() > t_end:
                return False
            time.sleep(0.1)

    def _grab_offset(self):
        """Offset barang dari gripper di kerangka dunia (lurus ke bawah)."""
        return (0.0, 0.0, -C.HANG)

    def place(self, item, xy, surface_z):
        """Bawa barang yang dipegang ke (x, y) dunia, turunkan sampai alas PLACE_DROP_DZ di atas surface_z, lepas."""
        self.goto_world_xy(*xy, lift=self.carry_lift(surface_z))
        # bila barang sudah menyentuh dasar/dinding, sendi tidak akan sampai target: cukup tunggu sebentar
        self.move(lift=self.lift_for_world_z(surface_z + C.PLACE_DROP_DZ + C.ITEM / 2 + C.HANG), timeout=6.0)
        self.arm.detach(item)
        time.sleep(1.2)
        self.move(lift=self.carry_lift(surface_z))
        if self.is_attached(item, timeout=0.5):
            self.log.error(f"[PLACE] {item} masih menempel di gripper setelah detach, ulangi detach")
            self.arm.detach(item)
            time.sleep(1.0)
        self.holding = None
        it = self.poses.get(f"item_{item}")
        if it is not None:
            err = math.hypot(it[0] - xy[0], it[1] - xy[1])
            self.log.info(f"[PLACE] {item}: mendarat ({it[0]:.2f},{it[1]:.2f}) z={it[2]:.2f}, galat {err:.3f} m")
            return err < 0.5 and it[2] > surface_z - 0.3
        return True

    def container_world(self, item):
        boat = self.poses.get("wamv")
        return boat_to_world(boat, C.CONTAINER_XY[item])

    def deck_z(self):
        w = self.poses.get("wamv")
        return (w[2] if w else 0.0) + C.DECK_TOP_Z

    def stow(self):
        self.move(lift=0.0)
        if self.q["slide"] > 1.5:
            self.move(slide=1.0)
        self.move(yaw=C.ARM_STOW[0])
        self.move(slide=C.ARM_STOW[1])
