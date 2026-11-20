"""Persepsi: model YOLO yang sudah ada (dataset/model) + geometri kamera.

  FrontPerception   kamera depan WAM-V + frontnav.pt -> landmark buoy / rintangan / banner di kerangka dunia
  ArmPerception     kamera gripper + arm.pt          -> posisi dunia barang & zona taruh (sinar piksel x bidang z)
  OverviewLocator   kamera atas arena (HSV)          -> lokasi kasar barang (kuning) & zona taruh (putih) di dermaga
"""
import math
import threading
import time

import cv2
import numpy as np

from . import config as C
from .geometry import (PinholeCam, localize_by_range, pose_T, ray_plane_z, sphere_range, world_to_boat,
                       xyz_rpy_T, yaw_of)

FRONT_COLORS = {"green": (0, 200, 0), "red": (0, 0, 255), "yellow": (0, 220, 255), "black": (60, 60, 60),
                "target_banner": (255, 0, 255), "dock": (255, 200, 0)}


def refine_red_black(frame, box, name):
    """Model kadang tertukar merah <-> hitam (warna gelap). Cek saturasi/kecerahan pusat bbox."""
    if name not in ("red", "black"):
        return name
    x1, y1, x2, y2 = [int(v) for v in box]
    bw, bh = x2 - x1, y2 - y1
    roi = frame[y1 + bh // 3:y2 - bh // 3, x1 + bw // 3:x2 - bw // 3]
    if roi.size == 0:
        return name
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    s, v = float(np.mean(hsv[:, :, 1])), float(np.mean(hsv[:, :, 2]))
    if v < 70 and s < 110:
        return "black"
    if s > 120 and v > 80:
        hue = float(np.median(hsv[:, :, 0]))
        if hue < 12 or hue > 165:
            return "red"
    return name


class FrontPerception:
    """Deteksi kamera depan -> posisi dunia -> LandmarkMap."""

    def __init__(self, model, lmap, logger):
        self.model, self.map, self.log = model, lmap, logger
        self.cam = None
        self.lock = threading.Lock()
        self.last_ann = None
        self.last_ann_t = 0.0
        self.T_boat_cam = xyz_rpy_T(C.FRONT_CAM_XYZ, (0.0, C.FRONT_CAM_PITCH, 0.0))
        self.banner_bearing = None      # sudut banner relatif haluan (rad, + = kiri), untuk aim water cannon
        self.banner_t = -1e9
        self.banner_rays = []           # (cam_x, cam_y, bearing_dunia): posisi lateral banner TANPA perlu jarak

    def set_intrinsics(self, w, h, fx=None):
        if self.cam is None or self.cam.w != w:
            self.cam = PinholeCam(w, h, hfov=C.FRONT_CAM_HFOV, fx=fx)

    def process(self, frame, boat_pose, t):
        """frame BGR, boat_pose = pose WAM-V di dunia pada waktu gambar diambil."""
        h, w = frame.shape[:2]
        self.set_intrinsics(w, h)
        res = self.model(frame, conf=C.FRONT_CONF, verbose=False)[0]
        T_wc = pose_T(boat_pose) @ self.T_boat_cam
        ann = frame.copy()
        seen = set()
        n = 0
        for b in res.boxes:
            name = self.model.names[int(b.cls[0])]
            box = b.xyxy[0].tolist()
            name = refine_red_black(frame, box, name)
            x1, y1, x2, y2 = box
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            bw = x2 - x1
            col = FRONT_COLORS.get(name, (255, 255, 255))
            cv2.rectangle(ann, (int(x1), int(y1)), (int(x2), int(y2)), col, 2)
            touches = x1 < C.EDGE_MARGIN_PX or x2 > w - C.EDGE_MARGIN_PX
            if y2 > h - C.EDGE_MARGIN_PX:
                continue        # menyentuh tepi bawah: biasanya muatan kuning di dek kapal sendiri
            if name in ("green", "red", "yellow", "black"):
                rad = C.BUOY_R if name in ("green", "red") else C.OBS_R
                if touches or not (C.BBOX_ASPECT[0] <= bw / max(y2 - y1, 1.0) <= C.BBOX_ASPECT[1]):
                    continue
                rng = sphere_range(self.cam, bw, rad)
                rng = None if rng is None else rng * C.RANGE_SCALE.get(name, 1.0)
            elif name == "target_banner":
                # jarak dari lebar bbox tidak dipakai untuk posisi (label bisa papan 5 m atau segitiga 3 m);
                # cukup ARAH: sinar banner dipotongkan dengan garis dinding Spot 2 di misi
                ray = self.T_boat_cam[:3, :3] @ self.cam.ray(u, v)
                self.banner_bearing = math.atan2(ray[1], ray[0])
                self.banner_t = t
                rw = T_wc[:3, :3] @ self.cam.ray(u, v)
                self.banner_rays.append((float(T_wc[0, 3]), float(T_wc[1, 3]), math.atan2(rw[1], rw[0])))
                self.banner_rays = self.banner_rays[-60:]
                cv2.putText(ann, f"banner {math.degrees(self.banner_bearing):+.1f}deg", (int(x1), max(14, int(y1) - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
                continue
            else:
                continue
            if rng is None or not (C.MAP_MIN_RANGE < rng < C.MAP_MAX_RANGE):
                continue
            p = localize_by_range(T_wc, self.cam, u, v, rng)
            bx, by = world_to_boat(boat_pose, p)
            if bx < C.BOAT_LEN / 2 + 1.5 or abs(p[2]) > 2.5:
                continue        # di/dekat badan kapal atau melayang: bukan buoy
            lm = self.map.add(name, float(p[0]), float(p[1]), rng, t)
            seen.add(lm.id)
            n += 1
            cv2.putText(ann, f"{name} {rng:.1f}m #{lm.id}", (int(x1), max(14, int(y1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
        cam_yaw = yaw_of(boat_pose)
        self.map.negative(T_wc[0, 3], T_wc[1, 3], cam_yaw, C.FRONT_CAM_HFOV, seen)
        with self.lock:
            self.last_ann = ann
            self.last_ann_t = time.time()
        return n


class ArmPerception:
    """Kamera gripper: deteksi barang (cube/sphere/prism) dan zona taruh, diproyeksikan ke bidang z dunia."""

    def __init__(self, model, logger):
        self.model, self.log = model, logger
        self.cam = PinholeCam(*C.ARM_CAM_WH, hfov=C.ARM_CAM_HFOV)
        self.T_grip_cam = xyz_rpy_T(C.ARM_CAM_XYZ, (0.0, C.ARM_CAM_PITCH, 0.0))
        self.lock = threading.Lock()
        self.frame = None
        self.frame_pose = None     # pose kamera dunia saat frame diterima
        self.last_ann = None
        self.last_ann_t = 0.0

    def set_frame(self, frame, T_world_cam):
        with self.lock:
            self.frame, self.frame_pose = frame, T_world_cam

    def cam_pose(self, arm_model_pose, gripper_rel_pose):
        """Pose kamera di dunia = pose model lengan * pose link gripper (relatif model) * pose sensor."""
        return pose_T(arm_model_pose) @ pose_T(gripper_rel_pose) @ self.T_grip_cam

    def detect(self, want, z_plane):
        """Deteksi kelas `want` ('cube'|'sphere'|'prism'|'mark'|'any_item') pada frame terbaru.
        -> list (cls, conf, (x, y) dunia, (u, v)) diurutkan dari yang paling dekat pusat gambar."""
        with self.lock:
            frame, T = self.frame, self.frame_pose
        if frame is None or T is None:
            return []
        ann = frame.copy()
        out = []
        if want == "mark":
            out = self._white_marks(frame, T, z_plane, ann)
        res = self.model(frame, conf=C.ARM_CONF, verbose=False)[0]
        for b in res.boxes:
            name = self.model.names[int(b.cls[0])]
            conf = float(b.conf[0])
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            cv2.rectangle(ann, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(ann, f"{name} {conf:.2f}", (int(x1), max(14, int(y1) - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
            cls = "mark" if name == "dock" else name
            if cls in C.ITEMS:
                shape = self._shape(frame, (x1, y1, x2, y2))
                # pengecek kedua untuk YOLO: dari atas, bola = lingkaran; kubus/prisma = poligon bersudut
                if shape == "round" and cls != "sphere":
                    self.log.info(f"[ARM-VIS] YOLO={cls} tapi kontur bundar -> sphere")
                    cls = "sphere"
                elif shape == "poly" and cls == "sphere":
                    self.log.info("[ARM-VIS] YOLO=sphere tapi kontur bersudut -> bukan bola")
                    cls = "poly"
                cv2.putText(ann, shape or "?", (int(x1), int(y2) + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            if want == "any_item" and cls not in C.ITEMS + ("poly",):
                continue
            if want not in ("any_item",) and cls != want:
                continue
            p = ray_plane_z(T, self.cam, u, v, z_plane)
            if p is not None:
                out.append((cls, conf, (float(p[0]), float(p[1])), (u, v)))
        cx, cy = self.cam.cx, self.cam.cy
        out.sort(key=lambda d: (d[3][0] - cx) ** 2 + (d[3][1] - cy) ** 2)
        with self.lock:
            self.last_ann = ann
            self.last_ann_t = time.time()
        return out

    @staticmethod
    def _shape(frame, box):
        """'round' | 'poly' | None dari kontur kuning terbesar di dalam bbox (circularity + jumlah sudut)."""
        x1, y1, x2, y2 = [int(v) for v in box]
        pad = 6
        h, w = frame.shape[:2]
        if x1 < 2 or y1 < 2 or x2 > w - 2 or y2 > h - 2:
            return None                                   # terpotong tepi gambar
        roi = frame[max(0, y1 - pad):min(h, y2 + pad), max(0, x1 - pad):min(w, x2 + pad)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (12, 90, 60), (40, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not cnts:
            return None
        c = max(cnts, key=cv2.contourArea)
        a, per = cv2.contourArea(c), cv2.arcLength(c, True)
        if a < 300 or per <= 0:
            return None
        circ = 4 * math.pi * a / per ** 2
        hull = cv2.convexHull(c)
        n = len(cv2.approxPolyDP(hull, 0.04 * cv2.arcLength(hull, True), True))
        if circ > 0.83 and n >= 6:
            return "round"
        if n <= 5:
            return "poly"
        return None

    def _white_marks(self, frame, T, z_plane, ann):
        """Zona taruh = kotak putih di dermaga. HSV: saturasi rendah, kecerahan tinggi, bentuk kotak."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, 200), (180, 45, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cnts:
            a = cv2.contourArea(c)
            if a < 400:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if not (0.4 < w / max(h, 1) < 2.5) or a / float(w * h) < 0.6:
                continue
            m = cv2.moments(c)
            u, v = m["m10"] / m["m00"], m["m01"] / m["m00"]
            p = ray_plane_z(T, self.cam, u, v, z_plane)
            cv2.rectangle(ann, (x, y), (x + w, y + h), (255, 255, 255), 2)
            if p is not None:
                out.append(("mark", 0.9, (float(p[0]), float(p[1])), (u, v)))
        return out


class OverviewLocator:
    """Kamera atas statis (topik gz 'overview_camera/image', disediakan guidebook untuk Vision).

    Pose kamera dibaca dari SDF world (0 0 20*S, rpy 0 pi/2 pi/2), jadi piksel -> dunia cukup sinar x bidang.
    Dipakai untuk lokasi KASAR (+-0.15 m): kubus/bola/prisma kuning di dermaga Spot 1 dan kotak putih
    di dermaga Spot 2. Kelasnya ditentukan kemudian oleh kamera gripper (arm.pt).
    """

    def __init__(self, logger):
        self.log = logger
        self.cam = PinholeCam(1280, 720, hfov=1.2)
        self.T = xyz_rpy_T((0.0, 0.0, 20.0 * C.SCALE), (0.0, 1.5708, 1.5708))
        self.lock = threading.Lock()
        self.frame = None

    def set_frame(self, rgb):
        with self.lock:
            self.frame = rgb

    def has_frame(self):
        with self.lock:
            return self.frame is not None

    def _blobs(self, mask, z, min_px, max_px):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in cnts:
            a = cv2.contourArea(c)
            if not (min_px <= a <= max_px):
                continue
            x, y, w, h = cv2.boundingRect(c)
            if not (0.5 < w / max(h, 1) < 2.0):
                continue
            m = cv2.moments(c)
            if m["m00"] <= 0:
                continue
            p = ray_plane_z(self.T, self.cam, m["m10"] / m["m00"], m["m01"] / m["m00"], z)
            if p is not None:
                out.append((float(p[0]), float(p[1]), a))
        return out

    def items_near(self, center_xy, radius):
        """Blob kuning (barang) di permukaan dermaga dalam radius dari center_xy."""
        with self.lock:
            f = None if self.frame is None else self.frame.copy()
        if f is None:
            return []
        hsv = cv2.cvtColor(f, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, (15, 120, 120), (40, 255, 255))
        px_per_m = self.cam.fx / (20.0 * C.SCALE - C.DOCK_TOP_Z)
        area = (C.ITEM * px_per_m) ** 2
        blobs = self._blobs(mask, C.DOCK_TOP_Z + C.ITEM / 2, 0.25 * area, 3.0 * area)
        return [(x, y) for x, y, _ in blobs if math.hypot(x - center_xy[0], y - center_xy[1]) < radius]

    def marks_near(self, center_xy, radius):
        with self.lock:
            f = None if self.frame is None else self.frame.copy()
        if f is None:
            return []
        hsv = cv2.cvtColor(f, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv, (0, 0, 215), (180, 40, 255))
        px_per_m = self.cam.fx / (20.0 * C.SCALE - C.DOCK_TOP_Z)
        area = (1.6 * C.ITEM * px_per_m) ** 2
        blobs = self._blobs(mask, C.DOCK_TOP_Z, 0.35 * area, 2.5 * area)
        return [(x, y) for x, y, _ in blobs if math.hypot(x - center_xy[0], y - center_xy[1]) < radius]
