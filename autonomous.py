#!/usr/bin/env python3
import math
from collections import deque
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from ultralytics import YOLO

# ===================== KONFIGURASI UTAMA =====================
MODEL_PATH = '/home/alpro/fp3d_ws/src/arena-vrx/dataset/model/frontnav.pt'
HFOV = 1.047              # rad, fallback jika camera_info belum masuk
DOCK_PX_OFFSET = 30.0     # px, offset titik bidik akhir ke banner/slot

# Kecepatan & Kontrol Kemudi (halus untuk arena 18m & gate 1.5m - 2.0m)
BASE_SPEED = 60.0         # thrust maju normal
MIN_SPEED = 38.0          # thrust maju saat belok tajam
KP_STEER = 135.0          # gain belok proporsional terhadap sudut target (rad)
MAX_TURN = 85.0           # batas maksimum diferensial thrust kiri-kanan

# Margin sudut aman (dalam radian) diukur dari lebar layar
BUOY_MARGIN_RAD = 0.14    # ~8 derajat aman dari buoy hijau/merah
OBS_MARGIN_RAD = 0.22     # ~12.5 derajat aman dari obstacle kuning/hitam
OBS_ACTIVE_Y = 0.38       # mulai hindari obstacle saat y2/h > 0.38 (sudah masuk arena dekat)
OBS_PASS_Y = 0.88         # obstacle di bawah y2/h > 0.88 sudah di samping lambung

# Deteksi stuck / nabrak
STUCK_WIN = 2.0
STUCK_PX = 2.5
STUCK_GRACE = 12.0


class ASV(Node):
    def __init__(self):
        super().__init__('asv_vision_only')
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH)
        self.get_logger().info(f"Classes: {self.model.names}")

        self.declare_parameter('camera_topic',
                               '/wamv/sensors/cameras/front_left_camera_sensor/image_raw')
        self.declare_parameter('cam_angle_offset', -0.06)  # kompensasi posisi kamera front_left
        self.cam_offset = float(self.get_parameter('cam_angle_offset').value)

        cam_topic = self.get_parameter('camera_topic').value
        self.create_subscription(Image, cam_topic, self.cam_cb, 10)
        self.create_subscription(CameraInfo, cam_topic.replace('image_raw', 'camera_info'),
                                 self.info_cb, 10)

        self.pub_l = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.pub_r = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)

        self.fx = None
        self.last_cmd = (0.0, 0.0)
        self.t0 = self.now()
        self.last_log = 0.0
        self.ring = deque(maxlen=24)

        self.halted = False
        self.docked = False
        self.brake = 0

        # Memori arah hindar untuk obstacle yang sedang aktif (mencegah ragu-ragu kanan/kiri)
        self.locked_avoid_dir = None   # 'R' atau 'L'
        self.obs_lost_frames = 0

        # Memori dinding terakhir jika kamera sempat kosong sesaat
        self.last_seen_t = self.now()
        self.last_target_ang = 0.0

        # Stuck detector
        self.sig_ref = None
        self.sig_t = 0.0
        self.motion = 0.0
        self.still = 0

        self.get_logger().info("=== asv_vision_only v10-ANGULAR-CORRIDOR STARTED ===")

    # ------------------------------------------------------------------
    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def info_cb(self, m):
        if self.fx is None and m.k[0] > 1.0:
            self.fx = float(m.k[0])
            self.get_logger().info(f"CameraInfo fx={self.fx:.1f} ({m.width}x{m.height})")

    def set_thrust(self, l, r):
        a, b = Float64(), Float64()
        a.data = float(max(-200.0, min(250.0, l)))
        b.data = float(max(-200.0, min(250.0, r)))
        self.pub_l.publish(a)
        self.pub_r.publish(b)
        self.last_cmd = (a.data, b.data)

    def halt(self):
        self.halted = True
        self.set_thrust(0.0, 0.0)
        lines = list(self.ring)[-12:]
        for ln in ["########## HALTED: NABRAK / NYANGKUT ##########"] + lines:
            self.get_logger().info(ln)

    def check_stuck(self, sig, now):
        if self.sig_ref is not None and now - self.sig_t < STUCK_WIN:
            return
        if self.sig_ref is not None:
            ds = []
            for n, cx, y2 in self.sig_ref:
                best = None
                for n2, cx2, y22 in sig:
                    if n2 == n:
                        d = math.hypot(cx2 - cx, y22 - y2)
                        if d < 60 and (best is None or d < best):
                            best = d
                if best is not None:
                    ds.append(best)
            if len(ds) >= 2:
                self.motion = sum(ds) / len(ds)
                cmd = (self.last_cmd[0] + self.last_cmd[1]) / 2.0
                if (cmd > 45 and now - self.t0 > STUCK_GRACE and self.motion < STUCK_PX
                        and not self.docked and self.brake == 0):
                    self.still += 1
                else:
                    self.still = 0
                if self.still >= 2:
                    self.halt()
            else:
                self.still = 0
        self.sig_ref, self.sig_t = list(sig), now

    def refine_class(self, frame, x1, y1, x2, y2, name):
        if 'red' not in name:
            return name
        bw, bh = x2 - x1, y2 - y1
        roi = frame[int(y1 + 0.3 * bh):int(y2 - 0.3 * bh), int(x1 + 0.3 * bw):int(x2 - 0.3 * bw)]
        if roi.size == 0:
            return name
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        if np.mean(hsv[:, :, 2]) < 115 and np.mean(hsv[:, :, 1]) < 95:
            return 'black'
        return name

    def show(self, img, text, dbg='', col=(0, 255, 255)):
        t = self.now()
        if t - self.last_log > 0.5:
            self.last_log = t
            line = (f"[t={t - self.t0:5.1f}s] {text} | {dbg} | "
                    f"L={self.last_cmd[0]:.0f} R={self.last_cmd[1]:.0f}")
            self.ring.append(line)
            if not self.halted:
                self.get_logger().info(line)
        cv2.putText(img, text, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
        cv2.imshow('ASV', img)
        cv2.waitKey(1)

    # ------------------------------------------------------------------
    def cam_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        fa = float(w * h)
        F = self.fx if self.fx else w / (2.0 * math.tan(HFOV / 2.0))
        ann = frame.copy()
        now = self.now()

        if self.halted:
            self.set_thrust(0.0, 0.0)
            self.show(ann, "HALTED", col=(0, 0, 255))
            return

        res = self.model(frame, conf=0.30, verbose=False)[0]

        # Setiap deteksi disimpan sebagai: (angle_rad, y2_norm, area_norm, label)
        # angle_rad: NEGATIF = KIRI kapal, POSITIF = KANAN kapal
        greens = []
        reds = []
        obsts = []
        sig = []
        banner_x, banner_a = None, 0.0
        dock_x, dock_a = None, 0.0

        cols = {'green': (0, 255, 0), 'red': (0, 0, 255),
                'yellow': (0, 255, 255), 'black': (255, 255, 255)}

        for b in res.boxes:
            raw = self.model.names[int(b.cls[0])]
            conf = float(b.conf[0])
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            name = self.refine_class(frame, x1, y1, x2, y2, raw)
            if 'black' not in name and conf < 0.40:
                continue

            cx = (x1 + x2) / 2.0
            area = (x2 - x1) * (y2 - y1)
            area_n = area / fa
            y2_n = y2 / float(h)
            sig.append((name, cx, y2))

            # Sudut horizontal relatif terhadap sumbu tengah kapal (+ kanan, - kiri)
            ang = math.atan2(cx - w / 2.0, F) + self.cam_offset

            col = next((k for k in cols if k in name), None)
            bc = cols.get(col, (255, 180, 0))
            cv2.rectangle(ann, (int(x1), int(y1)), (int(x2), int(y2)), bc, 2)
            cv2.putText(ann, f"{name} {math.degrees(ang):+.0f}d y={y2_n:.2f}",
                        (int(x1), max(15, int(y1) - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, bc, 2)

            if col is None:
                if 'banner' in name and area > banner_a:
                    banner_a, banner_x = area, cx
                elif ('dock' in name or 'slot' in name) and area > dock_a:
                    dock_a, dock_x = area, cx
                continue

            if area_n < 0.00025:
                continue

            if col == 'green':
                greens.append((ang, y2_n, area_n))
            elif col == 'red':
                reds.append((ang, y2_n, area_n))
            else:
                obsts.append((ang, y2_n, area_n, col))

        self.check_stuck(sig, now)
        if self.halted:
            self.show(ann, "HALTED", col=(0, 0, 255))
            return

       # Kunci agar parkir HANYA aktif setelah semua buoy/obstacle benar-benar lewat > 1.0 detik
        if greens or reds or obsts:
            self._saw_buoy = True
            self._clear_t = None
        elif getattr(self, '_saw_buoy', False) and getattr(self, '_clear_t', None) is None:
            self._clear_t = now

        ready_to_dock = (getattr(self, '_clear_t', None) is not None) and ((now - self._clear_t) > 1.0)
        b_ratio = banner_a / fa
        d_ratio = dock_a / fa

        # ================= 1. FASE PARKIR / REM DI SPOT 2 =================
        if self.docked:
            # Tahan lurus di tengah ceruk U melawan arus ringan
            if banner_x is not None:
                err_x = (w / 2.0 + DOCK_PX_OFFSET) - banner_x
                turn = 0.0 if abs(err_x) < 8.0 else max(-25.0, min(25.0, 0.25 * err_x))
                err_dist = 0.034 - b_ratio
                hold_fwd = 0.0 if abs(err_dist) < 0.002 else max(-35.0, min(25.0, err_dist * 1500.0))
                self.set_thrust(hold_fwd - turn, hold_fwd + turn)
            else:
                self.set_thrust(0.0, 0.0)
            self.show(ann, f"DOCKED AT SPOT 2 (b={b_ratio:.4f})", col=(0, 255, 0))
            return

        if self.brake > 0 or (not greens and not reds and not obsts and (b_ratio > 0.036 or d_ratio > 0.16)):
            if self.brake < 16:
                self.set_thrust(-140.0, -140.0)
                self.brake += 1
            else:
                self.set_thrust(0.0, 0.0)
                self.docked = True
            self.show(ann, f"BRAKING {self.brake}/16 (b={b_ratio:.4f})", col=(0, 0, 255))
            return

        # ================= 2. FASE MENUJU BANNER/SLOT (JIKA SUDAH DI UJUNG) =================
        if not greens and not reds and not obsts and (banner_x is not None or dock_x is not None):
            aim_px = banner_x if banner_x is not None else dock_x
            err = (w / 2.0 + DOCK_PX_OFFSET) - aim_px
            turn = max(-55.0, min(55.0, 0.35 * err))
            fwd = max(24.0, 42.0 * (1.0 - min(0.55, b_ratio / 0.036)))
            if abs(err) > 40.0:
                fwd = 22.0
            self.set_thrust(fwd - turn, fwd + turn)
            self.show(ann, f"DOCKING APPROACH (b={b_ratio:.4f})", col=(255, 200, 0))
            return
        # ================= 3. KORIDOR NAVIGASI (GREEN KIRI, RED KANAN) =================
        # Pilih buoy terdekat (y2_n paling besar = paling bawah di layar = paling dekat kapal)
        near_g = max(greens, key=lambda item: item[1]) if greens else None
        near_r = max(reds, key=lambda item: item[1]) if reds else None

        left_bound = (near_g[0] + BUOY_MARGIN_RAD) if near_g else -0.42
        right_bound = (near_r[0] - BUOY_MARGIN_RAD) if near_r else +0.42

        # Pastikan batas kiri tidak menyilang batas kanan
        if left_bound > right_bound:
            mid_b = (left_bound + right_bound) / 2.0
            left_bound = mid_b - 0.05
            right_bound = mid_b + 0.05

        corridor_mid = (left_bound + right_bound) / 2.0

        # ================= 4. PENGHINDARAN OBSTACLE (KUNING & HITAM) =================
        # Ambil obstacle terdekat yang berada dalam zona aktif (belum lewat di bawah lambung)
        active_obs = [o for o in obsts if OBS_ACTIVE_Y <= o[1] <= OBS_PASS_Y]
        cur_obs = max(active_obs, key=lambda item: item[1]) if active_obs else None

        mode = "CORRIDOR"
        target_ang = corridor_mid

        if cur_obs is not None:
            self.obs_lost_frames = 0
            obs_ang, obs_y, _, obs_col = cur_obs

            # Tentukan sisi lewat sekali saja saat obstacle mulai didekati:
            # Cek sisi mana yang punya celah sudut lebih lebar di dalam koridor (Green..Red)
            if self.locked_avoid_dir is None:
                space_right = right_bound - obs_ang
                space_left = obs_ang - left_bound
                self.locked_avoid_dir = 'R' if space_right >= space_left else 'L'

            if self.locked_avoid_dir == 'R':
                # Lewat KANAN obstacle (di antara obstacle dan Red buoy)
                safe_left = obs_ang + OBS_MARGIN_RAD
                target_ang = max(safe_left, (safe_left + right_bound) / 2.0)
                target_ang = min(target_ang, right_bound + 0.08)
                mode = f"AVOID {obs_col.upper()} -> RIGHT"
            else:
                # Lewat KIRI obstacle (di antara Green buoy dan obstacle)
                safe_right = obs_ang - OBS_MARGIN_RAD
                target_ang = min(safe_right, (left_bound + safe_right) / 2.0)
                target_ang = max(target_ang, left_bound - 0.08)
                mode = f"AVOID {obs_col.upper()} -> LEFT"
        else:
            # Reset kunci arah setelah obstacle benar-benar hilang beberapa frame
            if self.locked_avoid_dir is not None:
                self.obs_lost_frames += 1
                if self.obs_lost_frames >= 5:
                    self.locked_avoid_dir = None
                    self.obs_lost_frames = 0

        # ================= 5. KENDALI BLIND SESUAT / HALUS =================
        if greens or reds or obsts:
            self.last_seen_t = now
            self.last_target_ang = target_ang
        elif now - self.last_seen_t < 1.5:
            # Pertahankan arah terakhir secara lembut jika kedip < 1.5 detik
            target_ang = self.last_target_ang * 0.7
            mode = "HOLD LAST"
        else:
            target_ang = 0.0
            mode = "SEARCH STRAIGHT"

        # Gambar garis vertikal target sudut di layar OpenCV untuk visualisasi debug
        tx = int(w / 2.0 + F * math.tan(target_ang - self.cam_offset))
        tx = max(0, min(w - 1, tx))
        cv2.line(ann, (tx, int(h * 0.4)), (tx, h), (255, 255, 0), 3)
        cv2.line(ann, (int(w / 2), int(h * 0.7)), (int(w / 2), h), (200, 200, 200), 1)

        # ================= 6. MIXING THRUSTER KIRI & KANAN =================
        # target_ang > 0 artinya target di KANAN -> thruster KIRI harus lebih besar dari KANAN
        turn = KP_STEER * target_ang
        turn = max(-MAX_TURN, min(MAX_TURN, turn))

        # Pelankan laju maju secara proporsional saat sudut belok besar
        speed_factor = 1.0 - min(0.45, abs(target_ang) / 0.65)
        fwd = MIN_SPEED + (BASE_SPEED - MIN_SPEED) * speed_factor
        if cur_obs is not None and cur_obs[1] > 0.60:
            fwd = min(fwd, 45.0)

        self.set_thrust(fwd + turn, fwd - turn)

        dbg = (f"G={len(greens)} R={len(reds)} O={len(obsts)} | "
               f"ang={math.degrees(target_ang):+.1f}deg")
        self.show(ann, mode, dbg)


def main():
    rclpy.init()
    node = ASV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.set_thrust(0.0, 0.0)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()