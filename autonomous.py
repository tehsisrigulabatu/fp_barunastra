#!/usr/bin/env python3
import math
import sys
import time
import threading
from collections import deque
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from ultralytics import YOLO

sys.path.insert(0, '/home/alpro/fp3d_ws/src/arena-vrx')
from lengan import Arm, wait_pose

# ===================== KONFIGURASI UTAMA (TUNED FOR LOADED CARGO) =====================
MODEL_PATH = '/home/alpro/fp3d_ws/src/arena-vrx/dataset/model/frontnav.pt'
ARM_MODEL_PATH = '/home/alpro/fp3d_ws/src/arena-vrx/dataset/model/arm.pt'
HFOV = 1.047              # rad, fallback jika camera_info belum masuk
DOCK_PX_OFFSET = 0.0      # px, lurus tepat ke tengah banner Spot 2

# Kecepatan & Kontrol Kemudi (lebih kalem tapi torsi belok lebih sigap karena bawa beban)
BASE_SPEED = 48.0         # thrust maju normal (diturunkan dari 60 agar sempat belok)
MIN_SPEED = 26.0          # thrust maju saat belok tajam / hindar obstacle
KP_STEER = 175.0          # gain belok lebih responsif melawan inersia muatan
MAX_TURN = 105.0          # torsi diferensial maksimum untuk belok tajam

# Margin sudut aman & deteksi dini ("tau diri" bawa muatan lebar)
BUOY_MARGIN_RAD = 0.19    # ~11 deg ruang aman dari buoy hijau/merah
OBS_MARGIN_RAD = 0.30     # ~17 deg ruang aman dari obstacle kuning/hitam
OBS_ACTIVE_Y = 0.25       # mulai menghindar JAUH lebih awal (sebelumnya 0.38)
OBS_PASS_Y = 0.90         # tetap waspada sampai obstacle benar-benar melewati lambung

# Deteksi stuck / nabrak
STUCK_WIN = 2.0
STUCK_PX = 2.5
STUCK_GRACE = 12.0


class ASV(Node):
    def __init__(self):
        super().__init__('asv_vision_only')
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH)
        self.arm_model = YOLO(ARM_MODEL_PATH)
        self.get_logger().info(f"Front Classes: {self.model.names}")
        self.get_logger().info(f"Arm Classes: {self.arm_model.names}")

        self.declare_parameter('camera_topic',
                               '/wamv/sensors/cameras/front_left_camera_sensor/image_raw')
        self.declare_parameter('cam_angle_offset', -0.06)
        self.cam_offset = float(self.get_parameter('cam_angle_offset').value)

        cam_topic = self.get_parameter('camera_topic').value
        self.create_subscription(Image, cam_topic, self.cam_cb, 10)
        self.create_subscription(CameraInfo, cam_topic.replace('image_raw', 'camera_info'),
                                 self.info_cb, 10)
        self.create_subscription(Image, '/arm/camera/image', self.arm_cam_cb, 1)

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

        # Memori arah hindar obstacle
        self.locked_avoid_dir = None
        self.obs_lost_frames = 0

        # Memori dinding terakhir
        self.last_seen_t = self.now()
        self.last_target_ang = 0.0

        # Stuck detector
        self.sig_ref = None
        self.sig_t = 0.0
        self.motion = 0.0
        self.still = 0

        # State lengan & misi: 'PICKUP' -> 'NAVIGATE' -> 'DROP' -> 'DONE'
        self.arm = Arm()
        self.arm_dets = {}
        self.arm_frame_size = (640, 480)
        self.arm_yaw = 0.0
        self.arm_slide = 0.9
        self.arm_lift = 0.0
        self.mission_phase = 'PICKUP'
        self.drop_started = False
        self.grabbed_items = []

        threading.Thread(target=self.pickup_routine, daemon=True).start()
        self.get_logger().info("=== asv_vision_only LOADED-TUNED MISSION STARTED ===")

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
                if (cmd > 35 and now - self.t0 > STUCK_GRACE and self.motion < STUCK_PX
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

    # ===================== KONTROL LENGAN ULTRA-HALUS (SUDAH WORK!) =====================
    def arm_cam_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape
        self.arm_frame_size = (w, h)
        ann = frame.copy()

        res = self.arm_model(frame, conf=0.25, verbose=False)[0]
        dets = {}
        for b in res.boxes:
            raw_name = self.arm_model.names[int(b.cls[0])].lower()
            conf = float(b.conf[0])
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

            key = None
            if 'cube' in raw_name or 'kubus' in raw_name or 'box' in raw_name:
                key = 'cube'
            elif 'sphere' in raw_name or 'bola' in raw_name or 'ball' in raw_name:
                key = 'sphere'
            elif 'prism' in raw_name or 'cone' in raw_name or 'prisma' in raw_name:
                key = 'prism'
            elif 'slot' in raw_name or 'dock' in raw_name or 'white' in raw_name:
                key = 'slot'

            if key:
                dets[key] = (cx, cy)
            cv2.rectangle(ann, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(ann, f"{raw_name} {conf:.2f}", (int(x1), max(15, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

        self.arm_dets = dets
        cv2.circle(ann, (w // 2, h // 2), 5, (0, 0, 255), -1)
        cv2.putText(ann, f"PHASE: {self.mission_phase} | GRABBED: {len(self.grabbed_items)}/3",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 2)
        cv2.imshow('ARM_CAM', ann)
        cv2.waitKey(1)

    def move_arm_smooth(self, yaw=None, slide=None, lift=None, settle_s=0.4):
        target_yaw = self.arm_yaw if yaw is None else max(-3.14, min(3.14, yaw))
        target_slide = self.arm_slide if slide is None else max(0.0, min(7.0, slide))
        target_lift = self.arm_lift if lift is None else max(-1.00, min(0.0, lift))

        steps = max(
            1,
            int(abs(target_yaw - self.arm_yaw) / 0.035),
            int(abs(target_slide - self.arm_slide) / 0.10),
            int(abs(target_lift - self.arm_lift) / 0.06),
        )

        start_yaw, start_slide, start_lift = self.arm_yaw, self.arm_slide, self.arm_lift
        for i in range(1, steps + 1):
            alpha = i / float(steps)
            self.arm_yaw = start_yaw + (target_yaw - start_yaw) * alpha
            self.arm_slide = start_slide + (target_slide - start_slide) * alpha
            self.arm_lift = start_lift + (target_lift - start_lift) * alpha
            self.arm.cmd(yaw=self.arm_yaw, slide=self.arm_slide, lift=self.arm_lift)
            time.sleep(0.05)

        if settle_s > 0:
            time.sleep(settle_s)

    def pickup_routine(self):
        wait_pose(self.arm, "wamv", t=15)
        wait_pose(self.arm, "gripper", t=15)
        self.get_logger().info("=== MEMULAI PICKUP ULTRA-HALUS ===")

        grab_offsets = {
            'cube':   (-0.30, 0.0, -0.35),
            'sphere': ( 0.00, 0.0, -0.35),
            'prism':  ( 0.30, 0.0, -0.35),
        }

        pickup_plan = [
            ('sphere', [(1.57, 2.8), (1.57, 3.4), (1.57, 4.0)]),
            ('cube',   [(3.14, 3.8), (3.14, 4.5), (3.14, 5.1)]),
            ('prism',  [(-1.57, 2.8), (-1.57, 3.4), (-1.57, 4.0)]),
        ]

        for item, search_poses in pickup_plan:
            self.get_logger().info(f"[PICKUP] Mencari {item}...")
            found = False
            for sy, ss in search_poses:
                self.move_arm_smooth(slide=0.9, lift=0.0, settle_s=0.2)
                self.move_arm_smooth(yaw=sy, lift=0.0, settle_s=0.3)
                self.move_arm_smooth(slide=ss, lift=0.0, settle_s=0.5)
                if item in self.arm_dets:
                    found = True
                    break

            if found:
                t_start = time.time()
                while time.time() - t_start < 10.0:
                    det = self.arm_dets.get(item)
                    if det is None:
                        time.sleep(0.1)
                        continue
                    w, h = self.arm_frame_size
                    err_x = (w / 2.0) - det[0]
                    err_y = (h / 2.0) - det[1]

                    if abs(err_x) < 15.0 and abs(err_y) < 15.0:
                        break

                    dyaw = max(-0.04, min(0.04, err_x * 0.0010))
                    dslide = max(-0.08, min(0.08, err_y * 0.0020))
                    self.move_arm_smooth(yaw=self.arm_yaw + dyaw, slide=self.arm_slide + dslide, settle_s=0.06)

            grabbed = False
            for target_lift in [-0.35, -0.55, -0.75, -0.95]:
                self.move_arm_smooth(lift=target_lift, settle_s=0.35)
                if self.arm.grab(item, offset=grab_offsets[item]):
                    grabbed = True
                    self.grabbed_items.append(item)
                    self.get_logger().info(f"[PICKUP] BERHASIL GRAB {item} di lift={target_lift}!")
                    break

            if not grabbed:
                self.get_logger().error(f"[PICKUP] Gagal grab {item}.")

            self.move_arm_smooth(lift=0.0, settle_s=0.35)
            self.move_arm_smooth(slide=0.9, lift=0.0, settle_s=0.3)

        self.move_arm_smooth(yaw=0.0, slide=0.9, lift=0.0, settle_s=0.6)

        if len(self.grabbed_items) == 0:
            self.get_logger().error("=== TIDAK ADA BARANG TERANGKUT! KAPAL DITAHAN DI SPOT 1 ===")
            return

        self.t0 = self.now()
        self.mission_phase = 'NAVIGATE'
        self.get_logger().info(f"=== {len(self.grabbed_items)} BARANG SIAP ({self.grabbed_items}) -> BERANGKAT KE SPOT 2 ===")

    def drop_routine(self):
        self.get_logger().info("=== MEMULAI FASE DROP DI KOTAK PUTIH SPOT 2 ===")
        self.move_arm_smooth(yaw=0.0, slide=3.1, lift=0.0, settle_s=0.6)
        self.move_arm_smooth(lift=-0.50, settle_s=0.5)

        t_start = time.time()
        while time.time() - t_start < 5.0:
            det = self.arm_dets.get('slot')
            if det is None:
                break
            w, h = self.arm_frame_size
            err_x = (w / 2.0) - det[0]
            err_y = (h / 2.0) - det[1]
            if abs(err_x) < 18.0 and abs(err_y) < 18.0:
                break
            self.move_arm_smooth(
                yaw=self.arm_yaw + max(-0.04, min(0.04, err_x * 0.001)),
                slide=self.arm_slide + max(-0.08, min(0.08, err_y * 0.002)),
                settle_s=0.06
            )

        drop_yaw_offsets = {
            'cube':   -0.10,
            'sphere':  0.00,
            'prism':  +0.10,
        }
        base_yaw = self.arm_yaw
        for item in self.grabbed_items:
            self.move_arm_smooth(yaw=base_yaw + drop_yaw_offsets.get(item, 0.0), settle_s=0.4)
            self.arm.detach(item)
            self.get_logger().info(f"[DROP] Detach {item} tepat di kotak putih Spot 2")
            time.sleep(0.4)

        self.move_arm_smooth(lift=0.0, settle_s=0.4)
        self.move_arm_smooth(yaw=0.0, slide=0.9, lift=0.0, settle_s=0.6)
        self.mission_phase = 'DONE'
        self.get_logger().info("=== MISI SELESAI 100% ===")

    # ===================== NAVIGASI BERMUATAN & PARKIR SPOT 2 =====================
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

        if self.mission_phase == 'PICKUP':
            self.set_thrust(0.0, 0.0)
            self.show(ann, f"PICKUP ({len(self.grabbed_items)}/3)", col=(255, 255, 0))
            return

        res = self.model(frame, conf=0.30, verbose=False)[0]

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

            if area_n < 0.00020:
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
            if not self.drop_started:
                self.drop_started = True
                self.mission_phase = 'DROP'
                threading.Thread(target=self.drop_routine, daemon=True).start()

            if banner_x is not None:
                err_x = (w / 2.0 + DOCK_PX_OFFSET) - banner_x
                turn = 0.0 if abs(err_x) < 10.0 else max(-18.0, min(18.0, 0.15 * err_x))
                err_dist = 0.024 - b_ratio
                hold_fwd = 0.0 if abs(err_dist) < 0.002 else max(-30.0, min(20.0, err_dist * 1200.0))
                self.set_thrust(hold_fwd - turn, hold_fwd + turn)
            else:
                self.set_thrust(0.0, 0.0)
            self.show(ann, f"DOCKED ({self.mission_phase}) b={b_ratio:.4f}", col=(0, 255, 0))
            return

        if self.brake > 0 or (ready_to_dock and not greens and not reds and not obsts and (b_ratio > 0.024 or d_ratio > 0.11)):
            if self.brake < 16:
                self.set_thrust(-140.0, -140.0)
                self.brake += 1
            else:
                self.set_thrust(0.0, 0.0)
                self.docked = True
            self.show(ann, f"BRAKING {self.brake}/16 (b={b_ratio:.4f})", col=(0, 0, 255))
            return

        # ================= 2. FASE MASUK CERUK U SPOT 2 (LURUS HALUS) =================
        if ready_to_dock and not greens and not reds and not obsts:
            if banner_x is not None:
                err = (w / 2.0 + DOCK_PX_OFFSET) - banner_x
                turn = max(-22.0, min(22.0, 0.14 * err))
                fwd = 34.0
                self.set_thrust(fwd - turn, fwd + turn)
                self.show(ann, f"DOCKING TO BANNER (b={b_ratio:.4f})", col=(255, 200, 0))
            else:
                self.set_thrust(32.0, 32.0)
                self.show(ann, "DOCKING STRAIGHT IN", col=(255, 200, 0))
            return

        # ================= 3. KORIDOR NAVIGASI (GREEN KIRI, RED KANAN) =================
        near_g = max(greens, key=lambda item: item[1]) if greens else None
        near_r = max(reds, key=lambda item: item[1]) if reds else None

        left_bound = (near_g[0] + BUOY_MARGIN_RAD) if near_g else -0.45
        right_bound = (near_r[0] - BUOY_MARGIN_RAD) if near_r else +0.45

        if left_bound > right_bound:
            mid_b = (left_bound + right_bound) / 2.0
            left_bound = mid_b - 0.05
            right_bound = mid_b + 0.05

        corridor_mid = (left_bound + right_bound) / 2.0

        # ================= 4. PENGHINDARAN OBSTACLE DINI (KUNING & HITAM) =================
        active_obs = [o for o in obsts if OBS_ACTIVE_Y <= o[1] <= OBS_PASS_Y]
        cur_obs = max(active_obs, key=lambda item: item[1]) if active_obs else None

        mode = "CORRIDOR"
        target_ang = corridor_mid

        if cur_obs is not None:
            self.obs_lost_frames = 0
            obs_ang, obs_y, _, obs_col = cur_obs

            if self.locked_avoid_dir is None:
                if near_r is None and near_g is not None:
                    self.locked_avoid_dir = 'L'
                elif near_g is None and near_r is not None:
                    self.locked_avoid_dir = 'R'
                else:
                    space_right = right_bound - obs_ang
                    space_left = obs_ang - left_bound
                    self.locked_avoid_dir = 'R' if space_right >= space_left else 'L'

            if self.locked_avoid_dir == 'R':
                safe_left = obs_ang + OBS_MARGIN_RAD
                target_ang = max(safe_left, (safe_left + right_bound) / 2.0)
                target_ang = min(target_ang, right_bound + 0.10)
                mode = f"AVOID {obs_col.upper()} -> RIGHT"
            else:
                safe_right = obs_ang - OBS_MARGIN_RAD
                target_ang = min(safe_right, (left_bound + safe_right) / 2.0)
                target_ang = max(target_ang, left_bound - 0.10)
                mode = f"AVOID {obs_col.upper()} -> LEFT"
        else:
            if self.locked_avoid_dir is not None:
                self.obs_lost_frames += 1
                if self.obs_lost_frames >= 6:
                    self.locked_avoid_dir = None
                    self.obs_lost_frames = 0

        # ================= 5. KENDALI BLIND SESAAT / HALUS =================
        if greens or reds or obsts:
            self.last_seen_t = now
            self.last_target_ang = target_ang
        elif now - self.last_seen_t < 1.5:
            target_ang = self.last_target_ang * 0.7
            mode = "HOLD LAST"
        else:
            target_ang = 0.0
            mode = "SEARCH STRAIGHT"

        tx = int(w / 2.0 + F * math.tan(target_ang - self.cam_offset))
        tx = max(0, min(w - 1, tx))
        cv2.line(ann, (tx, int(h * 0.4)), (tx, h), (255, 255, 0), 3)
        cv2.line(ann, (int(w / 2), int(h * 0.7)), (int(w / 2), h), (200, 200, 200), 1)

        # ================= 6. MIXING THRUSTER KIRI & KANAN (ADAPTIF BEBAN) =================
        turn = KP_STEER * target_ang
        turn = max(-MAX_TURN, min(MAX_TURN, turn))

        speed_factor = 1.0 - min(0.65, abs(target_ang) / 0.50)
        fwd = MIN_SPEED + (BASE_SPEED - MIN_SPEED) * speed_factor
        if cur_obs is not None:
            # Melambat otomatis saat menghindar obstacle agar tidak nyenggol buoy
            fwd = min(fwd, 32.0 if cur_obs[1] > 0.45 else 40.0)

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