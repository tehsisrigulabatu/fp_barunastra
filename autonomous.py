#!/usr/bin/env python3
"""Misi otonom penuh ASV Transporter (Final Project NALA 16) di VRX / Gazebo, world skala x5.

  INIT -> PICKUP (Spot 1, tahan posisi di slot) -> FTP (gate hijau-merah + hindari rintangan)
       -> DOCKING (slot Spot 2) -> UNLOAD (taruh 3 barang di zona putih)
       -> BUZZER + WATER CANNON (tepat saat barang terakhir dilepas) -> DONE

Kill switch: publish std_msgs/Bool true ke /asv/kill -> semua thruster 0 dan lengan berhenti di tempat
(misi lanjut saat false). Status misi di /asv/state (std_msgs/String, JSON) untuk dashboard web.

Jalankan (setelah ./jalankan_wamv.sh selesai memasang lengan):
    python3 autonomous.py                           # misi penuh
    python3 autonomous.py --ros-args -p gui:=false  # tanpa jendela OpenCV
    python3 autonomous.py --ros-args -p skip_pickup:=true   # uji FTP + docking saja
Uji logika tanpa Gazebo:  python3 tests/sim_ftp_offline.py
"""
import json
import math
import os
import sys
import threading
import time
import traceback
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float64, String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import gz.msgs10.image_pb2 as gz_image  # noqa: E402
import gz.msgs10.pose_v_pb2 as pvb  # noqa: E402
from ultralytics import YOLO  # noqa: E402

from asv import config as C  # noqa: E402
from asv.arm_ctrl import ArmController, ArmIK  # noqa: E402
from asv.aux_effects import AuxEffects  # noqa: E402
from asv.control import BoatState, LOSGuidance, Pilot, StationKeeper, StationKeeper3DOF  # noqa: E402
from asv.geometry import CourseFrame, boat_to_world, world_to_boat, yaw_of  # noqa: E402
from asv.perception import ArmPerception, FrontPerception, OverviewLocator  # noqa: E402
from asv.planner import LandmarkMap, plan_path  # noqa: E402
from lengan import Arm, wait_pose  # noqa: E402


class PoseSource:
    """Pose semua entitas dari gz dynamic_pose/info, plus riwayat ber-timestamp untuk WAM-V
    (supaya deteksi kamera dipasangkan dengan pose kapal pada saat gambar diambil)."""

    def __init__(self, gz_node):
        self.lock = threading.Lock()
        self.p = {}
        self.hist = deque(maxlen=400)
        self.stamp = 0.0
        gz_node.subscribe(pvb.Pose_V, f"/world/{C.WORLD}/dynamic_pose/info", self._cb)

    def _cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        with self.lock:
            for q in m.pose:
                self.p[q.name] = (q.position.x, q.position.y, q.position.z,
                                  q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w)
                if q.name == "wamv":
                    self.hist.append((t, self.p["wamv"]))
            self.stamp = t

    def get(self, name):
        with self.lock:
            return self.p.get(name)

    def roll_pitch(self):
        """(roll, pitch) kapal dalam derajat."""
        w = self.get("wamv")
        if w is None:
            return (0.0, 0.0)
        qx, qy, qz, qw = w[3:7]
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
        return math.degrees(roll), math.degrees(pitch)

    def wamv_at(self, t):
        with self.lock:
            if not self.hist:
                return None
            best = min(self.hist, key=lambda h: abs(h[0] - t))
            return best[1] if abs(best[0] - t) < 0.5 else self.hist[-1][1]


class ASVMission(Node):
    def __init__(self):
        super().__init__("asv_mission")
        self.declare_parameter("gui", True)
        self.declare_parameter("skip_pickup", False)
        self.declare_parameter("gt_fallback", C.ALLOW_GT_FALLBACK)
        self.declare_parameter("azimuth_hold", C.AZIMUTH_HOLD)
        self.gui = bool(self.get_parameter("gui").value)
        self.skip_pickup = bool(self.get_parameter("skip_pickup").value)
        self.gt_fallback = bool(self.get_parameter("gt_fallback").value)
        self.log = self.get_logger()

        # ---------------- Gazebo (gz transport) lewat kelas Arm bawaan arena
        self.arm = Arm()
        self.poses = PoseSource(self.arm.n)
        self.overview = OverviewLocator(self.log)
        self.arm.n.subscribe(gz_image.Image, C.OVERVIEW_GZ_TOPIC, self._overview_cb)
        self.aux = AuxEffects(self.arm.n, self.log)

        # ---------------- persepsi
        self.bridge = CvBridge()
        self.lmap = LandmarkMap()
        self.map_lock = threading.Lock()
        self.front = FrontPerception(YOLO(C.FRONT_MODEL), self.lmap, self.log)
        self.armvis = ArmPerception(YOLO(C.ARM_MODEL), self.log)
        self.log.info(f"frontnav: {self.front.model.names} | arm: {self.armvis.model.names}")
        self.front_frame = None
        self.front_lock = threading.Lock()
        self.front_active = False

        self.create_subscription(Image, C.FRONT_CAM_TOPIC, self._front_cb, 2)
        self.create_subscription(CameraInfo, C.FRONT_CAM_TOPIC.replace("image_raw", "camera_info"),
                                 self._front_info_cb, 2)
        self.create_subscription(Image, C.ARM_CAM_TOPIC, self._arm_cb, 2)
        self.create_subscription(Bool, "/asv/kill", self._kill_cb, 10)

        self.pub_l = self.create_publisher(Float64, C.THRUST_L, 10)
        self.pub_r = self.create_publisher(Float64, C.THRUST_R, 10)
        self.pub_lpos = self.create_publisher(Float64, C.THRUST_L_POS, 10)
        self.pub_rpos = self.create_publisher(Float64, C.THRUST_R_POS, 10)
        self.pub_state = self.create_publisher(String, "/asv/state", 10)
        self.pub_buzzer = self.create_publisher(Bool, "/asv/buzzer", 10)
        self.pub_cannon = self.create_publisher(Bool, "/asv/water_cannon", 10)

        # ---------------- kendali
        self.killed = False
        self.phase = "INIT"
        self.mode = "idle"            # idle | hold | path
        self.st = BoatState()
        self.pilot = Pilot()
        self.keeper = StationKeeper()
        self.keeper3 = StationKeeper3DOF()
        self.azimuth_hold = bool(self.get_parameter("azimuth_hold").value)
        self.los = LOSGuidance()
        self.path_lock = threading.Lock()
        self.u_ref = 0.0
        self.cmd = (0.0, 0.0)
        self.nav_info = {}
        self.frame = None             # CourseFrame (ditetapkan dari pose start)
        self.loaded = []              # barang di kompartemen kapal
        self.placed = []
        self.pick_boat_xy = {}        # posisi barang (kerangka kapal) saat diambil di Spot 1
        self.gates_log = {}
        self.events = []

        self.armc = ArmController(self.arm, self.armvis, self.poses, self.log, self._arm_must_wait)

        self.create_timer(0.05, self._control_tick)
        self.create_timer(0.5, self._publish_state)
        if self.gui:
            # SEMUA panggilan cv2 GUI dari SATU thread (HighGUI/Qt tidak thread-safe; timer di
            # MultiThreadedExecutor berpindah-pindah thread -> jendela "Not Responding")
            threading.Thread(target=self._gui_loop, daemon=True).start()
        threading.Thread(target=self._front_worker, daemon=True).start()
        threading.Thread(target=self._mission, daemon=True).start()
        self.log.info("=== ASV MISSION NODE SIAP ===")

    # ================================================================== callbacks sensor
    def _overview_cb(self, m):
        if m.width == 0:
            return
        img = np.frombuffer(m.data, np.uint8)
        try:
            img = img.reshape(m.height, m.width, -1)[:, :, :3]
        except ValueError:
            return
        self.overview.set_frame(img)

    def _front_info_cb(self, m):
        if m.k[0] > 1.0:
            self.front.set_intrinsics(m.width, m.height, float(m.k[0]))

    def _front_cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.front_lock:
            self.front_frame = (t, msg)

    def _arm_cb(self, msg):
        ap = self.poses.get("asv_arm") or self.poses.get("wamv")
        g = self.poses.get("gripper")
        if ap is None or g is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.armvis.set_frame(frame, self.armvis.cam_pose(ap, g))

    def _kill_cb(self, msg):
        if bool(msg.data) != self.killed:
            self.killed = bool(msg.data)
            self.log.warn(f"########## KILL SWITCH {'AKTIF' if self.killed else 'DILEPAS'} ##########")
            if self.killed:
                self._thrust(0.0, 0.0)

    def _front_worker(self):
        """Thread YOLO kamera depan: selalu memproses frame TERBARU (frame lama dibuang)."""
        last_t = -1.0
        while rclpy.ok():
            if not self.front_active:
                time.sleep(0.1)
                continue
            with self.front_lock:
                fr = self.front_frame
            if fr is None or fr[0] == last_t:
                time.sleep(0.01)
                continue
            last_t, msg = fr
            pose = self.poses.wamv_at(last_t)
            if pose is None:
                continue
            try:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                with self.map_lock:
                    self.front.process(img, pose, last_t)
            except Exception as e:  # noqa: BLE001
                self.log.error(f"[FRONT] {e!r}")

    # ================================================================== kendali (20 Hz)
    def _thrust(self, left, right, az_l=0.0, az_r=0.0):
        self.cmd = (left, right)
        self.pub_l.publish(Float64(data=float(left)))
        self.pub_r.publish(Float64(data=float(right)))
        self.pub_lpos.publish(Float64(data=float(az_l)))
        self.pub_rpos.publish(Float64(data=float(az_r)))

    def _arm_must_wait(self):
        """Lengan dijeda saat kill switch aktif atau kapal miring berlebihan."""
        if self.killed:
            return True
        roll, pitch = self.poses.roll_pitch()
        if max(abs(roll), abs(pitch)) > C.ROLL_PITCH_LIMIT:
            now = time.time()
            if now - getattr(self, "_tilt_log_t", 0.0) > 2.0:
                self._tilt_log_t = now
                self.log.warn(f"[ARM] kapal miring roll {roll:+.1f} pitch {pitch:+.1f} deg -> lengan dijeda")
            return True
        return False

    def _control_tick(self):
        w = self.poses.get("wamv")
        if w is None:
            return
        t = self.poses.stamp
        self.st.update(t, w[0], w[1], yaw_of(w))
        if self.killed:
            self._thrust(0.0, 0.0)
            self.pilot.reset()
            return
        if self.mode == "hold" and self.keeper.ref is not None:
            if self.azimuth_hold:
                ((tl, al), (tr, ar)), (ex, ey, e_psi) = self.keeper3.command(self.st, t)
                self.nav_info = dict(e_f=round(ex, 2), e_lat=round(ey, 2), e_psi_deg=round(math.degrees(e_psi), 1))
                self._thrust(tl, tr, al, ar)
            else:
                (left, right), e_f, e_psi = self.keeper.command(self.st)
                self.nav_info = dict(e_f=round(e_f, 2), e_psi_deg=round(math.degrees(e_psi), 1))
                self._thrust(left, right)
        elif self.mode == "path":
            with self.path_lock:
                psi_d, e, rem, reached = self.los.step(w[0], w[1])
                u_d = self.los.speed_for(self.u_ref)
            if psi_d is None:
                psi_d = self.frame.heading
            s_now = self.frame.to_sl(w)[0]
            if s_now < C.SLOT_EXIT:
                u_d = min(u_d, C.U_EXIT)
            if rem < C.DOCK_SLOW_DIST:
                u_d = min(u_d, max(C.U_DOCK_MIN, self.u_ref * rem / C.DOCK_SLOW_DIST))
            (left, right), e_psi = self.pilot.command(self.st, psi_d, u_d, t)
            self.nav_info = dict(xte=round(e, 2), rem=round(rem, 1), u=round(self.st.u, 2), u_d=round(u_d, 2),
                                 e_psi_deg=round(math.degrees(e_psi), 1))
            self.nav_info["rem_raw"] = rem
            self._thrust(left, right)
        else:
            self._thrust(0.0, 0.0)

    def _hold_here(self, heading=None):
        w = self.poses.get("wamv")
        self._hold_at(w[0], w[1], yaw_of(w) if heading is None else heading)

    def _hold_at(self, x, y, psi):
        self.keeper.set(x, y, psi)
        self.keeper3.set(x, y, psi)
        self.mode = "hold"

    # ================================================================== status & GUI
    def _publish_state(self):
        try:
            self._publish_state_impl()
        except Exception as e:  # noqa: BLE001  (status tidak boleh menjatuhkan executor/kendali)
            self.log.error(f"[STATE] {e!r}")

    def _publish_state_impl(self):
        w = self.poses.get("wamv")
        d = dict(phase=self.phase, mode=self.mode, killed=self.killed, loaded=self.loaded, placed=self.placed,
                 thrust=[round(self.cmd[0], 1), round(self.cmd[1], 1)],
                 pose=None if w is None else [round(w[0], 2), round(w[1], 2), round(math.degrees(yaw_of(w)), 1)],
                 nav={k: v for k, v in self.nav_info.items() if k != "rem_raw"},
                 gates=self.gates_log, events=self.events[-5:])
        self.pub_state.publish(String(data=json.dumps(d)))

    def _gui_frame(self, name):
        """Frame untuk ditampilkan: anotasi deteksi bila masih baru (<1 s), selain itu gambar kamera live."""
        src = self.front if name == "FRONT" else self.armvis
        with src.lock:
            fresh = src.last_ann is not None and time.time() - src.last_ann_t < 1.0
            img = src.last_ann.copy() if fresh else None
        if img is None:
            if name == "FRONT":
                with self.front_lock:
                    fr = self.front_frame
                img = None if fr is None else self.bridge.imgmsg_to_cv2(fr[1], desired_encoding="bgr8")
            else:
                with self.armvis.lock:
                    img = None if self.armvis.frame is None else self.armvis.frame.copy()
        return img

    def _gui_loop(self):
        for name in ("FRONT", "ARM"):
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow("FRONT", 960, 540)
        cv2.resizeWindow("ARM", 640, 480)
        while rclpy.ok():
            for name in ("FRONT", "ARM"):
                try:
                    img = self._gui_frame(name)
                except Exception:  # noqa: BLE001
                    img = None
                if img is None:
                    continue
                info = f"{self.phase} | {self.mode}{' | KILLED' if self.killed else ''}"
                if name == "FRONT":
                    rp = self.poses.roll_pitch()
                    info += f" | roll {rp[0]:+.1f} pitch {rp[1]:+.1f} deg"
                cv2.putText(img, info, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow(name, img)
            cv2.waitKey(30)          # memproses event jendela ~30 Hz
        cv2.destroyAllWindows()

    def event(self, text):
        self.events.append(dict(t=round(self.poses.stamp, 1), msg=text))
        self.log.info(f"[EVENT] {text}")

    def wait_until(self, cond, timeout, period=0.1):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if not self.killed and cond():
                return True
            time.sleep(period)
        return False

    # ================================================================== misi
    def _mission(self):
        try:
            self._init()
            if not self.skip_pickup:
                self.phase = "PICKUP"
                self._pickup()
            else:
                self.loaded = []
            self.phase = "FTP"
            self._ftp_and_dock()
            self.phase = "UNLOAD"
            self._unload()
            self.phase = "DONE"
            self.event("MISI SELESAI")
        except Exception as e:  # noqa: BLE001
            self.log.error(f"[MISSION] EXCEPTION {e!r}\n{traceback.format_exc()}")
            self.phase = "ERROR"
            self._hold_here()

    def _init(self):
        self.log.info("[INIT] menunggu pose wamv / lengan dari Gazebo ...")
        for n in ("wamv", "gripper"):
            wait_pose(self.arm, n, t=30)
            if not self.wait_until(lambda: self.poses.get(n) is not None, 30):
                raise RuntimeError(f"pose '{n}' tidak ada. Sudah ./jalankan_wamv.sh dan lengan terpasang?")
        self.wait_until(lambda: self.poses.get("wamv") is not None and self.st.t is not None, 5)
        w = self.poses.get("wamv")
        self.frame = CourseFrame(w[:2], yaw_of(w))
        self.log.info(f"[INIT] start ({w[0]:.2f}, {w[1]:.2f}) heading {math.degrees(yaw_of(w)):.1f} deg")
        self._hold_here(self.frame.heading)
        self.front_active = True                      # peta landmark mulai dibangun sejak di slot start
        self.armc.sync()
        self.armc.move(lift=0.0)

    # ------------------------------------------------------------------ PICKUP di Spot 1
    def _item_candidates(self):
        """Lokasi kasar barang: kamera overview (HSV kuning) dalam jangkauan lengan."""
        w = self.poses.get("wamv")
        pivot = boat_to_world(w, C.ARM_PIVOT)
        reach = C.ARM_CARR0 + C.ARM_SLIDE_MAX
        self.wait_until(self.overview.has_frame, 5.0)
        cands = [p for p in self.overview.items_near(pivot, reach) if ArmIK.reachable(*world_to_boat(w, p))]
        src = "overview"
        if not cands and self.gt_fallback:
            self.log.warn("[PICKUP] kamera overview tidak menemukan barang -> FALLBACK pose Gazebo")
            cands = [self.poses.get(f"item_{k}")[:2] for k in C.ITEMS if self.poses.get(f"item_{k}")]
            src = "gazebo"
        self.log.info(f"[PICKUP] {len(cands)} kandidat barang ({src}): "
                      + ", ".join(f"({x:.2f},{y:.2f})" for x, y in cands))
        # urut menurut sudut sendi yaw supaya lengan berputar satu arah
        return sorted(cands, key=lambda p: ArmIK.boat_xy_to_joints(*world_to_boat(w, p))[0])

    def _gt_class_near(self, xy, tol=1.0):
        best, bd = None, tol
        for k in C.ITEMS:
            p = self.poses.get(f"item_{k}")
            if p and math.hypot(p[0] - xy[0], p[1] - xy[1]) < bd:
                best, bd = k, math.hypot(p[0] - xy[0], p[1] - xy[1])
        return best

    def _pickup(self):
        self.event("PICKUP mulai")
        z_item = C.DOCK_TOP_Z + 0.7 * C.ITEM
        for attempt in range(2):
            for xy in self._item_candidates():
                if len(self.loaded) == len(C.ITEMS):
                    break
                if any(math.hypot(xy[0] - p[0], xy[1] - p[1]) < 0.4 for p in self._loaded_world_xy()):
                    continue
                ref, cls = self.armc.refine("any_item", xy, z_item)
                if cls == "poly" or (cls in self.loaded):
                    # YOLO ragu / kelas sudah terambil: pilih dari sisa kelas yang bukan bola
                    rest = [k for k in ("cube", "prism") if k not in self.loaded]
                    self.log.warn(f"[PICKUP] kelas ambigu ({cls}) di {xy}, sisa kandidat {rest}")
                    cls = rest[0] if len(rest) == 1 else (self._gt_class_near(xy) if self.gt_fallback else None)
                if cls is None:
                    cls = self._gt_class_near(xy) if self.gt_fallback else None
                    if cls is None:
                        continue
                    self.log.warn(f"[PICKUP] kamera lengan gagal mengenali barang di {xy}, FALLBACK kelas Gazebo={cls}")
                    ref = self.poses.get(f"item_{cls}")[:2]
                if cls in self.loaded:
                    continue
                self.log.info(f"[PICKUP] ambil {cls} di ({ref[0]:.2f},{ref[1]:.2f})")
                if not self.armc.pick(cls, ref, C.DOCK_TOP_Z):
                    continue
                w = self.poses.get("wamv")
                self.pick_boat_xy[cls] = world_to_boat(w, ref)
                cxy = self.armc.container_world(cls)
                if self.armc.place(cls, cxy, self.armc.deck_z()):
                    self.loaded.append(cls)
                    self.event(f"{cls} masuk kompartemen ({len(self.loaded)}/3)")
                else:
                    self.log.error(f"[PICKUP] {cls} tidak mendarat di kompartemen!")
            if len(self.loaded) == len(C.ITEMS):
                break
        self.armc.stow()
        if not self.loaded:
            raise RuntimeError("tidak ada barang yang berhasil diambil")
        self.event(f"PICKUP selesai: {self.loaded}")

    def _loaded_world_xy(self):
        w = self.poses.get("wamv")
        return [boat_to_world(w, self.pick_boat_xy[k]) for k in self.loaded if k in self.pick_boat_xy]

    # ------------------------------------------------------------------ FTP + DOCKING
    def _banner_lateral(self):
        """Posisi lateral banner (l) = median perpotongan sinar-sinar banner dengan garis dinding Spot 2."""
        s_wall = C.SPOT_TO_SPOT + C.BANNER_TO_SLOT
        ls = []
        for cx, cy, brg in list(self.front.banner_rays):
            s0, l0 = self.frame.to_sl((cx, cy))
            a = brg - self.frame.heading
            if math.cos(a) < 0.2 or s_wall - s0 < 3.0:
                continue
            ls.append(l0 + (s_wall - s0) * math.tan(a))
        if len(ls) < 5:
            return None
        return float(np.median(ls))

    def _dock_target(self):
        """(s, l) pusat slot Spot 2. s dari geometri guidebook (jarak slot-ke-slot), l dari arah banner
        (visi) bila tersedia. Banner berada di dinding tepat di belakang pusat slot."""
        lb = self._banner_lateral()
        if lb is not None and abs(lb) < 5.0:
            return (C.SPOT_TO_SPOT, lb), "banner"
        return (C.SPOT_TO_SPOT, 0.0), "prior"

    def _replan(self):
        w = self.poses.get("wamv")
        with self.map_lock:
            self.lmap.merge()
            end_sl, src = self._dock_target()
            wps, gates = plan_path(self.lmap, self.frame, (0.0, 0.0), end_sl)
        xy = [(*self.frame.to_xy(p["s"], p["l"]), p["kind"]) for p in wps]
        with self.path_lock:
            self.los.set_path(xy, w[0], w[1])
        return gates, end_sl, src

    def _check_gates(self, gates, prev_xy, cur_xy):
        """Catat saat kapal memotong garis gerbang (segmen hijau-merah) dan offset dari tengahnya."""
        for g in gates:
            key = f"g{g.green.id}-r{g.red.id}"
            if key in self.gates_log:
                continue
            a, b = np.array(g.green.xy), np.array(g.red.xy)
            p, q = np.array(prev_xy), np.array(cur_xy)
            r, s = q - p, b - a
            den = r[0] * s[1] - r[1] * s[0]
            if abs(den) < 1e-9:
                continue
            t = ((a - p)[0] * s[1] - (a - p)[1] * s[0]) / den
            u = ((a - p)[0] * r[1] - (a - p)[1] * r[0]) / den
            if 0 <= t <= 1:
                inside = 0 <= u <= 1
                off = (u - 0.5) * g.width
                self.gates_log[key] = dict(n=len(self.gates_log) + 1, width=round(g.width, 2),
                                             offset=round(off, 2), inside=bool(inside))
                lvl = self.log.info if inside else self.log.error
                lvl(f"[FTP] GERBANG #{len(self.gates_log)} (lebar {g.width:.1f} m) dilewati, "
                    f"offset dari tengah {off:+.2f} m {'OK' if inside else 'DI LUAR GERBANG!'}")

    def _ftp_and_dock(self):
        self.event("FTP mulai")
        self.front_active = True
        self.pilot.reset()
        self.u_ref = C.U_CRUISE
        gates, end_sl, src = self._replan()
        self.mode = "path"                 # _thrust di mode path mengirim sudut mesin 0 (diferensial biasa)
        prev = self.poses.get("wamv")[:2]
        last_log = 0.0
        t_start = time.time()
        while True:
            time.sleep(0.4)
            if self.killed:
                continue
            gates, end_sl, src = self._replan()
            cur = self.poses.get("wamv")[:2]
            self._check_gates(gates, prev, cur)
            prev = cur
            rem = self.nav_info.get("rem_raw", 1e9)
            if time.time() - last_log > 2.0:
                last_log = time.time()
                with self.map_lock:
                    nconf = {c: len(self.lmap.confirmed(c)) for c in ("green", "red", "yellow", "black",
                                                                     "target_banner")}
                s, l = self.frame.to_sl(cur)
                self.log.info(f"[FTP] s={s:5.1f} l={l:+5.2f} | gerbang={len(gates)} landmark={nconf} | "
                              f"dock={src} ({end_sl[0]:.1f},{end_sl[1]:+.2f}) | {self.nav_info}")
            if rem < C.DOCK_TOL:
                break
            if time.time() - t_start > 900:
                raise RuntimeError("FTP timeout")
        dock_xy = self.frame.to_xy(*end_sl)
        self._hold_at(dock_xy[0], dock_xy[1], self.frame.heading)
        self.wait_until(lambda: abs(self.st.u) < 0.08 and abs(self.nav_info.get("e_f", 9)) < 0.3, 30)
        w = self.poses.get("wamv")
        err = math.hypot(w[0] - dock_xy[0], w[1] - dock_xy[1])
        self.event(f"DOCKED di slot Spot 2 (target {src}), galat posisi {err:.2f} m")
        self.log.info(f"[FTP] ringkasan gerbang: {list(self.gates_log.values())}")
        with self.map_lock:
            for m in sorted(self.lmap.confirmed(), key=lambda m: (m.cls, m.x)):
                self.log.info(f"[MAP] {m.cls:13s} #{m.id:3d} ({m.x:7.2f},{m.y:7.2f}) hits={m.hits} "
                              f"min_rng={m.min_rng:.1f}")

    # ------------------------------------------------------------------ UNLOAD + BUZZER + WATER CANNON
    def _drop_targets(self):
        w = self.poses.get("wamv")
        pivot = boat_to_world(w, C.ARM_PIVOT)
        reach = C.ARM_CARR0 + C.ARM_SLIDE_MAX
        marks = [m for m in self.overview.marks_near(pivot, reach) if ArmIK.reachable(*world_to_boat(w, m))]
        self.log.info(f"[UNLOAD] {len(marks)} zona taruh dari overview: "
                      + ", ".join(f"({x:.2f},{y:.2f})" for x, y in marks))
        targets, used = {}, set()
        for k in self.loaded:
            # Spot 2 = cermin Spot 1: zona taruh barang k ~ posisi ambil dicerminkan di sumbu lintang kapal
            bx, by = self.pick_boat_xy.get(k, C.CONTAINER_XY[k])
            prior = boat_to_world(w, (-bx, by))
            best = None
            for i, m in enumerate(marks):
                if i in used:
                    continue
                d = math.hypot(m[0] - prior[0], m[1] - prior[1])
                if d < 2.5 and (best is None or d < best[0]):
                    best = (d, i)
            if best is not None:
                used.add(best[1])
                targets[k] = marks[best[1]]
            else:
                targets[k] = prior
                self.log.warn(f"[UNLOAD] zona taruh {k} tidak terlihat di overview, pakai cermin posisi ambil")
        return targets

    def _banner_bearing_now(self):
        """Sudut banner dari haluan: deteksi live bila ada; di dalam slot banner terpotong tepi atas gambar,
        jadi pakai posisi banner yang dipetakan saat mendekat (perpotongan sinar-sinar banner)."""
        if self.poses.stamp - self.front.banner_t < 1.0 and self.front.banner_bearing is not None:
            return self.front.banner_bearing, "live"
        if self._banner_lateral() is None:
            return None, None
        w = self.poses.get("wamv")
        bx, by = world_to_boat(w, self._banner_xyz()[:2])
        return math.atan2(by, bx), "peta"

    def _aim_cannon(self):
        """Pastikan banner target tepat di depan haluan, koreksi heading tahan bila perlu."""
        self.front_active = True
        for _ in range(3):
            time.sleep(0.5)
            brg, src = self._banner_bearing_now()
            if brg is None:
                self.log.warn("[CANNON] posisi banner tidak diketahui, pakai heading lintasan")
                return
            self.log.info(f"[CANNON] sudut banner dari haluan {math.degrees(brg):+.1f} deg ({src})")
            if abs(brg) < C.BANNER_AIM_TOL:
                return
            x0, y0, psi0 = self.keeper.ref
            self._hold_at(x0, y0, psi0 + brg)
            self.wait_until(lambda: abs(self.nav_info.get("e_psi_deg", 99)) < 1.0, 8.0)
            time.sleep(1.0)

    def _banner_xyz(self):
        lb = self._banner_lateral()
        x, y = self.frame.to_xy(C.SPOT_TO_SPOT + C.BANNER_TO_SLOT, 0.0 if lb is None else lb)
        return (x, y, C.BANNER_Z)

    def _fire_effects(self):
        """Dipanggil TEPAT saat barang terakhir dilepas."""
        w = self.poses.get("wamv")
        self.pub_buzzer.publish(Bool(data=True))
        self.pub_cannon.publish(Bool(data=True))
        print("\a", end="", flush=True)
        self.aux.buzzer_marker(w)
        self.aux.water_jet(w, self._banner_xyz())
        self.event("BUZZER ON + WATER CANNON ON (barang terakhir ditaruh)")

        def _off():
            time.sleep(max(C.BUZZER_TIME, C.CANNON_TIME))
            self.pub_buzzer.publish(Bool(data=False))
            self.pub_cannon.publish(Bool(data=False))
            self.aux.clear()
            self.event("BUZZER OFF + WATER CANNON OFF")
        threading.Thread(target=_off, daemon=True).start()

    def _unload(self):
        self._hold_here(self.keeper.ref[2] if self.keeper.ref else None)
        if not self.loaded:
            self.log.warn("[UNLOAD] tidak ada barang di kapal")
            self._aim_cannon()
            self._fire_effects()
            return
        self.event("UNLOAD mulai")
        self._aim_cannon()
        targets = self._drop_targets()
        # zona taruh di-refine SEKARANG selagi gripper kosong (barang yang dibawa menutupi kamera gripper)
        for k in list(targets):
            mxy, _ = self.armc.refine("mark", targets[k], C.DOCK_TOP_Z, max_jump=1.5)
            if mxy is not None:
                targets[k] = mxy
        self.armc.move(lift=0.0)
        deck_z = self.armc.deck_z()
        order = list(self.loaded)
        for i, k in enumerate(order):
            last = i == len(order) - 1
            cxy = self.armc.container_world(k)
            ref, cls = self.armc.refine(k, cxy, deck_z + 0.7 * C.ITEM, max_jump=0.7)
            if cls is None:
                ref = cxy
            if not self.armc.pick(k, ref, deck_z):
                self.log.error(f"[UNLOAD] gagal mengambil {k} dari kompartemen")
                continue
            mxy = targets[k]
            if last:
                self._place_and_fire(k, mxy)
            elif self.armc.place(k, mxy, C.DOCK_TOP_Z):
                self.placed.append(k)
                self.event(f"{k} ditaruh di Spot 2 ({len(self.placed)}/{len(order)})")
        self.armc.stow()

    def _place_and_fire(self, k, mxy):
        """Taruh barang terakhir; efek dipicu tepat setelah detach (bukan menunggu lengan naik)."""
        orig_detach = self.arm.detach

        def detach_and_fire(item):
            orig_detach(item)
            self._fire_effects()
        self.arm.detach = detach_and_fire
        try:
            ok = self.armc.place(k, mxy, C.DOCK_TOP_Z)
        finally:
            self.arm.detach = orig_detach
        if ok:
            self.placed.append(k)
            self.event(f"{k} ditaruh di Spot 2 ({len(self.placed)}/{len(self.loaded)}) - BARANG TERAKHIR")


def main():
    rclpy.init()
    node = ASVMission()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node._thrust(0.0, 0.0)
        except Exception:  # noqa: BLE001  (context ROS sudah ditutup oleh sinyal)
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
