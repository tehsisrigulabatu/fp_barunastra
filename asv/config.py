"""Semua konstanta misi di satu tempat. Ukuran arena diambil dari figs.CFG (guidebook) lalu dikali SCALE.

Konvensi kerangka:
  - DUNIA  : origin pusat kolam, x ke Spot 2, y ke kiri, z ke atas (sama dengan README).
  - KAPAL  : base_link WAM-V, x ke haluan, y ke kiri.
  - COURSE : s = jarak sepanjang sumbu lintasan (dari posisi start ke arah Spot 2), l = lateral (kiri +).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
from figs import CFG  # noqa: E402

SCALE = 5.0                      # world x5 untuk WAM-V bawaan VRX
WORLD = "pengiriman_kapal_otonom"

# ----------------------------------------------------------------- model YOLO
MODEL_DIR = os.path.join(HERE, "dataset", "model")
FRONT_MODEL = os.path.join(MODEL_DIR, "frontnav.pt")   # black, dock, green, red, target_banner, yellow
ARM_MODEL = os.path.join(MODEL_DIR, "arm.pt")          # cube, dock, prism, sphere
FRONT_CONF = 0.35
ARM_CONF = 0.30

# ----------------------------------------------------------------- topik
FRONT_CAM_TOPIC = "/wamv/sensors/cameras/front_left_camera_sensor/image_raw"
ARM_CAM_TOPIC = "/arm/camera/image"
OVERVIEW_GZ_TOPIC = "overview_camera/image"            # dibaca langsung lewat gz-transport
THRUST_L = "/wamv/thrusters/left/thrust"
THRUST_R = "/wamv/thrusters/right/thrust"

# ----------------------------------------------------------------- geometri arena (meter, sudah x SCALE)
S = SCALE
ITEM = CFG["item"] * S                 # sisi / diameter barang
BUOY_R = CFG["buoy_r"] * S             # radius buoy hijau / merah
OBS_R = CFG["obs_r"] * S               # radius rintangan kuning / hitam
DOCK_TOP_Z = 0.25 * S                  # permukaan dermaga di atas air (make_world: DOCK_TOP)
BANNER_W = 1.0 * S                     # lebar papan banner (make_world: box 0.04 x 1.0 x 0.8)
BANNER_H = 0.8 * S
BANNER_Z = (0.25 + 0.45) * S           # tinggi pusat banner
# jarak pusat slot ke pusat slot (Spot 1 -> Spot 2) dan jarak dinding banner ke pusat slot
SLOT_CENTER_X = CFG["W"] / 2 - CFG["dock_w"] + CFG["slot_d"] / 2          # skala 1, dari pusat kolam
SPOT_TO_SPOT = 2.0 * SLOT_CENTER_X * S                                     # 97.5 m di x5
BANNER_TO_SLOT = (CFG["W"] / 2 - 0.08) * S - SLOT_CENTER_X * S             # 10.85 m di x5
SLOT_EXIT = (CFG["slot_d"] / 2 + 1.2) * S                                  # lurus sejauh ini saat keluar slot Spot 1
DOCK_STRAIGHT = (CFG["slot_d"] / 2 + 2.4) * S                              # lurus sejauh ini sebelum pusat slot Spot 2

# ----------------------------------------------------------------- kapal (WAM-V)
HALF_BEAM = 1.25                       # setengah lebar WAM-V (m)
BOAT_LEN = 4.9
# kamera depan VRX: front_left_camera x=0.75 y=0.1 z=1.5, pitch 15 derajat ke bawah, HFOV 1.396, 1280x720
FRONT_CAM_XYZ = (0.75, 0.1, 1.5)
FRONT_CAM_PITCH = math.radians(15.0)
FRONT_CAM_HFOV = 1.3962634

# ----------------------------------------------------------------- persepsi / peta landmark
MAP_ASSOC_R = 0.9 * S                  # radius asosiasi minimum deteksi ke landmark yang sama
MAP_ASSOC_K = 0.15                     # + radius asosiasi tumbuh dengan jarak (galat jarak ~ proporsional)
MAP_MAX_RANGE = 45.0                   # deteksi lebih jauh dari ini diabaikan (estimasi jarak kurang akurat)
MAP_MIN_RANGE = 2.0
MAP_CONFIRM_HITS = 4                   # landmark dianggap nyata setelah terlihat N kali
MAP_CONFIRM_RANGE = 30.0               # ... dan minimal sekali terlihat dari jarak <= ini
MAP_NEG_RANGE = 22.0                   # landmark di FOV sedekat ini tapi tidak terdeteksi -> dihitung "miss"
EDGE_MARGIN_PX = 4                     # bbox yang menempel tepi gambar diabaikan (lebarnya terpotong)
# Kalibrasi di Gazebo (32 deteksi, pose diketahui): bbox YOLO hijau/merah lebih longgar dari bolanya
# -> jarak_sebenarnya = faktor * jarak_dari_lebar_bbox (median).
RANGE_SCALE = {"green": 1.23, "red": 1.23, "yellow": 1.06, "black": 1.06}
BBOX_ASPECT = (0.75, 1.35)             # w/h di luar ini = buoy tertutup sebagian, lebarnya tidak dipakai
GATE_W_MIN = 0.8 * S                   # lebar pasangan hijau-merah yang masuk akal (gate 1.5..2.0, lane 3.5)
GATE_W_MAX = 4.5 * S
GATE_MAX_SKEW = 0.55                   # |cos| sudut vektor hijau->merah terhadap sumbu lintasan (0 = tegak lurus)
CORRIDOR_HALF = 6.0 * S                # landmark di luar |l| ini bukan bagian lintasan

# ----------------------------------------------------------------- perencanaan jalur
OBS_CLEAR = OBS_R + HALF_BEAM + 0.3 * S     # jarak pusat rintangan ke garis tengah kapal minimal
BUOY_CLEAR = BUOY_R + HALF_BEAM + 0.10 * S   # jarak pusat buoy tepi ke garis tengah kapal minimal
DETOUR_SPAN = 0.45 * S                       # titik hindar dipasang di s_obs +/- ini (kapal lewat sejajar)
GATE_STRAIGHT = 0.6 * S                      # jalur dipaksa lurus di tengah gerbang sepanjang +/- ini

# ----------------------------------------------------------------- kendali kapal
LOOKAHEAD = 1.1 * S                    # jarak carrot di depan proyeksi kapal pada lintasan (m)
U_CRUISE = 1.1                         # m/s di FTP
U_EXIT = 0.8
U_DOCK_MIN = 0.25
DOCK_SLOW_DIST = 18.0                  # mulai melambat sejauh ini dari titik docking
DOCK_TOL = 0.6                         # m, dianggap sudah di titik docking
# hidrodinamika WAM-V di VRX (wamv_gazebo_dynamics_plugin.xacro): xU=100, xUU=150, nR=800, Izz=446, m=180.
# Cocok dengan README: 150 N per thruster -> 0.5*(100*1.1 + 150*1.1^2) = 146 N -> ~1.1 m/s.
K_FF = 1.0                             # feed-forward per thruster = K_FF * 0.5 * (xU*u + xUU*u|u|)
DRAG_U, DRAG_UU = 100.0, 150.0
KP_U, KI_U = 150.0, 30.0               # PI kecepatan (N per m/s, per thruster)
# heading: torsi = 2 * diff * 1.03 m. KP 200 -> w_n ~0.9 rad/s, redaman nR=800 -> zeta ~0.9
KP_PSI, KD_PSI = 380.0, 120.0          # keluaran = diff (N): kiri = F - diff, kanan = F + diff
MAX_DIFF = 260.0
THRUST_MIN, THRUST_MAX = -400.0, 400.0
# station keeping (di slot Spot 1 saat pickup dan Spot 2 saat unload)
HOLD_KP_X, HOLD_KD_X = 60.0, 110.0    # per thruster: w_n ~0.7 rad/s, zeta ~1 (massa kapal + lengan ~230 kg)
HOLD_KP_PSI, HOLD_KD_PSI = 250.0, 100.0
HOLD_MAX_F, HOLD_MAX_DIFF = 90.0, 140.0
# station keeping 3-DOF dengan mesin azimuth WAM-V (/wamv/thrusters/*/pos, +-pi): menahan juga posisi SAMPING,
# supaya kapal tidak hanyut ke dinding slot saat lengan bergerak. Gain dalam gaya total (N, N/m, Nm/rad).
AZIMUTH_HOLD = True
THRUSTER_XY = ((-2.374, 1.027), (-2.374, -1.027))   # kiri, kanan (wamv_aft_thrusters.xacro)
THRUST_L_POS = "/wamv/thrusters/left/pos"
THRUST_R_POS = "/wamv/thrusters/right/pos"
SK_KP = (120.0, 110.0, 500.0)          # x, y, yaw
SK_KD = (240.0, 230.0, 300.0)
SK_KI = (3.0, 3.0, 0.0)
SK_MAX = (180.0, 180.0, 400.0)
AZ_MAX = math.pi - 0.05                 # batas sendi mesin +-pi (engine.xacro)
ROLL_PITCH_LIMIT = 5.0                 # derajat: lengan dijeda bila kapal miring melebihi ini

# ----------------------------------------------------------------- lengan (dari make_arm.py)
ARM_PIVOT = (-0.4, 0.0)                # sumbu yaw di kerangka kapal
ARM_CARR0 = 0.9                        # jangkauan saat slide = 0
ARM_SLIDE_MIN, ARM_SLIDE_MAX = 0.0, 7.0
ARM_LIFT_MIN, ARM_LIFT_MAX = -2.0, 0.0
ARM_YAW_LIM = 3.138                    # batas sendi SDF +-3.14159; kubus tepat di belakang kapal (yaw ~ pi)
ARM_GRIP_Z0 = 3.4 + 0.12 - 0.6        # z gripper (kerangka kapal) saat lift = 0
HANG = 0.35                            # pusat barang di bawah gripper saat dipegang
ARM_VMAX = {"yaw": 0.25, "slide": 0.6, "lift": 0.45}    # rad/s, m/s (dibatasi supaya kapal tidak goyang)
ARM_TOL = {"yaw": 0.015, "slide": 0.03, "lift": 0.03}
ARM_STOW = (0.0, 0.2, 0.0)
# kamera lengan (make_arm.py): di link gripper (0.5, 0, -0.03), pitch 1.9 rad, HFOV 1.4, 640x480
ARM_CAM_XYZ = (0.5, 0.0, -0.03)
ARM_CAM_PITCH = 1.9
ARM_CAM_HFOV = 1.4
ARM_CAM_WH = (640, 480)
# kompartemen di dek (make_arm.py): kubus x -2.04..-0.97 ; bola/prisma x 0.98..2.02 dipisah sekat y = 0
DECK_TOP_Z = 0.44
CONTAINER_XY = {"cube": (-1.505, 0.0), "sphere": (1.50, 0.46), "prism": (1.50, -0.46)}
CONTAINER_WALL_TOP = 0.74
SAFE_CARRY_DZ = 0.35                   # alas barang minimal setinggi ini di atas dinding kompartemen saat diayun
WAMV_SENSOR_TOP = 1.84                 # puncak lidar WAM-V (kerangka kapal): barang diayun harus di atasnya
CARRY_CLEAR = 0.25
PLACE_DROP_DZ = 0.06                   # alas barang dilepas setinggi ini di atas permukaan tujuan
REFINE_ITERS = 4
REFINE_TOL = 0.08                      # m, koreksi visual lebih kecil dari ini = sudah tepat di atas
GRAB_TOL = 0.30                        # m, jarak horizontal gripper-barang maksimal saat grab
ITEMS = ("cube", "sphere", "prism")

# ----------------------------------------------------------------- misi
ALLOW_GT_FALLBACK = True     # kalau kamera gagal total, boleh pakai pose Gazebo (dilog sebagai WARNING)
CANNON_TIME = 6.0            # detik semprot air
BUZZER_TIME = 6.0
BANNER_AIM_TOL = math.radians(3.0)


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))
