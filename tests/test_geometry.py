#!/usr/bin/env python3
"""Uji konsistensi proyeksi kamera (tanpa Gazebo): titik dunia -> piksel -> kembali ke dunia.
Pakai: python3 tests/test_geometry.py"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from asv import config as C  # noqa: E402
from asv.arm_ctrl import ArmIK  # noqa: E402
from asv.geometry import (PinholeCam, localize_by_range, pose_T, ray_plane_z, sphere_range,  # noqa: E402
                          xyz_rpy_T)


def project(T_wc, cam, p):
    """Titik dunia -> piksel (u, v) dengan konvensi kamera Gazebo (x maju, y kiri, z atas)."""
    pc = np.linalg.inv(T_wc) @ np.array([*p, 1.0])
    x, y, z = pc[:3]
    assert x > 0, "titik di belakang kamera"
    return cam.cx - cam.fx * y / x, cam.cy - cam.fy * z / x


def yaw_pose(x, y, z, yaw):
    return (x, y, z, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def check(name, got, want, tol):
    err = float(np.linalg.norm(np.array(got[:2]) - np.array(want[:2])))
    print(f"  {name:40s} galat {err:.4f} m {'OK' if err < tol else 'GAGAL'}")
    return err < tol


ok = True
print("1) kamera overview (atas): piksel -> dunia")
cam = PinholeCam(1280, 720, hfov=1.2)
T = xyz_rpy_T((0, 0, 20 * C.SCALE), (0, 1.5708, 1.5708))
u, v = project(T, cam, (0, 0, 0))
print(f"  pusat kolam di piksel ({u:.1f}, {v:.1f}) (harus ~640, 360)")
for p in [(-55.5, 0.0, C.DOCK_TOP_Z + C.ITEM / 2), (-49.0, 5.0, C.DOCK_TOP_Z + C.ITEM / 2), (49.0, -5.0, C.DOCK_TOP_Z)]:
    u, v = project(T, cam, p)
    ok &= check(f"{p} -> ({u:.0f},{v:.0f}) -> balik", ray_plane_z(T, cam, u, v, p[2]), p, 0.01)
u1, _ = project(T, cam, (10, 0, 0))
print(f"  +x dunia ke arah u {'naik' if u1 > 640 else 'turun'} (piksel {u1:.0f})")

print("2) kamera depan WAM-V: bbox bola -> jarak + arah -> dunia")
camf = PinholeCam(1280, 720, hfov=C.FRONT_CAM_HFOV)
boat = yaw_pose(-40.0, 1.0, 0.0, math.radians(10))
Tf = pose_T(boat) @ xyz_rpy_T(C.FRONT_CAM_XYZ, (0, C.FRONT_CAM_PITCH, 0))
for buoy in [(-30.0, 8.0, 0.7 * C.BUOY_R), (-30.0, -2.0, 0.7 * C.BUOY_R), (-22.5, 4.0, 0.7 * C.OBS_R)]:
    r = C.BUOY_R if buoy == (-30.0, 8.0, 0.7 * C.BUOY_R) or buoy[1] == -2.0 else C.OBS_R
    u, v = project(Tf, camf, buoy)
    d = float(np.linalg.norm(np.array(buoy) - Tf[:3, 3]))
    w = 2 * camf.fx * math.tan(math.asin(r / d))           # lebar bbox ideal
    rng = sphere_range(camf, w, r)
    p = localize_by_range(Tf, camf, u, v, rng)
    ok &= check(f"buoy {buoy[:2]} jarak {d:.1f} m (bbox {w:.0f}px)", p, buoy, 0.15)

print("3) kamera lengan: gripper di atas titik -> proyeksi ke bidang barang")
camA = PinholeCam(*C.ARM_CAM_WH, hfov=C.ARM_CAM_HFOV)
boat = yaw_pose(-48.5, 0.0, 0.0, 0.0)
for item in [(-49.0, 5.0), (-55.5, 0.0), (-49.0, -5.0)]:
    bx, by = item[0] - boat[0], item[1] - boat[1]
    yaw, slide = ArmIK.boat_xy_to_joints(bx, by)
    gx, gy = ArmIK.joints_to_boat_xy(yaw, slide)
    ok &= check(f"IK {item} -> yaw {yaw:+.2f} slide {slide:.2f} -> FK", (gx + boat[0], gy + boat[1]), item, 0.12)
    grip_rel = (gx, gy, C.ARM_GRIP_Z0, 0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
    Ta = pose_T(boat) @ pose_T(grip_rel) @ xyz_rpy_T(C.ARM_CAM_XYZ, (0, C.ARM_CAM_PITCH, 0))
    z = C.DOCK_TOP_Z + 0.7 * C.ITEM
    target = (item[0], item[1], z)
    u, v = project(Ta, camA, target)
    inside = 0 <= u < camA.w and 0 <= v < camA.h
    print(f"  barang {item} terlihat di piksel ({u:.0f},{v:.0f}) {'(di dalam gambar)' if inside else 'DI LUAR GAMBAR'}")
    ok &= inside
    ok &= check("   piksel -> bidang z barang", ray_plane_z(Ta, camA, u, v, z), target, 0.01)
print("HASIL:", "LULUS" if ok else "GAGAL")
sys.exit(0 if ok else 1)
