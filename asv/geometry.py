"""Geometri murni (tanpa ROS / Gazebo) supaya bisa diuji offline.

Pose = (x, y, z, qx, qy, qz, qw). Kamera Gazebo: sumbu x = arah pandang, y = kiri gambar, z = atas gambar.
"""
import math

import numpy as np


def quat_to_R(q):
    qx, qy, qz, qw = q
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def rpy_to_R(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def yaw_of(pose):
    qx, qy, qz, qw = pose[3:7]
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def pose_T(pose):
    """Pose -> matriks homogen 4x4."""
    T = np.eye(4)
    T[:3, :3] = quat_to_R(pose[3:7])
    T[:3, 3] = pose[:3]
    return T


def xyz_rpy_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def T_to_quat(T):
    R = T[:3, :3]
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)


def world_to_boat(boat_pose, pw):
    """Titik dunia (x, y) -> kerangka kapal 2D (pakai yaw saja)."""
    yaw = yaw_of(boat_pose)
    dx, dy = pw[0] - boat_pose[0], pw[1] - boat_pose[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def boat_to_world(boat_pose, pb):
    yaw = yaw_of(boat_pose)
    c, s = math.cos(yaw), math.sin(yaw)
    return boat_pose[0] + c * pb[0] - s * pb[1], boat_pose[1] + s * pb[0] + c * pb[1]


class PinholeCam:
    """Kamera pinhole dengan konvensi sensor Gazebo."""

    def __init__(self, width, height, hfov=None, fx=None):
        self.w, self.h = width, height
        self.fx = fx if fx else (width / 2.0) / math.tan(hfov / 2.0)
        self.fy = self.fx
        self.cx, self.cy = width / 2.0, height / 2.0

    def ray(self, u, v):
        """Arah sinar (kerangka sensor, belum dinormalisasi) untuk piksel (u, v)."""
        return np.array([1.0, -(u - self.cx) / self.fx, -(v - self.cy) / self.fy])

    def angular_size(self, px):
        """Lebar piksel di tengah gambar -> setengah sudut (rad)."""
        return math.atan2(px / 2.0, self.fx)


def ray_plane_z(T_world_cam, cam, u, v, z_plane):
    """Titik potong sinar piksel dengan bidang horizontal z = z_plane (kerangka dunia). None jika sejajar/di belakang."""
    d = T_world_cam[:3, :3] @ cam.ray(u, v)
    o = T_world_cam[:3, 3]
    if abs(d[2]) < 1e-6:
        return None
    t = (z_plane - o[2]) / d[2]
    if t <= 0:
        return None
    return o + t * d


def sphere_range(cam, bbox_w_px, radius):
    """Jarak ke pusat bola dari lebar bbox (sudut pandang bola = 2 asin(r/d))."""
    a = cam.angular_size(bbox_w_px)
    if a <= 1e-4:
        return None
    return radius / math.sin(a)


def plate_range(cam, bbox_w_px, width):
    """Jarak ke papan datar (banner) yang menghadap kamera."""
    if bbox_w_px <= 1:
        return None
    return cam.fx * width / bbox_w_px


def localize_by_range(T_world_cam, cam, u, v, rng):
    """Titik dunia sejauh rng sepanjang sinar piksel (u, v)."""
    d = T_world_cam[:3, :3] @ cam.ray(u, v)
    d = d / np.linalg.norm(d)
    return T_world_cam[:3, 3] + rng * d


class CourseFrame:
    """Kerangka lintasan: origin p0, sumbu u (arah Spot 1 -> Spot 2). l positif ke kiri."""

    def __init__(self, p0, heading):
        self.p0 = np.array(p0[:2], dtype=float)
        self.u = np.array([math.cos(heading), math.sin(heading)])
        self.n = np.array([-self.u[1], self.u[0]])
        self.heading = heading

    def to_sl(self, p):
        d = np.asarray(p[:2], dtype=float) - self.p0
        return float(d @ self.u), float(d @ self.n)

    def to_xy(self, s, l):
        p = self.p0 + s * self.u + l * self.n
        return float(p[0]), float(p[1])
