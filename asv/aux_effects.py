"""Buzzer & water cannon di simulasi = event marker (cukup menurut guidebook, tabel Control - Disruptive).

Tiap aktuator:
  - topik ROS std_msgs/Bool (/asv/buzzer, /asv/water_cannon) -> nanti dipetakan ke perintah STM32
  - penanda visual di Gazebo: lampu merah di atas kapal (buzzer) dan semburan air biru dari haluan
    sampai banner target (water cannon), dibuat lewat /world/<world>/create dan dihapus setelah selesai.
"""
import math

import numpy as np

import gz.msgs10.boolean_pb2 as bb
import gz.msgs10.entity_factory_pb2 as ef
import gz.msgs10.entity_pb2 as ep

from . import config as C
from .geometry import T_to_quat

NOZZLE_BOAT = (2.3, 0.0, 1.3)       # posisi nosel di kerangka kapal (haluan, di atas dek)


def _visual_model(name, geom, rgba, emissive=None):
    em = f"<emissive>{emissive}</emissive>" if emissive else ""
    return (f"<?xml version='1.0'?><sdf version='1.9'><model name='{name}'><static>true</static>"
            f"<link name='link'><visual name='v'><geometry>{geom}</geometry>"
            f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>{em}</material>"
            f"<transparency>0.25</transparency><cast_shadows>false</cast_shadows></visual></link></model></sdf>")


class AuxEffects:
    def __init__(self, gz_node, logger):
        self.n, self.log = gz_node, logger

    def _spawn(self, sdf, xyz, quat=(0, 0, 0, 1)):
        req = ef.EntityFactory()
        req.sdf = sdf
        req.allow_renaming = False
        req.pose.position.x, req.pose.position.y, req.pose.position.z = xyz
        (req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z,
         req.pose.orientation.w) = quat
        ok, rep = self.n.request(f"/world/{C.WORLD}/create", req, ef.EntityFactory, bb.Boolean, 3000)
        return ok and rep.data

    def remove(self, name):
        e = ep.Entity()
        e.name, e.type = name, ep.Entity.MODEL
        self.n.request(f"/world/{C.WORLD}/remove", e, ep.Entity, bb.Boolean, 3000)

    def buzzer_marker(self, boat_pose):
        x, y, z = boat_pose[0], boat_pose[1], boat_pose[2]
        sdf = _visual_model("buzzer_marker", "<sphere><radius>0.45</radius></sphere>", "1 0.05 0.05 1",
                            emissive="1 0 0 1")
        return self._spawn(sdf, (x, y, z + 4.6))

    def water_jet(self, boat_pose, target_xyz):
        """Silinder biru tipis dari nosel ke target (banner)."""
        yaw = math.atan2(2 * (boat_pose[6] * boat_pose[5] + boat_pose[3] * boat_pose[4]),
                         1 - 2 * (boat_pose[4] ** 2 + boat_pose[5] ** 2))
        c, s = math.cos(yaw), math.sin(yaw)
        nx = boat_pose[0] + c * NOZZLE_BOAT[0] - s * NOZZLE_BOAT[1]
        ny = boat_pose[1] + s * NOZZLE_BOAT[0] + c * NOZZLE_BOAT[1]
        nz = boat_pose[2] + NOZZLE_BOAT[2]
        p0 = np.array([nx, ny, nz])
        p1 = np.array(target_xyz, dtype=float)
        d = p1 - p0
        L = float(np.linalg.norm(d))
        if L < 0.5:
            return False
        z = d / L                                   # sumbu silinder (z lokal) -> arah semburan
        x = np.cross([0, 0, 1], z)
        if np.linalg.norm(x) < 1e-6:
            x = np.array([1.0, 0, 0])
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        T = np.eye(4)
        T[:3, :3] = np.column_stack([x, y, z])
        q = T_to_quat(T)
        mid = (p0 + p1) / 2
        sdf = _visual_model("water_jet", f"<cylinder><radius>0.12</radius><length>{L:.3f}</length></cylinder>",
                            "0.2 0.55 1 0.7", emissive="0.1 0.3 0.8 1")
        ok = self._spawn(sdf, tuple(mid), q)
        splash = _visual_model("water_splash", "<sphere><radius>0.7</radius></sphere>", "0.3 0.6 1 0.6")
        self._spawn(splash, tuple(p1))
        return ok

    def clear(self):
        for n in ("water_jet", "water_splash", "buzzer_marker"):
            self.remove(n)
