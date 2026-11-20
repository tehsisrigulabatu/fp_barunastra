#!/usr/bin/env python3
"""Generator world Gazebo arena misi Pengiriman Kapal Otonom.

Geometri diambil dari figs.CFG (sama dengan gambar guidebook). Origin = pusat kolam, x ke Spot 2, y ke kiri.
Pakai:  python3 make_world.py [--scale 1.0] [--vrx-world PATH_follow_path_task.sdf] [--out nama.sdf]
Skala 5.0 dipakai bila ingin memakai WAM-V bawaan VRX (panjang sekitar 5 m).
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from figs import CFG, ITEM_SPOTS, layout

WORLD_NAME = "pengiriman_kapal_otonom"
ITEM_MASS = 0.05   # kg, massa ABSOLUT barang (tidak ikut dikali skala^3)
RED, GRN, YEL, BLK, WHT, WOOD = (0.82, 0.2, 0.17), (0.18, 0.62, 0.31), (0.95, 0.76, 0.19), (0.1, 0.1, 0.1), (0.95, 0.95, 0.95), (0.55, 0.43, 0.31)


def col(c, a=1.0):
    return f"{c[0]} {c[1]} {c[2]} {a}"


class W:
    def __init__(self, s):
        self.s = s
        self.out = []

    def f(self, v):
        return f"{v * self.s:.4f}"

    def model(self, name, x, y, z, shapes, static=True, mass=None, yaw=0.0, collide=True, abs_mass=False):
        """shapes: list of (kind, dims, (dx,dy,dz), color). Ukuran dalam meter skala 1."""
        s = self
        o = [f'    <model name="{name}">', f'      <static>{"true" if static else "false"}</static>',
             f'      <pose>{s.f(x)} {s.f(y)} {s.f(z)} 0 0 {yaw}</pose>', '      <link name="link">']
        if mass is not None:
            m = mass if abs_mass else mass * s.s ** 3
            ii = m * 0.00375 * s.s ** 2   # kubus sisi 0,15 m x skala: I = m L^2 / 6
            o.append(f'        <inertial><mass>{m:.4f}</mass><inertia><ixx>{ii:.6f}</ixx><iyy>{ii:.6f}</iyy><izz>{ii:.6f}</izz></inertia></inertial>')
        for i, sh in enumerate(shapes):
            kind, d, off, c = sh[:4]
            rot = sh[4] if len(sh) > 4 else (0, 0, 0)
            if kind == "box":
                g = f'<box><size>{s.f(d[0])} {s.f(d[1])} {s.f(d[2])}</size></box>'
            elif kind == "sph":
                g = f'<sphere><radius>{s.f(d[0])}</radius></sphere>'
            elif kind == "prism":  # penampang segitiga (alas d[0], tinggi d[1]) di bidang XY, panjang ekstrusi d[2]
                b, h, ln = d
                g = (f'<polyline><point>{s.f(-b/2)} 0</point><point>{s.f(b/2)} 0</point><point>0 {s.f(h)}</point>'
                     f'<height>{s.f(ln)}</height></polyline>')
            else:
                g = f'<cylinder><radius>{s.f(d[0])}</radius><length>{s.f(d[1])}</length></cylinder>'
            p = f'<pose>{s.f(off[0])} {s.f(off[1])} {s.f(off[2])} {rot[0]} {rot[1]} {rot[2]}</pose>'
            o.append(f'        <visual name="v{i}">{p}<geometry>{g}</geometry><material><ambient>{col(c)}</ambient><diffuse>{col(c)}</diffuse></material></visual>')
            if collide:
                o.append(f'        <collision name="c{i}">{p}<geometry>{g}</geometry></collision>')
        o += ['      </link>', '    </model>']
        self.out += o


def build(scale, vrx_world):
    c = CFG
    Wd, Hd = c["W"], c["H"]
    dw, dh = c["dock_w"], c["dock_h"]
    ox, oy = Wd / 2, Hd / 2       # geser: origin ke pusat kolam
    w = W(scale)
    DOCK_T, DOCK_TOP = 0.4, 0.25  # tebal dermaga, tinggi permukaan di atas air
    dz = DOCK_TOP - DOCK_T / 2

    sd, sw = c["slot_d"], c["slot_w"]
    side = (dh - sw) / 2
    # Spot 1 (barat) dan Spot 2 (timur): dermaga dengan slot (Spot 1 = slot start, Spot 2 = slot docking), sisi air menghadap tengah kolam
    for tag, sgn in (("spot1", -1), ("spot2", +1)):
        xb = sgn * (ox - (dw - sd) / 2)                 # blok belakang
        w.model(f"{tag}_dock_back", xb, 0, dz, [("box", (dw - sd, dh, DOCK_T), (0, 0, 0), WOOD)])
        xs_ = sgn * (ox - dw + sd / 2)                  # dua blok samping slot
        for sg2, nm in ((1, "left"), (-1, "right")):
            w.model(f"{tag}_dock_{nm}", xs_, sg2 * (sw / 2 + side / 2), dz, [("box", (sd, side, DOCK_T), (0, 0, 0), WOOD)])
    # Banner: Spot 1 = silang hitam, Spot 2 = segitiga hitam (sekaligus target semprotan air), menempel di dinding belakang
    bz = DOCK_TOP + 0.45
    w.model("spot1_banner", -ox + 0.08, 0, bz, [("box", (0.04, 1.0, 0.8), (0, 0, 0), WHT), ("box", (0.05, 0.12, 0.6), (0.02, 0, 0), BLK), ("box", (0.05, 0.6, 0.12), (0.02, 0, 0), BLK)], collide=False)
    w.model("spot2_banner", ox - 0.08, 0, bz, [("box", (0.04, 1.0, 0.8), (0, 0, 0), WHT), ("box", (0.05, 0.6, 0.04), (-0.02, 0, -0.25), BLK),
                                                ("box", (0.05, 0.04, 0.583), (-0.02, -0.15, 0.0), BLK, (-0.5404, 0, 0)),
                                                ("box", (0.05, 0.04, 0.583), (-0.02, 0.15, 0.0), BLK, (0.5404, 0, 0))], collide=False)
    # Penanda posisi start ASV di dalam slot Spot 1 (visual saja)
    w.model("start_marker", -ox + dw - sd / 2, 0, 0.012, [("box", (sd - 0.2, sw - 0.2, 0.01), (0, 0, 0), (0.75, 0.85, 0.95))], collide=False)

    # Barang di Spot 1 (dinamis, bisa diambil): kubus, bola, prisma segitiga. Spot 2 = cermin (penanda zona taruh)
    it = c["item"]
    for kind, ix, iy in ITEM_SPOTS:
        X, Y = ix - ox, iy
        if kind == "cube":
            w.model("item_cube", X, Y, DOCK_TOP + it / 2, [("box", (it, it, it), (0, 0, 0), YEL)], static=False, mass=ITEM_MASS, abs_mass=True)
        elif kind == "sphere":
            w.model("item_sphere", X, Y, DOCK_TOP + it / 2, [("sph", (it / 2,), (0, 0, 0), YEL)], static=False, mass=ITEM_MASS, abs_mass=True)
        else:
            w.model("item_prism", X, Y, DOCK_TOP + it / 2, [("prism", (it, it, it), (0, it / 2, -it / 2), YEL, (1.5708, 0, 0))], static=False, mass=ITEM_MASS, abs_mass=True)
        w.model(f"drop_mark_{kind}", ox - ix, Y, DOCK_TOP + 0.005, [("box", (it * 1.6, it * 1.6, 0.01), (0, 0, 0), WHT)], collide=False)

    # Jalur FTP: baris hijau (sisi kiri) dan merah (sisi kanan). Gerbang = pasangan sempit, penanda jalur = pasangan lebih lebar
    L = layout()
    gate_xs = {g[0] for g in L["gates"]}
    for k, ((gx, gy_), (rx, ry)) in enumerate(zip(L["green"], L["red"])):
        tag = "gate" if gx in gate_xs else "lane"
        w.model(f"{tag}{k+1}_green", gx - ox, gy_ - oy, c["buoy_r"] * 0.7, [("sph", (c["buoy_r"],), (0, 0, 0), GRN)])
        w.model(f"{tag}{k+1}_red", rx - ox, ry - oy, c["buoy_r"] * 0.7, [("sph", (c["buoy_r"],), (0, 0, 0), RED)])
    # Rintangan di dalam jalur
    for k, (x, y, cc, _) in enumerate(L["obs"]):
        col_ = YEL if cc == "Y" else BLK
        w.model(f"obstacle{k+1}_{'yellow' if cc == 'Y' else 'black'}", x - ox, y - oy, c["obs_r"] * 0.7, [("sph", (c["obs_r"],), (0, 0, 0), col_)])

    # Garis tepi kolam (visual saja)
    for nm, x, y, sx, sy in (("edge_n", 0, Hd / 2, Wd, 0.1), ("edge_s", 0, -Hd / 2, Wd, 0.1), ("edge_w", -Wd / 2, 0, 0.1, Hd), ("edge_e", Wd / 2, 0, 0.1, Hd)):
        w.model(nm, x, y, 0.02, [("box", (sx, sy, 0.03), (0, 0, 0), WHT)], collide=False)

    # Kamera atas (untuk Vision, dan untuk screenshot)
    cam_h = 20.0 * scale
    cam = f'''    <model name="overview_camera">
      <static>true</static>
      <pose>0 0 {cam_h:.3f} 0 1.5708 1.5708</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <camera><horizontal_fov>1.2</horizontal_fov><image><width>1280</width><height>720</height></image><clip><near>0.1</near><far>100</far></clip></camera>
          <always_on>1</always_on><update_rate>10</update_rate><visualize>false</visualize>
          <topic>overview_camera/image</topic>
        </sensor>
      </link>
    </model>'''

    lines = open(vrx_world).read().split("\n")
    end_gui = next(i for i, l in enumerate(lines) if l.strip() == "</gui>")
    head = "\n".join(lines[: end_gui + 1])
    head = head.replace('world name="follow_path_task"', f'world name="{WORLD_NAME}"')
    import re
    head = re.sub(r"<camera_pose>.*?</camera_pose>", f"<camera_pose>{-1.0*Wd*scale*0.35:.2f} {-Hd*scale*0.75:.2f} {10*scale:.2f} 0 0.62 1.1</camera_pose>", head)
    head = head.replace("<start_paused>true</start_paused>", "<start_paused>false</start_paused>")

    sys_start = next(i for i, l in enumerate(lines) if 'name="gz::sim::systems::Physics"' in l) - 2
    sys_end = next(i for i, l in enumerate(lines) if l.strip() == "<scene>")
    systems = "\n".join(lines[sys_start: sys_end])
    scene_light_start = sys_end
    light_end = next(i for i, l in enumerate(lines) if "</light>" in l)
    scene_light = "\n".join(lines[scene_light_start: light_end + 1])

    waves = '''    <include>
      <name>Coast Waves</name>
      <pose>0 0 0 0 0 0</pose>
      <uri>coast_waves</uri>
    </include>

    <plugin filename="libPublisherPlugin.so" name="vrx::PublisherPlugin">
      <message type="gz.msgs.Param" topic="/vrx/wavefield/parameters" every="2.0">
        params { key: "direction" value { type: DOUBLE double_value: 0.0 } }
        params { key: "gain" value { type: DOUBLE double_value: 0.0 } }
        params { key: "period" value { type: DOUBLE double_value: 5 } }
        params { key: "steepness" value { type: DOUBLE double_value: 0 } }
      </message>
    </plugin>'''

    return "\n".join([head, "", systems, scene_light, "", waves, "", "    <!-- ===== ARENA (dihasilkan make_world.py, skala %.2f) ===== -->" % scale,
                      "\n".join(w.out), cam, "", "  </world>", "</sdf>", ""])


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--vrx-world", default=os.path.expanduser("~/vrx_ws/src/vrx/vrx_gz/worlds/follow_path_task.sdf"))
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    out = a.out or os.path.join(os.path.dirname(os.path.abspath(__file__)), f"{WORLD_NAME}{'' if a.scale == 1 else '_x%g' % a.scale}.sdf")
    open(out, "w").write(build(a.scale, a.vrx_world))
    print(out)
