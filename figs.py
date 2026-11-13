"""Dimensi arena (satuan meter) dan posisi buoy. Dipakai make_world.py dan lengan.py."""
import numpy as np

CFG = dict(
    W=24.0, H=12.0,           # kolam (D): diperpanjang dari 18 x 10 m, lihat catatan dasar ukuran di guidebook
    dock_w=3.0, dock_h=4.0,   # ukuran spot (D)
    slot_d=1.5, slot_w=1.2,   # slot start (Spot 1) dan slot docking (Spot 2): kedalaman x lebar (D)
    item=0.15,                # sisi/diameter barang (D)
    buoy_r=0.15, obs_r=0.22,
    asv_L=0.99, asv_B=0.60,   # batas panjang dari MEC, lebar perkiraan 0,5 sampai 0,6 m
    # pasangan buoy jalur: (x, tengah y, lebar, jenis). Jenis gate = gerbang sempit, lane = penanda jalur lebih lebar
    lanes=[(6.0, 6.6, 2.0, "gate"), (9.0, 6.0, 3.5, "lane"), (12.0, 5.4, 1.5, "gate"), (15.0, 6.0, 3.5, "lane"), (18.0, 6.6, 2.0, "gate")],
    # rintangan di dalam jalur: (x, geser dari tengah jalur, warna). Geser + = ke sisi hijau (kiri jalur)
    obstacles=[(7.5, +0.5, "Y"), (10.5, -0.5, "K"), (13.5, +0.5, "K"), (16.5, -0.5, "Y")],
)
# barang di Spot 1 (x, y relatif ke tengah kolam y=H/2) dan bentuknya. Spot 2 = cermin (zona taruh)
ITEM_SPOTS = [("cube", 0.9, 0.0), ("sphere", 2.2, +1.0), ("prism", 2.2, -1.0)]
SHAPE_NAME = {"cube": "Cube", "sphere": "Sphere", "prism": "Triangular prism"}


def layout():
    """Posisi semua buoy dalam meter (origin pojok barat daya kolam). Dipakai gambar dan make_world.py."""
    from scipy.interpolate import PchipInterpolator
    c = CFG
    xs = [l[0] for l in c["lanes"]]
    cy = PchipInterpolator(xs, [l[1] for l in c["lanes"]])
    hw = lambda x: float(np.interp(x, xs, [l[2] / 2 for l in c["lanes"]]))
    green, red, gates = [], [], []
    for x, y, w, kind in c["lanes"]:
        green.append((x, y + w / 2)); red.append((x, y - w / 2))
        if kind == "gate":
            gates.append((x, y, w))
    obs = []
    for x, off, cc in c["obstacles"]:
        obs.append((x, float(cy(x)) + off, cc, hw(x)))
    return dict(green=green, red=red, gates=gates, obs=obs, cy=cy, hw=hw)
