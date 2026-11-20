#!/usr/bin/env python3
"""PANDUAN: mengendalikan mekanisme pengambil dan menyempurnakan misi di VRX / Gazebo.

File ini sengaja hanya berisi panduan. Kodenya kamu tulis sendiri (di file lain atau di sini).
Baca dulu make_arm.py untuk cara membuat modelnya.

======================================================================
CORE: dari model muncul sampai satu barang berhasil dipindah
======================================================================

LANGKAH 1. Siapkan alat bantu yang selalu dipakai
  - Environment:  source /opt/ros/jazzy/setup.bash ; source ~/vrx_ws/install/setup.bash
  - Dua cara bicara ke Gazebo:
      a) gz transport lewat python:  import gz.transport13 as gzt ; node = gzt.Node()
         pesan: gz.msgs10.double_pb2 (perintah sendi), pose_v_pb2 (pose), model_pb2 (joint state),
         entity_factory_pb2 dan entity_pb2 (spawn dan hapus), empty_pb2 (pemicu).
      b) ROS 2 lewat ros_gz_bridge:
         ros2 run ros_gz_bridge parameter_bridge /topik@std_msgs/msg/Float64]gz.msgs.Double
         (tanda ] = ROS ke Gazebo, [ = Gazebo ke ROS, @ dua arah dengan tanda yang sesuai)
  - Cek apa yang ada:  gz topic -l ;  gz service -l ;  gz topic -e -t <topik>

LANGKAH 2. Satu sendi, satu perintah
  Publish nilai target ke topik controller sendi (lihat langkah 6 di make_arm.py), amati hasilnya
  di joint_state. Tulis fungsi kecil: gerak(sendi, nilai) lalu tunggu_sampai(sendi, nilai, toleransi).
  Jangan lanjut sebelum tiap sendi bisa digerakkan ke posisi yang kamu mau, dengan galat kecil.

LANGKAH 3. Ketahui di mana segala sesuatu berada
  - Pose semua benda:  gz topic -e -t /world/pengiriman_kapal_otonom/dynamic_pose/info
    (berisi nama, posisi, orientasi; pose link dilaporkan relatif terhadap modelnya)
  - Posisi kapal juga dari /tf di ROS 2 setelah bridge.
  - Ubah dari kerangka kapal ke dunia dengan rotasi yaw kapal: yaw = 2 * atan2(qz, qw),
    lalu x_dunia = x_k + cos(yaw) * dx - sin(yaw) * dy (dan y_dunia serupa).
  - Koordinat arena ada di README (origin di pusat kolam).

LANGKAH 4. Inverse kinematics yang sederhana
  Untuk mekanisme berbasis putar + julur + naik (silindris): sudut = atan2(dy, dx) dari sumbu
  putar, julur = jarak datar, naik = beda tinggi. Hitung di kerangka mekanisme, bukan dunia.
  Lebih rumit? Tulis rumus maju (fk) dulu, uji, lalu balik pakai solusi analitik atau numerik.

LANGKAH 5. Urutan satu pengambilan (rancang sebagai state machine)
  Pindah ke atas barang -> turun -> pegang -> naik -> pindah ke atas zona taruh -> turun -> lepas -> naik.
  Tiap state: aksi, kondisi selesai (posisi sendi mencapai target, bukan sekadar sleep), timeout,
  dan apa yang dilakukan kalau gagal (misalnya ulangi atau lewati).

LANGKAH 6. Satukan dengan navigasi kapal
  Kapal harus berhenti di titik yang membuat lengan sampai. Sebelum mengambil, cek jarak
  kapal ke target, kecepatan mendekati nol, dan yaw sesuai. Pertahankan posisi dengan thruster
  selama lengan bergerak (reaksi lengan menggoyang kapal).

======================================================================
ADVANCED: supaya misi lebih andal dan skornya naik
======================================================================
  1. Gunakan deteksi untuk memastikan posisi barang dan zona, jangan hanya koordinat tetap.
     Kamera gripper dibridge ke ROS 2, olah dengan OpenCV (HSV untuk warna, kontur untuk bentuk).
  2. Bedakan kubus, bola, prisma dari rasio kontur dan jumlah sudut hasil approxPolyDP.
  3. Kontrol kapal: PID untuk heading dan jarak, batasi kecepatan dekat dermaga, anti-windup
     pada i_gain. Logging error tiap loop supaya mudah di-tuning.
  4. Perencanaan jalur: gerbang buoy hijau-merah sebagai waypoint tengah-tengah, hindari
     rintangan bola dengan jarak aman (hitung dari radius + lebar kapal).
  5. Mainkan skenario berulang tanpa GUI untuk cepat uji:  ./jalankan_arena.sh x5 -s -r
     lalu analisis log. Satu misi penuh sebaiknya bisa dijalankan satu perintah dan menghasilkan
     file log (waktu, jarak minimum ke penghalang, jumlah kontak, hasil taruh).
  6. Buat metrik sendiri: waktu total, jumlah kontak, jarak aman minimum, galat posisi taruh.
     Ubah satu hal tiap percobaan agar tahu mana yang memperbaiki.
  7. Kuatkan terhadap gangguan: tambah arus atau angin di world VRX, geser titik berhenti,
     lalu lihat apakah misi tetap selesai.
  8. Jaga kebersihan proses sebelum tiap uji:
       pgrep -fa "gz sim|ros2|parameter_bridge|optical_frame"
     Proses sisa dari uji sebelumnya adalah penyebab paling sering hasil yang aneh.
  9. Disk: log RViz bisa membengkak. Jalankan RViz dengan --log-level error dan hapus
     ~/.ros/log berkala.
 10. Pisahkan kode: modul kendali sendi, modul persepsi, modul navigasi, modul misi (state machine).
     Tiap modul diuji sendiri sebelum digabung.

======================================================================
DISRUPTIVE: hanya petunjuk
======================================================================
  - Bisakah misi diselesaikan tanpa mengambil barang secara terpisah satu per satu?
  - Apa yang terjadi kalau urutan pengambilan ditentukan oleh posisi, bukan oleh jenis barang?
  - Bisakah kapal dan mekanisme dikendalikan sebagai satu sistem, bukan dua loop terpisah?
  - Bagaimana mendeteksi sendiri bahwa pengambilan gagal (tanpa sensor khusus), lalu memulihkannya?
  - Kalau Gazebo tidak ada, bagian mana dari logikamu yang tetap bisa diuji di komputer biasa?
"""

#!/usr/bin/env python3
"""Pasang lengan pengambil ke WAM-V yang sudah ada di arena skala 5, dan fungsi kendali sederhananya.
Pakai:  python3 lengan.py pasang     (dipanggil jalankan_wamv.sh setelah kapal muncul)
Perlu: Gazebo berjalan (world pengiriman_kapal_otonom_x5.sdf) dan WAM-V bernama 'wamv'."""
import math, os, sys, time
import gz.transport13 as gzt
import gz.msgs10.pose_v_pb2 as pvb
import gz.msgs10.entity_factory_pb2 as ef
import gz.msgs10.boolean_pb2 as bb
import gz.msgs10.double_pb2 as db
import gz.msgs10.empty_pb2 as em
import gz.msgs10.model_pb2 as mdl
import gz.msgs10.entity_pb2 as ep
import make_world as MW
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_arm as MA

WORLD = "pengiriman_kapal_otonom"


def item_sdf(k):
    """SDF barang (sama dengan di make_world.py, skala 5), pose diberikan lewat EntityFactory."""
    w = MW.W(5.0)
    it = MW.CFG["item"]
    YEL = MW.YEL
    if k == "cube":
        w.model("item_cube", 0, 0, 0, [("box", (it, it, it), (0, 0, 0), YEL)], static=False, mass=MW.ITEM_MASS, abs_mass=True)
    elif k == "sphere":
        w.model("item_sphere", 0, 0, 0, [("sph", (it / 2,), (0, 0, 0), YEL)], static=False, mass=MW.ITEM_MASS, abs_mass=True)
    else:
        w.model("item_prism", 0, 0, 0, [("prism", (it, it, it), (0, it / 2, -it / 2), YEL, (1.5708, 0, 0))], static=False, mass=MW.ITEM_MASS, abs_mass=True)
    return "<?xml version='1.0'?><sdf version='1.9'>" + "\n".join(w.out) + "</sdf>"


class Arm:
    def __init__(self):
        self.n = gzt.Node()
        self.pubs = {t: self.n.advertise(f"/arm/{t}", db.Double) for t in ("yaw", "slide", "lift")}
        self.det = {k: self.n.advertise(f"/arm/detach_{k}", em.Empty) for k in MA.ITEMS}
        self.poses = {}
        self.joints = {}
        self.n.subscribe(pvb.Pose_V, f"/world/{WORLD}/dynamic_pose/info", self._pose)
        self.n.subscribe(mdl.Model, "/arm/joint_state", self._js)

    def _pose(self, m):
        for p in m.pose:
            self.poses[p.name] = (p.position.x, p.position.y, p.position.z, p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def _js(self, m):
        for j in m.joint:
            self.joints[j.name] = j.axis1.position if j.HasField("axis1") else 0.0

    def cmd(self, yaw=None, slide=None, lift=None):
        for k, v in (("yaw", yaw), ("slide", slide), ("lift", lift)):
            if v is not None:
                m = db.Double(); m.data = float(v); self.pubs[k].publish(m)

    def grab(self, k, offset=(0.0, 0.0, -0.42)):
        """Pegang barang k: barang dibuat ulang tepat di bawah gripper dengan sambungan kaku (DetachableJoint, anak = gripper).
        Gazebo 8 tidak punya topik attach, sambungan terbentuk saat model dimuat."""
        name = f"item_{k}"
        it, gp, ap = self.poses.get(name), self.poses.get("gripper"), self.poses.get("asv_arm")
        # pose gripper di dunia = pose model lengan (dynamic_pose melaporkan link relatif ke model)
        gx, gy, gz = self.gripper_world()
        q = it[3:7] if it else (0, 0, 0, 1)
        sdf = item_sdf(k)
        plug = ('<plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">'
                '<parent_link>link</parent_link><child_model>asv_arm</child_model><child_link>gripper</child_link>'
                f'<detach_topic>/arm/detach_{k}</detach_topic></plugin>')
        sdf = sdf.replace("</model>", plug + "</model>")
        rm = ep.Entity(); rm.name = name; rm.type = ep.Entity.MODEL
        self.n.request(f"/world/{WORLD}/remove", rm, ep.Entity, bb.Boolean, 3000)
        time.sleep(0.3)
        req = ef.EntityFactory(); req.sdf = sdf
        req.pose.position.x, req.pose.position.y, req.pose.position.z = gx + offset[0], gy + offset[1], gz + offset[2]
        req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w = q
        ok, rep = self.n.request(f"/world/{WORLD}/create", req, ef.EntityFactory, bb.Boolean, 5000)
        return ok and rep.data

    def gripper_world(self):
        """Posisi gripper di dunia (model lengan berimpit dengan base_link WAM-V; link dilaporkan relatif ke model lengan)."""
        ap = self.poses["wamv"]; g = self.poses["gripper"]
        x, y, z, qx, qy, qz, qw = ap
        yaw = 2 * math.atan2(qz, qw)
        c, s = math.cos(yaw), math.sin(yaw)
        return x + c * g[0] - s * g[1], y + s * g[0] + c * g[1], z + g[2]

    def detach(self, k):
        self.det[k].publish(em.Empty())


def wait_pose(arm, name="wamv", t=10):
    t0 = time.time()
    while name not in arm.poses and time.time() - t0 < t:
        time.sleep(0.1)
    return arm.poses.get(name)


def pasang():
    arm = Arm()
    p = wait_pose(arm)
    if p is None:
        sys.exit("WAM-V tidak ditemukan di world.")
    # tunggu kapal benar-benar tenang (jatuh 0,3 m dari spawn lalu mengambang): z stabil dan tidak berubah selama 4 detik
    t0, last, since = time.time(), None, time.time()
    while time.time() - t0 < 60:
        z = arm.poses["wamv"][2]
        if last is None or abs(z - last) > 0.004:
            since = time.time()
        last = z
        if time.time() - since > 4:
            break
        time.sleep(0.25)
    p = arm.poses["wamv"]
    req = ef.EntityFactory()
    req.sdf = MA.build()
    req.pose.position.x, req.pose.position.y, req.pose.position.z = p[0], p[1], p[2]
    req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w = p[3:7]
    ok, rep = arm.n.request(f"/world/{WORLD}/create", req, ef.EntityFactory, bb.Boolean, 20000)
    print("lengan dibuat:", ok, rep.data if ok else "")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "pasang":
        pasang()
