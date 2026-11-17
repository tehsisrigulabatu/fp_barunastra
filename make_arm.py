#!/usr/bin/env python3
"""PANDUAN: membuat model 3D (mekanisme pengambil) untuk WAM-V di VRX / Gazebo.

File ini sengaja hanya berisi panduan. Mekanismenya kamu rancang sendiri.
Kerjakan berurutan, uji tiap langkah sebelum lanjut. Pasangannya ada di lengan.py (kendali dan misi).

======================================================================
CORE: dari nol sampai model muncul di Gazebo
======================================================================

LANGKAH 1. Putuskan dulu mekanismenya di kertas
  - Barang: kubus, bola, prisma segitiga (lihat figs.py dan README untuk ukuran, skala 5).
  - Tentukan sendi apa saja: berapa derajat kebebasan, mana yang berputar (revolute),
    mana yang meluncur (prismatic), dan batas geraknya (lower, upper).
  - Hitung jangkauan: dari tempat model akan dipasang di kapal sampai (a) dudukan barang dan
    (b) zona taruh di dermaga seberang. Lengan yang tidak sampai = desain gagal sebelum dibuat.
  - Gambar daftar link (bagian kaku) dan joint (sambungan) beserta induk dan anaknya.

LANGKAH 2. Tulis SDF kosong dan cek sintaksnya
  - Format: <sdf version="1.9"><model name="..."> ... </model></sdf>
  - Cek: gz sdf -k model.sdf        (harus keluar "Valid")
  - Lihat struktur: gz sdf -p model.sdf
  - Tulis SDF lewat python (f-string) supaya ukuran bisa diparameterkan, atau tulis tangan lalu
    pindahkan angka ke konstanta di atas file.

LANGKAH 3. Buat satu link dulu: <link name="...">, jangan semuanya sekaligus
  Tiap link butuh tiga hal:
    <inertial>  massa dan inersia. Jangan nol dan jangan terlalu kecil: inersia terlalu kecil
                membuat simulasi meledak atau bergetar. Patokan kotak: Ixx = m (b^2 + c^2) / 12.
    <visual>    bentuk yang tampak (box, cylinder, sphere, mesh). Boleh asal dulu.
    <collision> bentuk yang menabrak. Samakan dengan visual atau lebih sederhana.
  Spawn dan lihat hasilnya (langkah 5) sebelum menambah link berikutnya.

LANGKAH 4. Tambah link lain dan sambungkan dengan <joint>
  - <joint name="..." type="revolute|prismatic"><parent>A</parent><child>B</child>
      <axis><xyz>0 0 1</xyz><limit><lower>..</lower><upper>..</upper></limit>
            <dynamics><damping>..</damping></dynamics></axis></joint>
  - Pose link ditulis di kerangka model. Pastikan pose awal link anak sudah pas dengan posisi
    sendi (kalau tidak, begitu simulasi mulai link melompat).
  - Beri damping di sendi supaya tidak berayun terus.

LANGKAH 5. Masukkan ke dunia yang sedang berjalan (tanpa restart)
  Jalankan arena dulu:  ./jalankan_wamv.sh
  Lalu spawn:
    gz service -s /world/pengiriman_kapal_otonom/create \
      --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 5000 \
      --req 'sdf_filename: "/path/ke/model.sdf", pose: {position: {x: -48.5, y: 0, z: 2}}'
  Hapus lagi untuk coba ulang:
    gz service -s /world/pengiriman_kapal_otonom/remove \
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 3000 \
      --req 'name: "nama_model" type: MODEL'
  Dari python: gz.transport13 dengan pesan EntityFactory (lihat pola di dokumentasi gz-transport).
  Sebelum spawn ulang, cek proses ganda:  pgrep -fa "gz sim|ros2|parameter_bridge"

LANGKAH 6. Gerakkan sendinya (controller)
  Tambah plugin per sendi di dalam <model>:
    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>NAMA_JOINT</joint_name><topic>/nama/topik</topic>
      <p_gain>..</p_gain><i_gain>..</i_gain><d_gain>..</d_gain>
      <cmd_max>..</cmd_max><cmd_min>..</cmd_min></plugin>
  Kirim perintah:
    gz topic -t /nama/topik -m gz.msgs.Double -p 'data: 0.5'
  Tuning: mulai dari p_gain kecil, naikkan sampai respons cukup cepat, lalu tambah d_gain
  sampai tidak berosilasi, i_gain terakhir dan kecil (hanya untuk menahan beban).
  Baca posisi sendi dengan plugin gz-sim-joint-state-publisher-system lalu
    gz topic -e -t /nama/joint_state

LANGKAH 7. Pasang ke kapal
  WAM-V yang sudah di-spawn tidak bisa diedit SDF-nya. Ada dua jalan:
    a) Model terpisah yang disambung kaku ke lambung dengan plugin
       gz-sim-detachable-joint-system (parent_link = link model kamu,
       child_model = wamv, child_link = wamv/base_link). Spawn model tepat di pose kapal.
    b) Menambah komponen lewat parameter spawn VRX (lihat vrx_gz spawn.launch.py dan wamv_gazebo).
  Gunakan jalan (a) kalau mau cepat. Tunggu kapal mengambang tenang dulu sebelum spawn.

LANGKAH 8. Pegang dan lepas barang
  Pegang = membuat sambungan kaku sementara antara gripper dan barang. Pelajari
  gz-sim-detachable-joint-system: ia punya <detach_topic>, tapi sambungannya terbentuk saat
  model dimuat, jadi di Gazebo 8 barang biasanya dibuat ulang di posisi gripper bersama pluginnya.
  Alternatif lain yang perlu kamu uji sendiri: gaya gesek dan jepitan fisik dari collision jari.

======================================================================
ADVANCED: supaya model tidak merusak simulasi
======================================================================
  1. Kapal WAM-V punya benda yang bertabrakan: baterai, balok busur, kamera, lidar, cpu case.
     Cek posisinya di SDF WAM-V (gz sdf -p) dan rancang dek, dinding, dan lengan yang
     menghindarinya. Kontak tak sengaja dengan itu membuat kapal terlempar.
  2. Reaksi gerak lengan menggoyang kapal yang mengambang bebas. Tahan dengan thruster
     atau gerakkan lengan pelan (batasi kecepatan perintah, beri ramp).
  3. Berat lengan memindahkan titik apung. Keluarkan massa yang tidak perlu, dan letakkan
     massa serendah dan sedekat sumbu tengah kapal mungkin.
  4. Hindari tinggi lengan yang menutupi lidar atau kamera kapal. Cek dengan visualisasi sensor.
  5. Gunakan collision sederhana (box, cylinder). Mesh rumit memperlambat dan sering salah kontak.
  6. Jaga real-time factor. Kalau turun jauh, kurangi jumlah link dan sensor kamera (resolusi, Hz).
  7. Kamera di gripper: pasang <sensor type="camera"> di link gripper dengan <topic>,
     jembatani ke ROS 2 dengan
       ros2 run ros_gz_bridge parameter_bridge /topik@sensor_msgs/msg/Image[gz.msgs.Image
  8. Simpan parameter ukuran (panjang boom, batas sendi, posisi dudukan) sebagai konstanta
     di bagian atas file, jangan angka ajaib di tengah string SDF.

======================================================================
DISRUPTIVE: hanya petunjuk
======================================================================
  - Pikirkan: apakah satu mekanisme harus mengambil tiga bentuk barang berbeda? Apa yang berubah
    kalau pegangannya bukan penjepit?
  - Pikirkan sumber gaya selain motor sendi (misalnya yang bekerja lewat fluida, tekanan, atau medan).
    Bagaimana Gazebo bisa meniru gaya itu tanpa sendi sama sekali?
  - Pikirkan berapa banyak bagian yang bergerak. Lebih sedikit sendi berarti lebih sedikit titik gagal.
  - Pikirkan apa yang terjadi kalau kapal bergeser 0,3 m dari titik berhenti yang direncanakan.
    Mekanisme seperti apa yang tetap berhasil?
"""

#!/usr/bin/env python3
"""Membuat SDF lengan pengambil (crane demo) untuk WAM-V skala 5. Dipasang kaku ke lambung lewat DetachableJoint.
Sumbu: yaw (putar), slide (julur), lift (naik turun). Tiga barang diambil lewat DetachableJoint ke gripper.
Kerangka model = kerangka base_link WAM-V (model dibuat tepat di pose WAM-V)."""
import sys

PIV = (-0.4, 0.0)        # pusat putar di dek (m, kerangka WAM-V)
DECK_Z = 0.425           # pusat pelat dek (tebal 0,03; atas pontoon 0,40, bawah baterai WAM-V 0,445)
BOOM_Z = 3.4           # cukup tinggi agar barang yang diayun lolos dari lidar WAM-V (puncak 1,84 m)
HANG = 0.6               # jarak gripper di bawah boom saat lift = 0
CARR0 = 0.9              # posisi awal carriage di boom (m dari sumbu putar)
ITEMS = ("cube", "sphere", "prism")


def box(n, size, pose, col, collide=False):
    g = f"<box><size>{size[0]} {size[1]} {size[2]}</size></box>"
    p = f"<pose>{pose[0]} {pose[1]} {pose[2]} 0 0 0</pose>"
    s = f'<visual name="{n}">{p}<geometry>{g}</geometry><material><ambient>{col}</ambient><diffuse>{col}</diffuse></material></visual>'
    if collide:
        s += f'<collision name="{n}_c">{p}<geometry>{g}</geometry></collision>'
    return s


def cyl(n, r, l, pose, col):
    return (f'<visual name="{n}"><pose>{pose[0]} {pose[1]} {pose[2]} 0 0 0</pose><geometry><cylinder><radius>{r}</radius><length>{l}</length></cylinder></geometry>'
            f'<material><ambient>{col}</ambient><diffuse>{col}</diffuse></material></visual>')


def inertial(m, com, i=None):
    i = i or m * 0.6
    return f"<inertial><pose>{com[0]} {com[1]} {com[2]} 0 0 0</pose><mass>{m}</mass><inertia><ixx>{i}</ixx><iyy>{i}</iyy><izz>{i}</izz></inertia></inertial>"


def ctrl(joint, topic, p, i, d, cmax):
    return (f'<plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">'
            f"<joint_name>{joint}</joint_name><topic>{topic}</topic><p_gain>{p}</p_gain><i_gain>{i}</i_gain><d_gain>{d}</d_gain>"
            f"<i_max>{cmax*0.3}</i_max><i_min>{-cmax*0.3}</i_min><cmd_max>{cmax}</cmd_max><cmd_min>{-cmax}</cmd_min></plugin>")


def build(name="asv_arm"):
    px, py = PIV
    ORG, DARK, ORN = "0.95 0.55 0.1 1", "0.15 0.17 0.2 1", "0.9 0.75 0.1 1"
    o = [f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><pose>0 0 0 0 0 0</pose>']
    # --- mount: dek + tiang, kaku ke WAM-V
    o.append('<link name="mount">' + inertial(30, (px, 0, 1.0), 30))
    o.append(box("deck", (4.2, 1.9, 0.03), (0.0, 0, DECK_Z), DARK, collide=True))
    # dudukan barang di dek: dinding rendah (0,34 m) supaya bola dan prisma tidak menggelinding saat kapal oleng
    WH, WZ = 0.30, DECK_Z + 0.015 + 0.15
    for nm, x in (("c1", -2.04), ("c2", -0.97), ("s1", 0.98), ("s2", 2.02)):      # dinding melintang: kubus di x -2,04..-0,97; bola dan prisma di x 1,0..2,0 (di luar sensor bawaan WAM-V)
        o.append(box(f"w_{nm}", (0.04, 1.9, WH), (x, 0, WZ), ORN, collide=True))
    o.append(box("w_mid", (1.04, 0.04, WH), (1.5, 0, WZ), ORN, collide=True))     # sekat antara bola dan prisma
    # dinding sisi hanya di zona barang (menghindari baterai WAM-V di |x| < 0,3 dan balok busur di y sekitar 1,0)
    for sy in (0.90, -0.90):
        o.append(box(f"w_side_bow{sy:+.1f}", (1.08, 0.03, WH), (1.5, sy, WZ), ORN, collide=True))
        o.append(box(f"w_side_stern{sy:+.1f}", (1.08, 0.03, WH), (-1.51, sy, WZ), ORN, collide=True))
    o.append(cyl("mast", 0.09, BOOM_Z - DECK_Z, (px, py, (BOOM_Z + DECK_Z) / 2), DARK))
    o.append(cyl("turntable", 0.2, 0.08, (px, py, BOOM_Z - 0.04), ORN))
    o.append('</link>')
    # --- boom (yaw)
    o.append(f'<link name="boom"><pose>{px} {py} {BOOM_Z + 0.12} 0 0 0</pose>' + inertial(10, (1.9, 0, 0), 25))
    o.append(box("boom_beam", (5.0, 0.24, 0.24), (1.9, 0, 0), ORN))
    o.append(box("counterweight", (0.5, 0.5, 0.4), (-0.85, 0, 0), DARK))
    o.append('</link>')
    # --- carriage (slide), lengan teleskop
    o.append(f'<link name="carriage"><pose>{px + CARR0} {py} {BOOM_Z + 0.12} 0 0 0</pose>' + inertial(4, (-1.0, 0, 0), 3))
    o.append(box("tele", (4.0, 0.16, 0.16), (-1.1, 0, 0), DARK))
    o.append(box("head", (0.3, 0.3, 0.3), (0.0, 0, 0), ORN))
    o.append('</link>')
    # --- gripper (lift)
    gz = BOOM_Z + 0.12 - HANG
    o.append(f'<link name="gripper"><pose>{px + CARR0} {py} {gz} 0 0 0</pose>' + inertial(6, (0, 0, 0.2), 1))
    o.append(box("rod", (0.06, 0.06, 1.2), (0, 0, 0.6 + HANG - 0.55), DARK))
    o.append(box("palm", (0.8, 0.8, 0.08), (0, 0, 0.08), ORN))
    for k, (cx, cy) in enumerate(((0.36, 0), (-0.36, 0), (0, 0.36), (0, -0.36))):
        o.append(box(f"finger{k}", (0.06 if cx else 0.5, 0.5 if cx else 0.06, 0.16), (cx, cy, 0.0), DARK))
    # kamera di braket samping gripper, menghadap ke bawah dan sedikit ke tengah: melihat barang dan titik taruh (telapak dan barang tidak menutupi pandangan)
    o.append(box("cam_bracket", (0.55, 0.06, 0.06), (0.25, 0, 0.0), DARK))
    o.append(box("cam_body", (0.14, 0.1, 0.1), (0.5, 0, 0.0), ORN))
    o.append('<sensor name="arm_camera" type="camera"><pose>0.5 0 -0.03 0 1.9 0</pose>'
             '<camera><horizontal_fov>1.4</horizontal_fov><image><width>640</width><height>480</height></image><clip><near>0.1</near><far>60</far></clip></camera>'
             '<always_on>1</always_on><update_rate>15</update_rate><visualize>false</visualize><topic>/arm/camera/image</topic></sensor>')
    o.append('</link>')
    for ln in ("boom", "carriage", "gripper"):    # berat lengan diabaikan di sendi (tanpa gravitasi), beban barang tetap nyata
        pass
    o.append('<joint name="yaw" type="revolute"><parent>mount</parent><child>boom</child><axis><xyz>0 0 1</xyz><limit><lower>-3.14159</lower><upper>3.14159</upper></limit><dynamics><damping>400</damping></dynamics></axis></joint>')
    o.append('<joint name="slide" type="prismatic"><parent>boom</parent><child>carriage</child><axis><xyz>1 0 0</xyz><limit><lower>0</lower><upper>7.0</upper></limit><dynamics><damping>150</damping></dynamics></axis></joint>')
    o.append('<joint name="lift" type="prismatic"><parent>carriage</parent><child>gripper</child><axis><xyz>0 0 1</xyz><limit><lower>-2.0</lower><upper>0</upper></limit><dynamics><damping>200</damping></dynamics></axis></joint>')
    # kunci kaku ke lambung WAM-V
    o.append('<plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint"><parent_link>mount</parent_link><child_model>wamv</child_model><child_link>wamv/base_link</child_link></plugin>')
    o.append(ctrl("yaw", "/arm/yaw", 3500, 0, 5000, 5000))
    o.append(ctrl("slide", "/arm/slide", 900, 0, 450, 3000))
    o.append(ctrl("lift", "/arm/lift", 4000, 300, 900, 4000))
    o.append(f'<plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"><topic>/arm/joint_state</topic></plugin>')
    o.append("</model></sdf>")
    return "".join(o)


if __name__ == "__main__":
    open(sys.argv[1] if len(sys.argv) > 1 else "asv_arm.sdf", "w").write(build())
    print("ok")
