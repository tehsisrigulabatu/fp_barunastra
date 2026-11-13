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

if __name__ == "__main__":
    print("File ini hanya panduan. Baca docstring di atas.")
