# Misi Otonom ASV (Vision + Control)

Rombakan total `autonomous.py`. Satu perintah menjalankan misi penuh sesuai guidebook NALA 16:
**pick up 3 barang di Spot 1 → FTP → docking di slot Spot 2 → taruh 3 barang di zona putih → buzzer + water cannon tepat saat barang terakhir dilepas.**
Semua barang dibawa dalam **satu trip** (kategori Disruptive pick-and-place).

## Cara menjalankan

```bash
./jalankan_wamv.sh                     # terminal 1: arena x5 + WAM-V + lengan + bridge kamera lengan
python3 autonomous.py                  # terminal 2: misi penuh (jendela FRONT & ARM)
# kedua terminal harus sudah: source /opt/ros/jazzy/setup.bash && source ~/vrx_ws/install/setup.bash
python3 autonomous.py --ros-args -p gui:=false          # tanpa jendela
python3 autonomous.py --ros-args -p skip_pickup:=true   # uji FTP + docking saja
python3 autonomous.py --ros-args -p gt_fallback:=false  # murni visi, tanpa cadangan pose Gazebo
```

Kill switch & status (untuk tim web / STM32):

```bash
ros2 topic pub --once /asv/kill std_msgs/msg/Bool "{data: true}"    # semua thruster 0, lengan berhenti
ros2 topic pub --once /asv/kill std_msgs/msg/Bool "{data: false}"   # lanjut
ros2 topic echo /asv/state            # JSON: phase, pose, thrust, barang, gerbang, event
ros2 topic echo /asv/buzzer           # std_msgs/Bool
ros2 topic echo /asv/water_cannon     # std_msgs/Bool
```

Uji misi penuh tanpa GUI (server Gazebo headless, semua proses dimatikan saat Ctrl+C):

```bash
tests/sim_headless.sh /tmp/run1        # log lengkap di /tmp/run1/mission.log
```

Uji tanpa Gazebo (cepat, untuk tuning):

```bash
python3 tests/sim_ftp_offline.py --plot ftp.png   # dinamika WAM-V 3-DOF + deteksi sintetis ber-noise
python3 tests/test_geometry.py                    # konsistensi proyeksi 3 kamera + IK lengan
```

## Struktur

| File | Isi |
|---|---|
| `autonomous.py` | Node ROS 2 + state machine misi, kill switch, status JSON |
| `asv/config.py` | Semua konstanta (geometri arena dari `figs.py` × SCALE, gain, batas lengan) |
| `asv/geometry.py` | Transformasi pose, model kamera pinhole, sinar piksel × bidang, kerangka lintasan (s, l) |
| `asv/perception.py` | YOLO `frontnav.pt` → peta landmark; YOLO `arm.pt` + kontur → posisi barang; kamera overview (HSV) → lokasi kasar |
| `asv/planner.py` | Peta landmark, pemasangan gerbang hijau-merah, lintasan elastic band |
| `asv/control.py` | Path following (carrot sepanjang lintasan), PID heading & kecepatan, station keeping |
| `asv/arm_ctrl.py` | IK lengan, gerak sendi dengan umpan balik joint_state, pick/place berbasis visi |
| `asv/aux_effects.py` | Event marker buzzer & semburan air di Gazebo |

## Vision

**Kamera depan (FTP & docking)** — `frontnav.pt` (green, red, yellow, black, target_banner, dock).
- Jarak buoy dari lebar bbox (bola, radius diketahui): `d = r / sin(atan(w/2f))`; arah dari piksel pusat, lewat pose kamera
  (x 0.75, z 1.5, pitch 15°) dan pose kapal **pada timestamp gambar** → posisi dunia.
- Digabung ke **peta landmark** (rata-rata berbobot 1/jarak², radius asosiasi tumbuh dengan jarak, gabung duplikat,
  hapus landmark yang seharusnya terlihat tapi tidak terdeteksi). Landmark baru dipakai setelah ≥4 deteksi dan pernah terlihat ≤30 m.
- Merah/hitam yang tertukar dikoreksi dengan cek HSV. Deteksi yang menyentuh tepi bawah / jatuh di badan kapal dibuang
  (muatan kuning di dek sendiri sempat terbaca sebagai rintangan).
- Banner target hanya dipakai **arahnya** (bukan jarak dari bbox): sinar banner dipotongkan dengan garis dinding Spot 2 → posisi
  lateral slot docking; juga untuk membidik water cannon.

**Kamera lengan (pick & place)** — `arm.pt` (cube, sphere, prism, dock).
- Piksel pusat bbox → sinar → dipotongkan dengan bidang z yang diketahui (permukaan dermaga / dek) → posisi dunia.
  Diulang (closed loop) sampai koreksi < 6 cm. Hasil di Gazebo: galat 2–6 cm.
- Pengecek kedua: kontur kuning bundar ⇒ bola, bersudut ⇒ bukan bola.
- Zona taruh putih: HSV + bentuk kotak.

**Kamera overview** (`overview_camera/image`, disediakan guidebook untuk Vision) — lokasi kasar barang (kuning) dan zona taruh
(putih) di dermaga dalam jangkauan lengan, galat ≈0.1 m. Kelas barang ditentukan kamera lengan.

## Control

**FTP (sesuai arahan: titik tengah hijau-merah = waypoint wajib).**
1. Pasangan hijau-merah dari peta: lebar 4–22.5 m, melintang terhadap lintasan, hijau di kiri → **gerbang**; titik tengahnya
   jadi waypoint **wajib**.
2. **Elastic band** pada kerangka lintasan l(s): dihaluskan (meminimalkan kelengkungan) dengan kendala
   - KERAS: lewat tepat di titik tengah gerbang dan lurus ±3 m di sekitarnya (memotong gerbang tegak lurus),
   - rintangan: jarak pusat ≥ radius + ½ lebar kapal + 1.5 m, sisi lewat dipilih otomatis (yang muat di koridor & belokan terkecil),
   - koridor: tidak mendekati baris buoy,
   - keluar slot Spot 1 dan masuk slot Spot 2 lurus.
   Direncanakan ulang 2.5×/detik selama peta bertambah.
3. **Path following**: proyeksi kapal ke lintasan + target sejauh 5.5 m di depan sepanjang lintasan (tikungan diantisipasi),
   kecepatan dikurangi di tikungan dan saat mendekati dock.
4. Heading PD + kecepatan PI dengan feed-forward drag WAM-V VRX (xU=100, xUU=150, nR=800).
5. Saat kapal memotong garis hijau-merah, offset dari tengah gerbang dicatat ke log dan `/asv/state`.

**Docking & station keeping (3-DOF).** Target = pusat slot Spot 2 (jarak slot-ke-slot dari guidebook, lateral dari banner).
Selama di slot (pickup di Spot 1, unload di Spot 2), kapal ditahan **maju, samping, dan heading** dengan PID (x, y, yaw) →
gaya (Fx, Fy, Mz) → alokasi pseudo-invers ke dua mesin **azimuth** WAM-V (`/wamv/thrusters/*/pos`). Sudut mesin dipilih
yang paling dekat ke sudut sekarang (tanpa lompatan 180°) dengan laju terbatas. Di sim: galat 1.8 m → <0.1 m dalam ±9 detik.
Tanpa ini kapal hanyut ke samping sampai menabrak dinding slot dan miring. Matikan dengan `-p azimuth_hold:=false`
(kembali ke hold diferensial maju + heading saja). Saat FTP mesin kembali lurus (kendali diferensial biasa).

**Pengaman kemiringan.** Bila |roll| atau |pitch| > 5° (`ROLL_PITCH_LIMIT`), gerakan lengan dijeda sampai kapal tegak lagi.
Roll/pitch ditampilkan di jendela FRONT. Berat lengan sendiri (menjulur penuh 7 m) hanya memiringkan kapal ±2°.

**Lengan.** IK silindris analitik (yaw = atan2, slide = jarak − 0.9). Setiap gerakan berupa ramp smoothstep terbatas kecepatan,
lalu menunggu joint_state mencapai toleransi. Barang diayun di atas puncak lidar WAM-V. Grab dianggap berhasil hanya
bila barang benar-benar ikut gripper (cek pose), bukan dari nilai balik service. Kalau gagal, barang di-detach untuk keamanan.

**Buzzer + water cannon.** Sebelum barang terakhir ditaruh, heading dibidik ke banner (kamera depan, toleransi 3°). Tepat saat
detach barang terakhir: `/asv/buzzer` dan `/asv/water_cannon` = true, lampu merah di atas kapal dan semburan air biru dari haluan
ke banner muncul di Gazebo selama 6 detik.

## Hasil uji di Gazebo (headless, world x5, run ke-4 setelah perbaikan)

| Tahap | Hasil |
|---|---|
| Pickup Spot 1 | 3/3 barang (visi: overview → kamera lengan), galat refine 2–6 cm, ±3.3 menit |
| FTP | 5/5 gerbang dilewati, offset dari tengah +0.42, +0.38, +0.03, −0.09, +0.16 m; lebar terukur 10.2 / 17.1 / 7.5 / 16.1 / 9.8 m (asli 10 / 17.5 / 7.5 / 17.5 / 10) |
| Docking | galat posisi 0.11 m dari pusat slot Spot 2, ±4 menit sejak keluar slot Spot 1 |
| Unload | 3/3 di zona putih, galat 3–8 cm (bola sedikit menggelinding setelahnya, tetap di zona) |
| Buzzer + cannon | menyala tepat saat detach barang terakhir, bidikan banner −0.3° |

Masalah yang ditemukan saat uji dan sudah diperbaiki: grab yang "gagal" padahal barang menempel (lengan lalu menabrakkan
barang ke barang lain), kamera depan memetakan muatan kuning di dek sendiri sebagai rintangan, bias jarak bbox YOLO
(dikalibrasi, `RANGE_SCALE`), lengan berputar 360° untuk kompartemen di yaw ±π, banner terpotong kamera saat di slot.

## Catatan / batasan
- Jendela OpenCV (FRONT, ARM) digambar oleh SATU thread khusus. HighGUI/Qt tidak thread-safe; memanggil `imshow` dari
  timer `MultiThreadedExecutor` membuat jendela "Not Responding". Jendela menampilkan gambar live + anotasi deteksi terbaru.
- `gt_fallback` (default true): hanya bila kamera gagal total, pose Gazebo dipakai untuk lokasi/kelas barang, dan selalu dicatat
  sebagai WARNING. Setel `false` untuk demo murni visi.
- Gazebo dipakai juga untuk pose kapal (pengganti GPS+IMU) dan verifikasi (barang benar-benar menempel/mendarat).
- Kamera lengan tidak bisa melihat barang di kompartemen haluan (terlalu dekat/tertutup). Posisi kompartemen diketahui dari
  desain dek, jadi dipakai langsung; di log tampil sebagai WARN "tidak terlihat", dan ini normal.
- Gain kendali dituning untuk WAM-V VRX. Untuk kapal asli (0.99 m), skala dan gain di `asv/config.py` perlu diubah.
- Massa barang 0.05 kg (`ITEM_MASS` di `make_world.py`, absolut, tidak dikali skala³).
