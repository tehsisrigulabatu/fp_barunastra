<div align="center">

<img width="100%" src="https://capsule-render.vercel.app/api?type=waving&color=0:0A1A3A,100:1B7F3B&height=150&section=header&text=Vision%20Sub-division&fontSize=34&fontColor=ffffff&fontAlignY=38&desc=Barunastra%20Internship%202026&descAlignY=60&descSize=16" alt="Vision sub-division" />

![Division](https://img.shields.io/badge/Division-PGR-1B7F3B?style=for-the-badge)
![Owner](https://img.shields.io/badge/Owner-@tehsisrigulabatu-1F3864?style=for-the-badge&logo=github&logoColor=white)

</div>

## Untuk set environment ros2 dan gazebo

```bash
cd ~/fp3d_ws/src/arena-vrx
source /opt/ros/jazzy/setup.bash
./jalankan_wamv.sh
```

## Untuk spawn lengan
```bash
python3 lengan.py pasang
```

## Untuk ambil dataset object dari camera arm
```bash
python3 arm_move.py
=== TELEOP ARM + AUTO-SAVE DATASET ===
  [A / D] : Putar Yaw (Kiri / Kanan)
  [W / S] : Julurkan / Tarik Slide (Maju / Mundur)
  [Q / E] : Naik / Turun Lift (Atas / Bawah)
  [R]     : Reset posisi awal lengan
  [P]     : Pause / Resume Auto-Save
  [SPACE] : Capture manual instan
  [ESC]   : Keluar
```

## Untuk navigate asv manual
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
buat dia gerak, butuh publisher left and right thruster, makanya running juga 
```bash
python3 dataset.py
```
Tekan 't' di teleop untuk MULAI rekam, 'b' untuk STOP rekam.

## Untuk navigate autonomous
```bash
./jalankan_wamv.sh
di terminal lain
python3 autonomous.py
```

## Topik penting
|topic|untuk|
|---|---|
|/arm/camera/image|kamera di arm|
|/clock|
|/parameter_events|
|/pinger/set_pinger_position|
|/rosout|
|/tf|
|/tf_static|
|/vrx/contacts|
|/wamv/joint_states|
|/wamv/pose|
|/wamv/pose_static|
|/wamv/robot_description|
|/wamv/sensors/acoustics/receiver/range_bearing|
|/wamv/sensors/cameras/front_left_camera_sensor/camera_info|
|/wamv/sensors/cameras/front_left_camera_sensor/image_raw|yang kupake buat dataset buoy|
|/wamv/sensors/cameras/front_left_camera_sensor/optical/camera_info|
|/wamv/sensors/cameras/front_left_camera_sensor/optical/image_raw|
|/wamv/sensors/cameras/front_right_camera_sensor/camera_info|
|/wamv/sensors/cameras/front_right_camera_sensor/image_raw|
|/wamv/sensors/cameras/front_right_camera_sensor/optical/camera_info|
|/wamv/sensors/cameras/front_right_camera_sensor/optical/image_raw|
|/wamv/sensors/cameras/middle_right_camera_sensor/camera_info|
|/wamv/sensors/cameras/middle_right_camera_sensor/image_raw|
|/wamv/sensors/cameras/middle_right_camera_sensor/optical/camera_info|
|/wamv/sensors/cameras/middle_right_camera_sensor/optical/image_raw|
|/wamv/sensors/gps/gps/fix|
|/wamv/sensors/imu/imu/data|
|/wamv/sensors/lidars/lidar_wamv_sensor/points|
|/wamv/sensors/lidars/lidar_wamv_sensor/scan|
|/wamv/shooters/ball_shooter/fire|
|/wamv/thrusters/left/pos|
|/wamv/thrusters/left/thrust|
|/wamv/thrusters/right/pos|
|/wamv/thrusters/right/thrust|
