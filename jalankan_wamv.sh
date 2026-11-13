#!/usr/bin/env bash
# Menjalankan arena skala 5 di Gazebo (VRX) lalu menaruh WAM-V di dalam slot start Spot 1.
# Pakai:  ./jalankan_wamv.sh
# Catatan: simulasi kadang mulai dalam keadaan jeda, jadi skrip ini menekan "play" otomatis.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
VRX_WS="${VRX_WS:-$HOME/vrx_ws}"
export QT_QPA_PLATFORM=xcb   # paksa X11: pada sesi Wayland jendela render gagal dibuat
source /opt/ros/jazzy/setup.bash
source "$VRX_WS/install/setup.bash"
"$HERE/jalankan_arena.sh" x5 "$@" &
GZ=$!
LAUNCH=
# saat keluar: hentikan ros2 launch dengan SIGINT (agar semua node anaknya ikut mati) lalu Gazebo
trap 'kill -INT $LAUNCH 2>/dev/null; sleep 2; kill $GZ 2>/dev/null' EXIT
play() {
  gz service -s /world/pengiriman_kapal_otonom/control --reqtype gz.msgs.WorldControl \
    --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false' >/dev/null 2>&1 || true
}
sleep "${SPAWN_DELAY:-12}"
play
# Skala 5: slot start Spot 1 berpusat sekitar x = -48,5 m, y = 0 (origin di pusat kolam), menghadap Spot 2
ros2 launch vrx_gz spawn.launch.py world:=pengiriman_kapal_otonom name:=wamv model:=wam-v x:=-48.5 y:=0 z:=0.3 Y:=0.0 &
LAUNCH=$!
sleep 20
play
wait $GZ
