#!/usr/bin/env bash
# Menjalankan world arena Pengiriman Kapal Otonom di Gazebo (butuh VRX terpasang).
# Pakai:  ./jalankan_arena.sh            -> arena skala asli
#         ./jalankan_arena.sh x5         -> arena skala 5 (untuk WAM-V bawaan VRX)
# Argumen tambahan diteruskan ke gz sim, contoh: ./jalankan_arena.sh -s --iterations 100
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
VRX_WS="${VRX_WS:-$HOME/vrx_ws}"
if [ ! -d "$VRX_WS/install/vrx_gz" ]; then
  echo "VRX tidak ditemukan di $VRX_WS. Set VRX_WS=/path/ke/vrx_ws" >&2; exit 1
fi
WORLD="$HERE/pengiriman_kapal_otonom.sdf"
if [ "${1:-}" = "x5" ]; then WORLD="$HERE/pengiriman_kapal_otonom_x5.sdf"; shift; fi
export QT_QPA_PLATFORM=xcb   # paksa X11: pada sesi Wayland jendela render gagal dibuat
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
# share/ dari paket model WAM-V wajib ada, kalau tidak mesh lambung, mesin, dan propeler gagal dimuat (kapal tampak tanpa hull)
INS="$VRX_WS/install"
export GZ_SIM_RESOURCE_PATH="$INS/vrx_gz/share/vrx_gz/models:$INS/wamv_description/share:$INS/wamv_gazebo/share:$INS/vrx_gazebo/share:$INS/wamv_description/share/wamv_description/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$VRX_WS/install/vrx_gz/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
exec gz sim -r "$WORLD" "$@"
