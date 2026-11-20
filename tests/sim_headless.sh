#!/usr/bin/env bash
# Uji misi penuh TANPA GUI (server Gazebo headless): arena x5 + WAM-V + lengan + bridge kamera lengan,
# lalu autonomous.py. Log di folder keluaran. Hentikan dengan Ctrl+C (semua proses ikut dimatikan).
# Pakai:  tests/sim_headless.sh [folder_log]
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
OUT="${1:-/tmp/asv_run_$(date +%H%M%S)}"
mkdir -p "$OUT"
VRX_WS="${VRX_WS:-$HOME/vrx_ws}"
source /opt/ros/jazzy/setup.bash
source "$VRX_WS/install/setup.bash"
INS="$VRX_WS/install"
export GZ_SIM_RESOURCE_PATH="$INS/vrx_gz/share/vrx_gz/models:$INS/wamv_description/share:$INS/wamv_gazebo/share:$INS/vrx_gazebo/share:$INS/wamv_description/share/wamv_description/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$VRX_WS/install/vrx_gz/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
PIDS=()
cleanup() { kill -INT "${PIDS[@]}" 2>/dev/null; sleep 2; kill "${PIDS[@]}" 2>/dev/null; }
trap cleanup EXIT
gz sim -r -s --headless-rendering "$HERE/pengiriman_kapal_otonom_x5.sdf" > "$OUT/gz.log" 2>&1 & PIDS+=($!)
sleep 15
gz service -s /world/pengiriman_kapal_otonom/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false' > /dev/null || true
ros2 launch vrx_gz spawn.launch.py world:=pengiriman_kapal_otonom name:=wamv model:=wam-v \
  x:=-48.5 y:=0 z:=0.3 Y:=0.0 > "$OUT/spawn.log" 2>&1 & PIDS+=($!)
sleep 25
(cd "$HERE" && python3 lengan.py pasang)
ros2 run ros_gz_bridge parameter_bridge /arm/camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  > "$OUT/bridge.log" 2>&1 & PIDS+=($!)
echo "log: $OUT/mission.log"
cd "$HERE" && python3 autonomous.py --ros-args -p gui:=false 2>&1 | tee "$OUT/mission.log" \
  | grep --line-buffered -E "EVENT|GERBANG|ERROR|WARN"
