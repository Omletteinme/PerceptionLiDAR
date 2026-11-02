#!/bin/bash
# View B check: Gazebo + both localizers (disturbed) + RViz in a virtual display; command checks + screenshots.
# Usage: view_verify.sh [severity]   -> screenshots in ~/sewer_ws/logs/day6/rviz/
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
SEV=${1:-3.0}
L=~/sewer_ws/logs
OUT=$L/day6/rviz
mkdir -p $OUT; rm -f $OUT/*.png
cleanup() {
  pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f ros2; pkill -f parameter_bridge; pkill -f localizer_node
  pkill -f scan_disturbance_node; pkill -f rviz2; pkill -f map_server; pkill -f lifecycle_manager
  pkill -f static_transform_publisher; pkill Xvfb; sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1
}
shot() { DISPLAY=:99 import -window root $OUT/$1.png 2>/dev/null; }
cleanup

Xvfb :99 -screen 0 1700x1000x24 +extension GLX +render -noreset > $L/xvfb.log 2>&1 &
sleep 2
ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=sewer_network.sdf x:=0.0 y:=0.0 yaw:=0.0 > $L/view_sim.log 2>&1 &
sleep 35
ros2 launch sewer_bringup localization.launch.py filter:=both disturbed:=true severity:=$SEV seed:=0 > $L/view_loc.log 2>&1 &
sleep 8
env -u WAYLAND_DISPLAY DISPLAY=:99 QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 \
  ros2 launch sewer_bringup view_b.launch.py > $L/view_rviz.log 2>&1 &
sleep 20

echo "== rviz process"; pgrep -fa 'rviz2' | grep -v pgrep | cut -c1-90 | head -2
echo "== /map info"; timeout 10 ros2 topic echo --once /map --qos-durability transient_local --qos-reliability reliable --field info 2>&1 | grep -E 'resolution|width|height' | tr '\n' ' '; echo
echo "== TF map->world, map->base_footprint"
for pair in "map world" "map base_footprint"; do
  timeout 6 ros2 run tf2_ros tf2_echo $pair 2>&1 | grep -m1 -A1 'At time' | tr '\n' ' '; echo " [$pair]"
done
echo "== status markers"
for ns in baseline reliability; do printf "$ns: "; timeout 6 ros2 topic echo --once /$ns/status_marker --field text 2>&1 | head -1; done
echo "== reliability_scan frame + intensities"
timeout 6 ros2 topic echo --once /reliability/reliability_scan --field header.frame_id 2>&1 | head -1
timeout 6 ros2 topic echo --once /reliability/reliability_scan --field intensities 2>&1 | head -1 | cut -c1-90
shot f_idle

python3 ~/sewer_ws/tests/ros_localizer_check.py 0.5 > $L/view_drive.log 2>&1 &
DRV=$!
T0=$(date +%s)
i=0
for off in 5 15 36 54 70 86; do
  while [ $(( $(date +%s) - T0 )) -lt $off ]; do sleep 1; done
  shot f$i; i=$((i+1))
done
wait $DRV
echo "== drive/check result"; tail -n 14 $L/view_drive.log
echo "== rviz log (errors/warnings)"; grep -iE 'error|warn|fail|unknown' $L/view_rviz.log | cut -c1-170 | sort | uniq -c | head -12
ls -la $OUT
cleanup
echo VIEW_VERIFY_DONE
