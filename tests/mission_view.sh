#!/bin/bash
# Mission in RViz (virtual display): whole-network view with the planned route, detected blockages and both estimates.
# Usage: mission_view.sh [severity]   -> screenshots in ~/sewer_ws/logs/day9/rviz/
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
SEV=${1:-3.0}
L=~/sewer_ws/logs
OUT=$L/day9/rviz
mkdir -p $OUT $L/day9; rm -f $OUT/*.png
cleanup() {
  pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f ros2; pkill -f parameter_bridge; pkill -f localizer_node; pkill -f scan_disturbance_node
  pkill -f blockage_detector_node; pkill -f route_planner_node; pkill -f route_follower_node; pkill -f mission_check
  pkill -f rviz2; pkill -f map_server; pkill -f lifecycle_manager; pkill -f static_transform_publisher; pkill Xvfb
  sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1
}
shot() { DISPLAY=:99 import -window root $OUT/$1.png 2>/dev/null; }
cleanup
python3 - <<'PY'
import yaml
src = '/home/jeevi/sewer_ws/install/sewer_bringup/share/sewer_bringup/config/view_b.rviz'
d = yaml.safe_load(open(src))
views = d['Visualization Manager']['Views']
views['Current'] = dict(next(v for v in views['Saved'] if v['Name'].startswith('Overview')))
yaml.safe_dump(d, open('/tmp/overview.rviz', 'w'))
PY
Xvfb :99 -screen 0 1700x1000x24 +extension GLX +render -noreset > $L/xvfb.log 2>&1 &
sleep 2
ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=sewer_network.sdf x:=0.0 y:=0.0 yaw:=0.0 > $L/mv_sim.log 2>&1 &
sleep 35
ros2 launch sewer_bringup localization.launch.py filter:=both disturbed:=true severity:=$SEV seed:=0 > $L/mv_loc.log 2>&1 &
sleep 10
ros2 launch sewer_bringup view_b.launch.py rviz:=false > $L/mv_view.log 2>&1 &
env -u WAYLAND_DISPLAY DISPLAY=:99 QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 ros2 run rviz2 rviz2 -d /tmp/overview.rviz > $L/mv_rviz.log 2>&1 &
sleep 15
shot f0_before
python3 ~/sewer_ws/tests/mission_check.py 230 $L/day9/traj_view.csv > $L/mv_check.log 2>&1 &
CHK=$!
sleep 3
ros2 launch sewer_bringup mission.launch.py > $L/mv_mission.log 2>&1 &
T0=$(date +%s)
i=1
for off in 9 24 40 56 72 90; do
  while [ $(( $(date +%s) - T0 )) -lt $off ]; do sleep 1; done
  shot f${i}; i=$((i+1))
done
wait $CHK
shot f_end
echo "== mission result"; grep -E 'end: status|RESULT|FAIL' $L/mv_check.log | cut -c1-170
echo "== rviz errors"; grep -iE 'error' $L/mv_rviz.log | cut -c1-150 | sort | uniq -c | head -5
ls $OUT
cleanup
echo MISSION_VIEW_DONE
