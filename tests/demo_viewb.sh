#!/bin/bash
# M6, step 2: View B. Replay the recorded bag into RViz on a virtual display and capture the 3D view as video.
# The clip covers the same SIM-time interval as View A (taken from sync.txt written by demo_record.sh).
# Usage: demo_viewb.sh [preset]  ->  ~/sewer_ws/logs/demo/<preset>/clips/rviz.mp4
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
PRESET=${1:-water_debris_slip}
RATE=${RATE:-0.5}                       # replay speed: software-GL RViz keeps up better at half speed; the video is sped up again
LEAD=4                                  # sim seconds replayed before the interval starts (RViz needs TF and the map first)
L=~/sewer_ws/logs
D=$L/demo/$PRESET
cleanup() {
  pkill -f "ros2 bag"; pkill -f ffmpeg; pkill -f rviz2; pkill -f map_server; pkill -f lifecycle_manager
  pkill -f static_transform_publisher; pkill -f robot_state_publisher; pkill Xvfb
  sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1
}
cleanup
# sim-time interval of View A: "... (sim t = T0 .. T1)"
read T0 T1 < <(sed -n 's/.*sim t = \([0-9.]*\) \.\. \([0-9.]*\)).*/\1 \2/p' $D/sync.txt | tail -1)
BAG_START=$(python3 -c "
import yaml
m = yaml.safe_load(open('$D/bag/metadata.yaml'))['rosbag2_bagfile_information']
print(m['starting_time']['nanoseconds_since_epoch'] * 1e-9)")
OFFSET=$(python3 -c "print(max(0.0, $T0 - $BAG_START - $LEAD))")
LEAD_REAL=$(python3 -c "print(($T0 - $BAG_START - $OFFSET) / $RATE)")     # wall seconds of lead to cut off the video
DUR_REAL=$(python3 -c "print(($T1 - $T0) / $RATE)")
echo "View A interval sim t = $T0 .. $T1; bag starts at $BAG_START; offset $OFFSET s; replay rate $RATE"

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
ros2 launch sewer_bringup view_b.launch.py rviz:=false > $L/demo_viewb_nodes.log 2>&1 &
ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true \
  -p robot_description:="$(xacro $(ros2 pkg prefix sewer_description)/share/sewer_description/urdf/sewer_robot.urdf.xacro)" \
  > $L/demo_viewb_rsp.log 2>&1 &
env -u WAYLAND_DISPLAY DISPLAY=:99 QT_QPA_PLATFORM=xcb LIBGL_ALWAYS_SOFTWARE=1 \
  ros2 run rviz2 rviz2 -d /tmp/overview.rviz --ros-args -p use_sim_time:=true > $L/demo_viewb_rviz.log 2>&1 &
sleep 15
# the 3D view of RViz (right of the docks), cropped from the 1700x1000 screen
ffmpeg -nostdin -y -loglevel error -f x11grab -framerate 10 -video_size 1240x908 -i :99.0+460,62 \
  -t $(python3 -c "print($LEAD_REAL + $DUR_REAL + 3)") -c:v libx264 -pix_fmt yuv420p -crf 18 -preset veryfast $D/clips/rviz_raw.mp4 &
FF=$!
ros2 bag play $D/bag --clock --start-offset $OFFSET -r $RATE > $L/demo_viewb_play.log 2>&1 &
wait $FF
# cut the lead-in, speed the clip back up to real time (sim seconds), constant 10 fps
ffmpeg -nostdin -y -loglevel error -ss $LEAD_REAL -i $D/clips/rviz_raw.mp4 -t $DUR_REAL \
  -vf "setpts=${RATE}*PTS,fps=10" -c:v libx264 -pix_fmt yuv420p -crf 20 -preset medium $D/clips/rviz.mp4
cleanup
ffprobe -v error -show_entries format=duration -of csv=p=0 $D/clips/rviz.mp4
echo DEMO_VIEWB_DONE
