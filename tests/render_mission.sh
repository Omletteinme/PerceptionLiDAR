#!/bin/bash
# Render the full mission from the Gazebo cameras (headless) and encode MP4 clips.
# Pipeline = the verified live mission: sim + both localizers + detector + planner + follower; mission_check observes.
# Usage: render_mission.sh [disturbed true|false] [severity]   -> ~/sewer_ws/logs/day10/clips/*.mp4
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
DIST=${1:-false}; SEV=${2:-1.0}
L=~/sewer_ws/logs
FR=$L/day10/frames
CL=$L/day10/clips
rm -rf $FR $CL; mkdir -p $FR $CL
cleanup() {
  pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f ros2; pkill -f parameter_bridge; pkill -f localizer_node
  pkill -f scan_disturbance_node; pkill -f blockage_detector_node; pkill -f route_planner_node
  pkill -f route_follower_node; pkill -f mission_check; pkill -f record_frames
  sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1
}
cleanup
ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=sewer_network.sdf x:=0.0 y:=0.0 yaw:=0.0 > $L/render_sim.log 2>&1 &
sleep 35
ros2 launch sewer_bringup localization.launch.py filter:=both disturbed:=$DIST severity:=$SEV seed:=0 > $L/render_loc.log 2>&1 &
sleep 10
ros2 run ros_gz_bridge parameter_bridge \
  '/robot_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' '/chase_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/oblique_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' > $L/render_bridge.log 2>&1 &
sleep 5
echo "== LiDAR self-hit probe (beacon mast must be above the scan plane)"
python3 ~/sewer_ws/tests/scan_probe.py
python3 ~/sewer_ws/tests/mission_check.py 300 > $L/render_check.log 2>&1 &
CHK=$!
python3 ~/sewer_ws/tests/record_frames.py $FR 10 400 robot=/robot_cam/image chase=/chase_cam/image oblique=/oblique_cam/image > $L/render_rec.log 2>&1 &
REC=$!
sleep 3
ros2 launch sewer_bringup mission.launch.py > $L/render_mission.log 2>&1 &
wait $REC
wait $CHK
echo "== mission result"; grep -E 'end: status|RESULT|FAIL' $L/render_check.log | cut -c1-170
echo "== frames"; cat $L/render_rec.log
for c in robot chase oblique; do echo "$c: $(ls $FR/$c | wc -l) frames"; done
cleanup

# ---- encode: H.264, constant 10 fps resampled by sim time, skipping the first 2.5 s (robot still at the entry;
#      the chase camera starts behind the end wall)
SEQ=$L/day10/seq
rm -rf $SEQ
python3 ~/sewer_ws/tests/build_sequence.py $FR $SEQ 10 2.5 robot chase oblique
ENC="-c:v libx264 -pix_fmt yuv420p -crf 20 -preset medium"
for c in robot chase oblique; do
  ffmpeg -y -loglevel error -framerate 10 -i $SEQ/$c/%05d.jpg $ENC $CL/$c.mp4
done
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf
LBL() { echo "drawtext=fontfile=$FONT:text='$1':x=10:y=10:fontsize=$2:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=6"; }
ffmpeg -y -loglevel error -framerate 10 -i $SEQ/oblique/%05d.jpg -framerate 10 -i $SEQ/chase/%05d.jpg \
  -framerate 10 -i $SEQ/robot/%05d.jpg -filter_complex \
  "[0]scale=960:540,$(LBL 'Oblique overview' 24)[a];[1]scale=480:270,$(LBL 'Chase camera' 18)[b];[2]scale=480:270,$(LBL 'Robot camera' 18)[c];[b][c]vstack[bc];[a][bc]hstack" \
  $ENC -shortest $CL/mission_3d.mp4
ls -la $CL
echo RENDER_DONE
