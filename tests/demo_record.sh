#!/bin/bash
# M6, step 1: run one demo preset and record everything the final video needs.
#   View A material: Gazebo camera frames (oblique overview, chase, robot camera), resampled to 10 fps by sim time
#   View B material: a rosbag of every topic RViz shows (replayed later by demo_viewb.sh, so RViz does not load the live run)
#   Scoring material: mission_check trajectory CSV, events and summary JSON (error, R, health, collisions, route events)
# Usage: demo_record.sh [preset]  ->  ~/sewer_ws/logs/demo/<preset>/{bag,frames,seq,clips,traj*.csv|json,sync.txt}
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
PRESET=${1:-water_debris_slip}
L=~/sewer_ws/logs
D=$L/demo/$PRESET
cleanup() {
  pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f "ros2 launch"; pkill -f "ros2 bag"; pkill -f parameter_bridge
  pkill -f localizer_node; pkill -f scan_disturbance_node; pkill -f odom_disturbance_node; pkill -f blockage_detector_node
  pkill -f route_planner_node; pkill -f route_follower_node; pkill -f collision_counter_node; pkill -f mission_check
  pkill -f record_frames; pkill -f robot_state_publisher
  sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1
}
cleanup
rm -rf $D; mkdir -p $D/frames $D/clips
ros2 launch sewer_bringup demo.launch.py preset:=$PRESET record:=true bag:=$D/bag > $L/demo_$PRESET.log 2>&1 &
sleep 36
ros2 run ros_gz_bridge parameter_bridge \
  '/robot_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' '/chase_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/oblique_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' > $L/demo_bridge.log 2>&1 &
sleep 4
python3 ~/sewer_ws/tests/mission_check.py ${CHK_T:-330} $D/traj.csv > $D/check.log 2>&1 &
CHK=$!
python3 ~/sewer_ws/tests/record_frames.py $D/frames 10 ${REC_T:-600} robot=/robot_cam/image chase=/chase_cam/image oblique=/oblique_cam/image > $D/rec.log 2>&1 &
REC=$!
wait $CHK
wait $REC
sleep 2
pkill -INT -f "ros2 bag record"; sleep 4          # close the bag cleanly (writes metadata.yaml)
echo "== mission result"; grep -E 'end: status|health:|RESULT|FAIL|PASS' $D/check.log | cut -c1-200
cat $D/rec.log
cleanup
# ---- View A clips: constant 10 fps by sim time, skipping the first 2.5 s of frames (the chase camera starts behind the end wall)
python3 ~/sewer_ws/tests/build_sequence.py $D/frames $D/seq 10 2.5 robot chase oblique | tee $D/sync.txt
ENC="-c:v libx264 -pix_fmt yuv420p -crf 20 -preset medium"
for c in robot chase oblique; do
  ffmpeg -nostdin -y -loglevel error -framerate 10 -i $D/seq/$c/%05d.jpg $ENC $D/clips/$c.mp4
done
ls -la $D/clips $D/bag
echo DEMO_RECORD_DONE
