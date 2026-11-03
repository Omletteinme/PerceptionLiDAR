#!/bin/bash
# For each world: LiDAR self/phantom check at the entry (tests/scan_probe.py) and still frames from every camera.
# Usage: world_stills.sh sewer_network sewer_network_v2   ->  ~/sewer_ws/logs/looks/<world>/{robot,chase,oblique,topdown}/*.jpg
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
L=~/sewer_ws/logs
cleanup() {
  pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f "ros2 launch"; pkill -f parameter_bridge; pkill -f collision_counter_node
  sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1
}
for W in "$@"; do
  OUT=$L/looks/$W
  mkdir -p $OUT
  cleanup
  ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=$W.sdf x:=0.0 y:=0.0 yaw:=0.0 > $L/looks_sim_$W.log 2>&1 &
  sleep 35
  ros2 run ros_gz_bridge parameter_bridge \
    '/robot_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' '/chase_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' \
    '/oblique_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' '/topdown_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' \
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' > $L/looks_bridge_$W.log 2>&1 &
  sleep 6
  echo "=== $W: LiDAR check"
  python3 ~/sewer_ws/tests/scan_probe.py
  python3 ~/sewer_ws/tests/record_frames.py $OUT 1 8 robot=/robot_cam/image chase=/chase_cam/image oblique=/oblique_cam/image topdown=/topdown_cam/image > /dev/null 2>&1
  echo "frames: $(ls $OUT/oblique | wc -l) oblique"
  cleanup
done
echo STILLS_DONE
