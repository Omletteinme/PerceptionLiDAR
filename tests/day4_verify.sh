#!/bin/bash
# Day 4-5 verification: for each scenario launch the sewer world headless, run the network check, clean up.
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
L=~/sewer_ws/logs
T=~/sewer_ws/tests

cleanup() { pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f ros2; pkill -f parameter_bridge; sleep 3; }

run() {  # run <world file> <scenario>
  cleanup
  ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=$1 x:=0.0 y:=0.0 yaw:=0.0 > $L/sewer_$2.log 2>&1 &
  sleep 35
  echo "=================== $2 ($1)"
  if [ "$2" = "blocked_direct" ]; then
    ros2 run ros_gz_bridge parameter_bridge '/topdown_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' > $L/cam_bridge.log 2>&1 &
    sleep 5
    python3 $T/day4_topdown.py $L/topdown_blocked.png
  fi
  python3 $T/day4_network_check.py $2
  echo "-- log errors:"; grep -iE 'error|failed|exception' $L/sewer_$2.log | cut -c1-160 | sort | uniq -c | head -5
}

run sewer_network_clear.sdf clear_direct
run sewer_network.sdf blocked_direct
run sewer_network.sdf blocked_alt
cleanup
echo VERIFY_DONE
