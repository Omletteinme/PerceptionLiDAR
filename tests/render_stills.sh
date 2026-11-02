#!/bin/bash
# Grab a few still frames from every Gazebo camera with the robot parked at the entry A (no mission). Used to tune cameras/lighting.
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
L=~/sewer_ws/logs
OUT=$L/day10/stills
rm -rf $OUT; mkdir -p $OUT
cleanup() {
  pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f ros2; pkill -f parameter_bridge
  sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1
}
cleanup
ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=sewer_network.sdf x:=0.0 y:=0.0 yaw:=0.0 > $L/stills_sim.log 2>&1 &
sleep 35
ros2 run ros_gz_bridge parameter_bridge \
  '/robot_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' '/chase_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/oblique_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' '/topdown_cam/image@sensor_msgs/msg/Image[gz.msgs.Image' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' > $L/stills_bridge.log 2>&1 &
sleep 6
python3 ~/sewer_ws/tests/record_frames.py $OUT 1 8 robot=/robot_cam/image chase=/chase_cam/image oblique=/oblique_cam/image topdown=/topdown_cam/image
ls $OUT/*
cleanup
echo STILLS_DONE
