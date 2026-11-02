#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
L=~/sewer_ws/logs
cleanup() {
  pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f ros2; pkill -f parameter_bridge; pkill -f collision_counter_node
  sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1
}
cleanup
ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=test_arena.sdf x:=-2.0 y:=1.0 yaw:=0.0 > $L/m4_sim.log 2>&1 &
sleep 30
echo "--- gz topics with contact"
gz topic -l | grep -i contact
timeout 40 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" > /dev/null &
PUB=$!
sleep 22
echo "--- ground truth after 22 s"
timeout 5 ros2 topic echo --once /ground_truth/odom --field pose.pose.position
echo "--- raw gz contacts (3 s)"
timeout 3 gz topic -e -t /contacts | head -40
echo "--- ros /collisions"
timeout 4 ros2 topic echo --once /collisions
kill $PUB 2>/dev/null
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null
cleanup
echo DEBUG_DONE
