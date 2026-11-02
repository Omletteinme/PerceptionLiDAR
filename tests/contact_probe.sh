#!/bin/bash
# Where do contact messages go missing? Start 0.5 m from the arena box, push into it, inspect the raw gz topics.
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
L=~/sewer_ws/logs
cleanup() {
  pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f "ros2 launch"; pkill -f parameter_bridge; pkill -f collision_counter_node
  sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1
}
cleanup
ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=test_arena.sdf x:=0.8 y:=1.0 yaw:=0.0 > $L/m4_probe_sim.log 2>&1 &
sleep 30
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" > /dev/null 2>&1 &
PUB=$!
sleep 15
echo "--- ground truth"
timeout 5 ros2 topic echo --once /ground_truth/odom --field pose.pose.position.x
T=/world/test_arena/model/sewer_robot/link
echo "--- topic info"
gz topic -i -t /contacts
gz topic -i -t $T/base_link/sensor/base_link_contact/contact
echo "--- raw base_link sensor topic (6 s)"
timeout 6 gz topic -e -t $T/base_link/sensor/base_link_contact/contact -n 1
echo "--- raw left wheel sensor topic (6 s)"
timeout 6 gz topic -e -t $T/left_wheel_link/sensor/left_wheel_link_contact/contact -n 1
echo "--- raw /contacts (6 s)"
timeout 6 gz topic -e -t /contacts -n 1
echo "--- ros /contacts hz"
timeout 5 ros2 topic hz /contacts
kill $PUB 2>/dev/null
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
cleanup
echo PROBE_DONE
