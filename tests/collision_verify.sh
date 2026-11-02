#!/bin/bash
# M4 live check in the test arena (robot starts at x=-2, y=1; the arena box occupies x 1.3-1.7, y 0.6-1.0):
# (1) drive on open floor -> 0 collisions (the floor is ignored), (2) drive into the box -> a collision is counted.
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
L=~/sewer_ws/logs
cleanup() {
  pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f "ros2 launch"; pkill -f parameter_bridge; pkill -f collision_counter_node
  sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1
}
cleanup
ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=test_arena.sdf x:=-2.0 y:=1.0 yaw:=0.0 > $L/m4_sim.log 2>&1 &
sleep 30
gt_x() { timeout 5 ros2 topic echo --once /ground_truth/odom --field pose.pose.position.x 2>/dev/null | head -1; }
drive_until() {   # drive forward until ground-truth x >= $1 or the robot stops moving (max 120 wall seconds)
  local end=$((SECONDS + 120)) last=-99
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" > /dev/null 2>&1 &
  local pub=$!
  while [ $SECONDS -lt $end ]; do
    x=$(gt_x); echo "  x=$x"
    if awk "BEGIN{exit !($x >= $1)}"; then break; fi
    if awk "BEGIN{exit !($x - $last < 0.01 && $x - $last > -0.01)}"; then break; fi
    last=$x
    sleep 3
  done
  kill $pub 2>/dev/null
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null 2>&1
}
echo "--- (1) open floor: drive to x = 0.5"
drive_until 0.5
sleep 1
timeout 4 ros2 topic echo --once /collisions
echo "--- (2) into the arena box (stalls at x = 1.15)"
drive_until 2.78
sleep 3
timeout 4 ros2 topic echo --once /collisions
echo "--- counter log"
grep -h COLLISION $L/m4_sim.log | head
cleanup
echo COLLISION_VERIFY_DONE
