#!/bin/bash
# Day 2-3 verification: launches the sim headless, checks topics/rates/TF, runs day2_robot_check.py, cleans up.
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
L=~/sewer_ws/logs
pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f ros2; pkill -f parameter_bridge; sleep 2

ros2 launch sewer_bringup robot_sim.launch.py headless:=true > $L/robot_sim.log 2>&1 &
sleep 35

echo "== topics"
ros2 topic list | grep -E '^/(clock|cmd_vel|odom|imu|scan|tf|joint_states|ground_truth/odom)$' | sort
echo "== rates"
for t in /scan /imu /odom; do
  echo -n "$t: "; timeout 12 ros2 topic hz $t --window 30 --use-sim-time 2>&1 | grep 'average rate' | tail -1
done
echo "== frames"
ros2 run tf2_ros tf2_echo odom lidar_link --ros-args -p use_sim_time:=true > $L/tf_echo.log 2>&1 &
TFPID=$!
sleep 6; kill $TFPID 2>/dev/null
grep -m1 -A3 'At time' $L/tf_echo.log
echo "-- view_frames"
(cd $L && timeout 30 ros2 run tf2_tools view_frames --ros-args -p use_sim_time:=true > view_frames.out 2>&1; ls frames_*.gv 2>/dev/null | tail -1 | xargs -r grep -E '"[a-z_]+" -> "[a-z_]+"')
echo "== robot check"
python3 ~/sewer_ws/tests/day2_robot_check.py
echo "== log errors"
grep -iE 'error|failed|exception' $L/robot_sim.log | cut -c1-200 | sort | uniq -c | head
pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f ros2; pkill -f parameter_bridge; sleep 2
echo VERIFY_DONE
