source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
L=~/sewer_ws/logs
pkill -f "gz sim"; pkill -f ros2; sleep 2
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False use_sim_time:=true > $L/nav2_sample.log 2>&1 &
sleep 75
echo "== clock t1"; timeout 10 ros2 topic echo --once /clock --field clock.sec
sleep 3
echo "== clock t2"; timeout 10 ros2 topic echo --once /clock --field clock.sec
echo "== odom before"; timeout 10 ros2 topic echo --once /odom --field pose.pose.position.x
echo "== drive 0.2 m/s for 8s (Twist)"; timeout 8 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}' > /dev/null 2>&1
echo "== odom after"; timeout 10 ros2 topic echo --once /odom --field pose.pose.position.x
pkill -f "gz sim"; pkill -f ros2; sleep 2; echo VERIFY_DONE
