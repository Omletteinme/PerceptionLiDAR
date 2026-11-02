#!/bin/bash
# Live verification of the ROS localizers in Gazebo.
# Usage: ros_verify.sh <label> <disturbed true|false> <severity> <max_rmse>   (run several in sequence)
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
L=~/sewer_ws/logs
T=~/sewer_ws/tests
cleanup() { pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f ros2; pkill -f parameter_bridge; pkill -f localizer_node; pkill -f scan_disturbance_node; sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1; }

run() {
  label=$1; dist=$2; sev=$3; maxr=$4
  cleanup
  ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=sewer_network.sdf x:=0.0 y:=0.0 yaw:=0.0 > $L/ros_sim_$label.log 2>&1 &
  sleep 35
  ros2 launch sewer_bringup localization.launch.py filter:=both disturbed:=$dist severity:=$sev seed:=0 > $L/ros_loc_$label.log 2>&1 &
  sleep 10
  echo "=================== $label (disturbed=$dist severity=$sev)"
  echo "-- nodes:"
  ros2 node list | grep -E 'localizer|scan_disturbance' | sort | tr '\n' ' '; echo
  python3 $T/ros_localizer_check.py $maxr
  echo "-- localizer log warnings/errors:"
  grep -iE 'error|traceback|exception|warn' $L/ros_loc_$label.log | cut -c1-160 | sort | uniq -c | head -5
}

for spec in "$@"; do run $(echo $spec | tr ',' ' '); done
cleanup
echo VERIFY_DONE
