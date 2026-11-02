#!/bin/bash
# Usage: run_diag.sh <script.py> [args...]  -- launches the sim headless, runs the script, cleans up.
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
L=~/sewer_ws/logs
pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f ros2; pkill -f parameter_bridge; sleep 2
ros2 launch sewer_bringup robot_sim.launch.py headless:=true > $L/robot_sim.log 2>&1 &
sleep 35
python3 "$@"
pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f ros2; pkill -f parameter_bridge; sleep 2
echo DIAG_DONE
