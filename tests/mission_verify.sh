#!/bin/bash
# Live mission in Gazebo: sim + both localizers (optionally disturbed) + perception + planner + follower.
# Usage: mission_verify.sh <label> <disturbed true|false> <severity>
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
L=~/sewer_ws/logs
label=${1:-clean}; dist=${2:-false}; sev=${3:-1.0}; seed=${4:-0}
cleanup() {
  pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f ros2; pkill -f parameter_bridge; pkill -f localizer_node; pkill -f scan_disturbance_node; pkill -f odom_disturbance_node; pkill -f collision_counter_node; pkill -f contact_bridge
  pkill -f blockage_detector_node; pkill -f route_planner_node; pkill -f route_follower_node; pkill -f mission_check
  sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1
}
cleanup
ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=sewer_network.sdf x:=0.0 y:=0.0 yaw:=0.0 > $L/m_sim_$label.log 2>&1 &
sleep 35
ros2 launch sewer_bringup localization.launch.py filter:=both disturbed:=$dist severity:=$sev seed:=$seed $LOC_ARGS > $L/m_loc_$label.log 2>&1 &
sleep 12
mkdir -p $L/day9
python3 ~/sewer_ws/tests/mission_check.py ${CHK_T:-230} $L/day9/traj_$label.csv > $L/m_check_$label.log 2>&1 &
CHK=$!
sleep 3
ros2 launch sewer_bringup mission.launch.py $MISSION_ARGS > $L/m_mission_$label.log 2>&1 &
wait $CHK
echo "=================== mission $label (disturbed=$dist severity=$sev)"
cat $L/m_check_$label.log
echo "-- node errors:"
grep -ihE 'error|traceback|exception' $L/m_mission_$label.log $L/m_loc_$label.log | cut -c1-170 | sort | uniq -c | head -8
cleanup
echo MISSION_VERIFY_DONE
