#!/bin/bash
# Account 2 live Gazebo checks, headless. Run from WSL2 after building the workspace:
#   bash ~/sewer_ws/tests/account2_live_check.sh topics     # stage 1: sensors, arm and bridges are alive
#   bash ~/sewer_ws/tests/account2_live_check.sh pick       # stage 2: detect -> pick -> carry -> drop -> verify
#   bash ~/sewer_ws/tests/account2_live_check.sh mission    # stage 3: full mission with Account 1's Mission Manager
#   bash ~/sewer_ws/tests/account2_live_check.sh dataset    # record a labelled dataset while the mission runs
# Logs and results: ~/sewer_ws/logs/account2/
source /opt/ros/jazzy/setup.bash
source ~/sewer_ws/install/setup.bash
L=~/sewer_ws/logs/account2
mkdir -p $L
mode=${1:-topics}
cleanup() {
  pkill -f "gz sim"; sleep 1; pkill -9 -f "gz sim"; pkill -f parameter_bridge; pkill -f robot_state_publisher
  pkill -f localizer_node; pkill -f collision_counter_node; pkill -f inspection_node; pkill -f manipulation_node
  pkill -f scan_self_filter_node; pkill -f dataset_recorder_node; pkill -f account2_pick_place_check
  pkill -f mission_manager_node; pkill -f route_follower_node; pkill -f blockage_detector_node; pkill -f account2_mission_check
  sleep 2; ros2 daemon stop > /dev/null 2>&1; sleep 1
}
cleanup
ros2 launch sewer_manipulation inspection_sim.launch.py headless:=true > $L/sim.log 2>&1 &
sleep 35
ros2 launch sewer_bringup localization.launch.py filter:=both network:=sewer_network_v2 > $L/loc.log 2>&1 &
sleep 12
REC=false; [ "$mode" = "dataset" ] && REC=true
ros2 launch sewer_manipulation inspection.launch.py record:=$REC > $L/nodes.log 2>&1 &
sleep 10

if [ "$mode" = "topics" ]; then
  echo "== topics (expect robot_depth, robot_cam/image, contacts/left_finger, perception/*, manipulation/status)"
  ros2 topic list | grep -E "robot_depth|robot_cam|contacts|perception|manipulation|disposal|arm/|sim/dynamic" | sort
  echo "== rates (depth ~5 Hz, rgb up to 10 Hz)"
  timeout 8 ros2 topic hz /robot_depth 2>&1 | grep -m1 average
  timeout 8 ros2 topic hz /robot_cam/image 2>&1 | grep -m1 average
  echo "== depth image (expect encoding 32FC1, 320x180)"
  timeout 10 ros2 topic echo --once --no-arr /robot_depth 2>&1 | grep -E "height|width|encoding"
  echo "== arm joints in /joint_states (expect 7 arm/finger names, incl. arm_wrist_yaw_joint)"
  timeout 8 ros2 topic echo /joint_states 2>&1 | grep -E "arm_|finger" | sort -u | head -8   # wheels/arm are separate msgs
  echo "== manipulation status (expect state IDLE)"
  timeout 10 ros2 topic echo --once --full-length /manipulation/status 2>&1 | head -3
  echo "== dynamic poses (optional evidence; expect child_frame_id debris_S1 ...)"
  timeout 10 ros2 topic echo /sim/dynamic_poses 2>&1 | grep -m3 debris_
  echo "   (Gazebo side, names published on dynamic_pose/info:)"
  timeout 8 gz topic -e -t /world/sewer_network/dynamic_pose/info -n 1 2>&1 | grep -m8 'name:' | sort -u
  echo "== perception (after the robot has looked down the corridor from A: expect D1 near x 2.5)"
  timeout 10 ros2 topic echo --once --full-length /perception/debris 2>&1 | head -c 600; echo
elif [ "$mode" = "pick" ]; then
  python3 ~/sewer_ws/tests/account2_pick_place_check.py ${CHK_T:-400} $L > $L/check.log 2>&1
  cat $L/check.log
  echo "== score of the final perception messages (inspection_scoring.py; only S1's corridor was driven)"
  python3 ~/sewer_ws/sandbox/inspection_scoring.py $L/final_debris.json $L/final_defects.json --out $L/score.json \
    | python3 -c "import json,sys; r=json.load(sys.stdin); print({k: r['debris'][k] for k in ('precision','recall','false_positives','misses')}); print({k: r['defects'][k] for k in ('precision','recall','false_positives','misses')})"
  echo "== evidence images"; ls ~/sewer_ws/logs/evidence | tail -5
else
  # full mission: Account 1's Mission Manager (+ blockage detector + route follower) drives; Account 2 perceives and grasps
  ros2 launch sewer_bringup mission_manager.launch.py network:=sewer_network_v2 > $L/mission.log 2>&1 &
  python3 ~/sewer_ws/tests/account2_mission_check.py ${MISSION_T:-2400} $L 2>&1 | tee $L/mission_check.log
  echo "== score of the final perception messages (objects already removed are no longer listed)"
  python3 ~/sewer_ws/sandbox/inspection_scoring.py $L/final_debris.json $L/final_defects.json --out $L/score.json \
    | python3 -c "import json,sys; r=json.load(sys.stdin); print({k: r['debris'][k] for k in ('precision','recall','false_positives','misses')}); print({k: r['defects'][k] for k in ('precision','recall','false_positives','misses')})"
  if [ "$mode" = "dataset" ]; then
    python3 ~/sewer_ws/sandbox/label_dataset.py ~/sewer_ws/logs/dataset
    python3 ~/sewer_ws/sandbox/run_inspection_eval.py --dataset ~/sewer_ws/logs/dataset
  fi
fi
echo "== node errors:"
grep -ihE 'error|traceback|exception' $L/*.log 2>/dev/null | cut -c1-170 | sort | uniq -c | head -10
cleanup
echo ACCOUNT2_CHECK_DONE
