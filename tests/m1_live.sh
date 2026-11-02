#!/bin/bash
# M1 live smoke test: rebuild, then a clean and a 3x-disturbed mission through the real pipeline.
source /opt/ros/jazzy/setup.bash
cd ~/sewer_ws
colcon build --symlink-install --packages-select sewer_localization sewer_perception sewer_planning sewer_bringup 2>&1 | grep -E "Finished|Failed|error"
cd tests
L=~/sewer_ws/logs
bash mission_verify.sh clean false 1.0 0 > $L/mission_verify_clean.log 2>&1
bash mission_verify.sh x3 true 3.0 0 > $L/mission_verify_x3.log 2>&1
for l in clean x3; do
  echo "== $l: $(grep -E 'end: status' $L/mission_verify_$l.log | cut -c1-140)"
  grep -E 'obstruction clusters|FAIL|RESULT' $L/mission_verify_$l.log | cut -c1-230
done
python3 analyze_spells.py clean x3
echo M1_LIVE_DONE
