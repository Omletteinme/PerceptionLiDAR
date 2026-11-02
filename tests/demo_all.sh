#!/bin/bash
# Wait for the running batch mission to end, then: build, record the demo preset, replay into RViz, compose the video.
# Usage: demo_all.sh [preset]
PRESET=${1:-water_debris_slip}
W=~/sewer_ws
source /opt/ros/jazzy/setup.bash
while pgrep -f mission_verify.sh > /dev/null; do sleep 5; done
echo "$(date +%T) batch mission finished; building"
cd $W && colcon build --symlink-install 2>&1 | tail -2
echo "$(date +%T) recording $PRESET"
bash $W/tests/demo_record.sh $PRESET > $W/logs/demo_record_$PRESET.out 2>&1
tail -25 $W/logs/demo_record_$PRESET.out
echo "$(date +%T) View B replay"
bash $W/tests/demo_viewb.sh $PRESET > $W/logs/demo_viewb_$PRESET.out 2>&1
tail -8 $W/logs/demo_viewb_$PRESET.out
echo "$(date +%T) compose"
python3 $W/tests/demo_compose.py $W/logs/demo/$PRESET 2>&1 | tail -5
echo DEMO_ALL_DONE
