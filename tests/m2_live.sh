#!/bin/bash
# M2 live verification: disturbed odometry/IMU with clean and disturbed scans.
cd ~/sewer_ws/tests
L=~/sewer_ws/logs
run() { LOC_ARGS="$4" bash mission_verify.sh $1 $2 $3 0 $4 > /dev/null 2>&1; }
runit() {  # label scan_disturbed scan_severity "loc args"
  LOC_ARGS="$4" bash mission_verify.sh $1 $2 $3 0 > $L/mission_verify_$1.log 2>&1
}
runit o1c false 1.0 "odom_disturbed:=true odom_severity:=1.0"
runit o3c false 1.0 "odom_disturbed:=true odom_severity:=3.0"
runit o3x true 3.0 "odom_disturbed:=true odom_severity:=3.0"
for l in o1c o3c o3x; do
  echo "== $l"
  grep -E 'end: status|position error vs|obstruction clusters|FAIL|RESULT' $L/mission_verify_$l.log | cut -c1-260
done
python3 analyze_spells.py o1c o3c o3x
echo M2_LIVE_DONE
