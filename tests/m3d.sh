#!/bin/bash
cd ~/sewer_ws
for s in 2 3; do
  CHK_T=330 LOC_ARGS='odom_disturbed:=true odom_severity:=2' bash tests/mission_verify.sh m3d$s true 2 $s > logs/m3d$s.out 2>&1
done
grep -E 'end:|position error|health:|FAIL|RESULT' logs/m3d2.out logs/m3d3.out
