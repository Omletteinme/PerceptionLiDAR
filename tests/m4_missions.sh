#!/bin/bash
cd ~/sewer_ws
CHK_T=330 bash tests/mission_verify.sh m4a false 1 1 > logs/m4a.out 2>&1
CHK_T=330 LOC_ARGS='odom_disturbed:=true odom_severity:=2' bash tests/mission_verify.sh m4b true 2 1 > logs/m4b.out 2>&1
grep -E 'end:|health:|no collision|RESULT|FAIL' logs/m4a.out logs/m4b.out
