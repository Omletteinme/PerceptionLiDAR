#!/bin/bash
# Repeat live missions with different disturbance seeds/severities and summarise.
cd ~/sewer_ws/tests
L=~/sewer_ws/logs
run() { bash mission_verify.sh $1 $2 $3 $4 > $L/mission_verify_$1.log 2>&1; }
run x3s1 true 3.0 1
run x3s2 true 3.0 2
run x4s0 true 4.0 0
run x4s1 true 4.0 1
for l in x3s1 x3s2 x4s0 x4s1; do
  echo "== $l"
  grep -E "RESULT|FAIL|end: status" $L/mission_verify_$l.log | cut -c1-170
done
echo BATCH_DONE
