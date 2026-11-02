#!/bin/bash
cd ~/sewer_ws/tests
L=~/sewer_ws/logs
run() { bash mission_verify.sh $1 $2 $3 $4 > $L/mission_verify_$1.log 2>&1; }
run clean false 1.0 0
run x3 true 3.0 0
run x3s1 true 3.0 1
for l in clean x3 x3s1; do
  echo "== $l: $(grep -E 'end: status' $L/mission_verify_$l.log | cut -c1-120)"
  grep -E 'obstruction clusters|FAIL|RESULT' $L/mission_verify_$l.log | cut -c1-260
done
echo BATCH3_DONE
