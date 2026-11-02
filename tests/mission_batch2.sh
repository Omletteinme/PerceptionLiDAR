#!/bin/bash
cd ~/sewer_ws/tests
L=~/sewer_ws/logs
run() { bash mission_verify.sh $1 $2 $3 $4 > $L/mission_verify_$1.log 2>&1; }
run clean false 1.0 0
run x3s2 true 3.0 2
run x4s0 true 4.0 0
run x4s1 true 4.0 1
run x3 true 3.0 0
for l in clean x3s2 x4s0 x4s1 x3; do
  echo "== $l: $(grep -E 'end: status' $L/mission_verify_$l.log | cut -c1-150)"
  grep -E 'FAIL|RESULT' $L/mission_verify_$l.log
done
python3 analyze_spells.py clean x3s2 x4s0 x4s1 x3
echo BATCH2_DONE
