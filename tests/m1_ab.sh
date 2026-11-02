#!/bin/bash
# Live A/B of the blockage detector: all valid beams (u) vs reliability-gated (g), 3x disturbance, two disturbance seeds, interleaved.
cd ~/sewer_ws/tests
L=~/sewer_ws/logs
run() { MISSION_ARGS="$2" bash mission_verify.sh $1 true 3.0 $3 > $L/mission_verify_$1.log 2>&1; }
run u0 "use_reliability:=false" 0
run g0 "use_reliability:=true" 0
run u1 "use_reliability:=false" 1
run g1 "use_reliability:=true" 1
for l in u0 g0 u1 g1; do
  echo "== $l: $(grep -E 'end: status' $L/mission_verify_$l.log | cut -c1-100)"
  grep -E 'obstruction clusters|FAIL|RESULT' $L/mission_verify_$l.log | cut -c1-200
done
python3 analyze_spells.py u0 g0 u1 g1
echo AB_DONE
