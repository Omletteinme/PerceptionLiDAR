#!/bin/bash
# Classic regression (clean) + v2 clean + v2 with disturbances (scan x2, slip x2, seed 3).
cd ~/sewer_ws
export CHK_T=330
NETWORK=sewer_network     bash tests/mission_verify.sh reg-classic   false 1 1 > logs/v2_reg_classic.out 2>&1
NETWORK=sewer_network_v2  bash tests/mission_verify.sh v2-clean      false 1 1 > logs/v2_clean.out 2>&1
NETWORK=sewer_network_v2  LOC_ARGS='odom_disturbed:=true odom_severity:=2' bash tests/mission_verify.sh v2-dist3 true 2 3 > logs/v2_dist3.out 2>&1
for f in reg_classic clean dist3; do
  echo "##### $f"; grep -E 'network |end:|health:|PASS|FAIL|RESULT' logs/v2_$f.out | cut -c1-230
done
