#!/bin/bash
# Compare roughening variants on the tuning seeds, in parallel (one process per variant).
cd ~/sewer_ws/sandbox
L=~/sewer_ws/logs
python3 check_sigma.py 5 0 0 0          > $L/sigma_v0.log 2>&1 &
python3 check_sigma.py 5 0.2 0 0        > $L/sigma_v1.log 2>&1 &
python3 check_sigma.py 5 0.2 0.005 0.0017 > $L/sigma_v2.log 2>&1 &
python3 check_sigma.py 5 0.1 0 0        > $L/sigma_v3.log 2>&1 &
wait
for v in 0 1 2 3; do echo "----- variant $v"; cat $L/sigma_v$v.log; done
echo VARIANTS_DONE
