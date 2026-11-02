#!/bin/bash
# M5 Gazebo batch: conditions x systems x seeds, one live mission each (tests/mission_verify.sh), resumable.
#   conditions: clean | mild (scan x2, wheel slip x2) | hard (scan x3, wheel slip x3)
#   systems:    rel   reliability-aware stack, health response on (default of mission.launch.py)
#               relnh reliability-aware stack, health_response:=false      (not run on clean: the state never leaves RELIABLE)
#               base  system:=baseline health_response:=false              (baseline pose everywhere, no scan reliability)
# Usage: eval_batch.sh "11 12 13 14 15"      (disturbance seeds; scan seed = seed, odometry seed = seed + 5000)
# A finished run leaves logs/day9/traj_<cond>-<sys>-<seed>_summary.json and is skipped on a re-run.
cd ~/sewer_ws
L=logs
SEEDS=${1:-"11 12 13 14 15"}
for seed in $SEEDS; do
  for cond in clean mild hard; do
    case $cond in
      clean) dist=false; sev=1; LOC="" ;;
      mild)  dist=true;  sev=2; LOC="odom_disturbed:=true odom_severity:=2" ;;
      hard)  dist=true;  sev=3; LOC="odom_disturbed:=true odom_severity:=3" ;;
    esac
    for sys in rel relnh base; do
      if [ $cond = clean ] && [ $sys = relnh ]; then continue; fi
      label=$cond-$sys-$seed
      if [ -f $L/day9/traj_${label}_summary.json ]; then continue; fi
      case $sys in
        rel)   MA="" ;;
        relnh) MA="health_response:=false" ;;
        base)  MA="system:=baseline health_response:=false" ;;
      esac
      echo "$(date +%T) start $label"
      CHK_T=330 LOC_ARGS="$LOC" MISSION_ARGS="$MA" bash tests/mission_verify.sh $label $dist $sev $seed > $L/eval_$label.out 2>&1
      echo "$(date +%T) done  $label: $(grep -E 'RESULT' $L/eval_$label.out)"
    done
  done
done
echo EVAL_BATCH_DONE
