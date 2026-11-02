#!/bin/bash
# M1 sandbox verification: all unit tests, then the full experiment again, compared with the earlier results.
cd ~/sewer_ws/sandbox
echo "=== unit tests"
python3 -m pytest -q tests 2>&1 | tail -4
echo "=== sigma credibility with the final defaults (tuning seeds 100-104)"
python3 check_sigma.py 5
echo "=== full experiment (reported seeds 0-4), same as Day 6"
python3 run_experiment.py --seeds 5 --severities 1,2,3,4 --out ../logs/m1 > ../logs/m1_run.log 2>&1
python3 - <<'PY'
import pandas as pd
new = pd.read_csv("../logs/m1/results.csv"); old = pd.read_csv("../logs/day6_after_fix/results.csv")
a = old.groupby(["scenario", "filter"]).rmse.mean().rename("rmse_before"); b = new.groupby(["scenario", "filter"]).rmse.mean().rename("rmse_after")
t = pd.concat([a, b], axis=1)
t = t[t.index.get_level_values("filter") != "dead_reckoning"]
print(t.round(3).to_string())
PY
echo M1_FINAL_DONE
