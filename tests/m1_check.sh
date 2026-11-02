#!/bin/bash
cd ~/sewer_ws/sandbox
echo "=== detector tests (default = every valid beam)"
python3 -m pytest -q tests/test_nav.py 2>&1 | tail -4
echo "=== sigma credibility AFTER roughening (tuning seeds 100-104)"
python3 check_sigma.py 5
echo M1_CHECK_DONE
