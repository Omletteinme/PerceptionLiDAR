# PROJECT_CONTEXT.md — Reliability-Aware Localization and Path Planning for Sewer Inspection

**TL;DR.** A ROS 2 Jazzy + Gazebo Harmonic "digital twin" of a small differential-drive sewer-inspection robot (student robotics minor project, simulation only). The robot localizes itself with a 2D-LiDAR particle filter on a known wall map, in two variants: a baseline filter and a *reliability-aware* filter that down-weights LiDAR beams that look corrupted (water reflections, spray, dropouts). It detects unmapped debris blockages from the scans, plans an uncertainty-aware route on a topological graph, slows or stops when a "localization health" state says conditions are hostile, and drives with pure pursuit. A pure-NumPy core is shared by a 2D sandbox (tests, 30-seed experiments) and the ROS nodes. Everything works end to end in simulation (64 unit tests, live Gazebo missions pass on a demo seed). Honest status: reliability weighting helps only under heavy disturbance and is slightly worse in mild conditions; the live Gazebo evaluation is thin (single runs); nothing is validated on real data. Built over about a week (10-27 to 11-03, see the timeline in 11b). Work is in `~/sewer_ws` inside WSL2; one local git commit, further changes uncommitted, nothing pushed yet.

---

## 1. Overview
- **What:** simulated sewer robot that must go from entry A to goal J4 through a corridor network where some debris is NOT on the prior map. Demo goals: show robot parts, entering the sewer, sensors detecting walls/debris, the position estimate changing when readings become unreliable, and a safer route around a blockage. Two views: Gazebo 3D and RViz.
- **Who/why:** a university robotics minor project; originally planned as a research-style comparison (baseline vs reliability-aware localization) plus a demo.
- **Status:** functional prototype. All demo goals work live on the demo seed. Evaluation: 30-seed sandbox study done; Gazebo batch cut short by choice (4 finished missions). Final side-by-side demo video not yet recorded on the newest map. No real-world data.
- **Everything simulated vs custom:** Gazebo supplies physics, LiDAR/IMU, wheel odometry, ground-truth pose, contact sensors. All localization, perception, planning, health, disturbance injection and evaluation code is custom. Ground truth is used for scoring only, never by an algorithm.

## 2. Tech stack
- Host: Windows 11, **Ubuntu 24.04.4 LTS in WSL2** (WSLg shows GUI windows). All work inside the Linux filesystem (`~/sewer_ws`), never `/mnt/c`.
- **ROS 2 Jazzy**, **Gazebo Harmonic** (`gz sim` 8.15.0, `ros_gz` bridge), **Nav2** (only `nav2_map_server` + lifecycle manager are used; Nav2 planners/costmaps are NOT used), **RViz2**, `xacro`, `colcon`.
- Python 3.12.3; numpy 1.26.4, scipy 1.11.4, pandas 2.1.4, matplotlib 3.6.3, PyYAML 6.0.1, pytest. Packages come from apt (ROS-provided), no separate pip env.
- Media tooling: ffmpeg (x11grab, drawtext), Xvfb + software GL for headless RViz capture.
- Install script: `install_stack.sh` (apt: ros-jazzy-desktop, ros-gz, navigation2, nav2-bringup, xacro, tf2-tools, colcon, rosdep, numpy/pandas/matplotlib/scipy).

## 3. Architecture
Shared pure-NumPy core (no ROS imports) is imported by both the sandbox and thin ROS node wrappers.

```
Gazebo (world + robot + sensors)                      ground truth: /ground_truth/odom (scoring/RViz only)
   |  /scan /odom /imu   (ros_gz bridge)
   v
[scan_disturbance_node]  /scan -> /scan_disturbed       (spray, water, dropout, noise bursts)
[odom_disturbance_node]  /odom,/imu -> /odom_disturbed,/imu_disturbed (scale errors, gyro bias, wheel-slip zones)
   v
[localizer_node] x2 (namespaces /baseline and /reliability)
   publishes estimated_pose, particle_cloud, scan_reliability(_R), reliability_scan, localization_health, status_marker, map->odom TF
   v (estimated pose + reliability-gated scan)
[blockage_detector_node] -> /edge_gaps, /blockages      evidence grid + per-edge cross-section gap
   v
[route_planner_node] -> /route, /route_info             uncertainty-aware route cost on the corridor graph (2 Hz replans)
   v
[route_follower_node] -> /cmd_vel                       pure pursuit; speed x1.0/0.5/0 by health state
   v
Gazebo DiffDrive          [collision_counter_node] <- contact sensors -> /collisions (evaluation only)
```
- Sandbox (`sandbox/sim2d.py`) re-creates this in 2D without ROS/Gazebo: scripted trajectory, same disturbance modules, same filters, for fast tests and 30-seed experiments.
- Two filters share everything (motion model, resampler, RNG stream) except the per-beam reliability weights, so differences are attributable.

## 4. Directory structure (`~/sewer_ws`, git root)
```
PLAN.md                  decisions, milestones, results, honest limitations (368 lines; the project log)
CLAUDE.md                working rules for the AI coding assistant
install_stack.sh / verify_nav2_sample.sh
src/
  sewer_description/     urdf/sewer_robot.urdf.xacro  robot: chassis, 2 wheels, casters, IMU, 2D LiDAR, cameras, contact sensors
  sewer_gazebo/          scripts/gen_network.py (generator), worlds/*.sdf, maps/*.pgm|yaml, config/network_graph*.yaml, blockages_truth*.yaml, bridge.yaml
  sewer_localization/    particle_filter.py, reliability.py, health.py, disturbances.py, map_utils.py, localizer_node.py, *_disturbance_node.py
  sewer_perception/      blockage.py (EvidenceGrid), blockage_detector_node.py
  sewer_planning/        graph.py, observability.py, route_cost.py, pursuit.py, route_planner_node.py, route_follower_node.py
  sewer_eval/            collisions.py, metrics.py, collision_counter_node.py
  sewer_bringup/         launch/{robot_sim,localization,mission,view_b,demo}.launch.py, config/view_b.rviz
sandbox/                 sim2d.py (2D simulator), run_experiment.py, run_eval.py (30 seeds), analysis scripts, tests/ (pytest)
tests/                   live-run tooling: mission_verify.sh, mission_check.py, eval_batch.sh, demo_*.sh, scan_vs_map.py, ...
logs/                    outputs (git-ignored; ~0.5 GB: bags, frames, CSVs)
```
Windows-side deliverables: `C:\Users\jeevi\Desktop\RVU DOCS\Perception_Localisation\renders\` (3D clips and stills).

## 5. Key modules
- `particle_filter.py`: `PFParams` (n=500, beam_stride=6, sigma_hit=0.15, kappa=0.25 likelihood tempering, gyro_weight=0.8, wall_block=0.10 m, roughening), `ParticleFilter` (baseline: odometry/IMU motion model + AMCL-style likelihood field; `predict`, `update`, `estimate`), `ReliabilityAwareFilter(ParticleFilter)` (adds scorer, scan reliability `R`, per-beam `r`), `fuse_heading`.
- `reliability.py`: `ReliabilityScorer.score()`: per-beam r_i = geometric consistency x temporal consistency (previous scan compensated by odometry motion).
- `health.py`: `HealthMonitor` -> RELIABLE/DEGRADED/CRITICAL from smoothed R, scan-to-map fit residual, wheel-vs-gyro yaw-rate disagreement; hysteresis. It is a hostile-conditions detector, not a failure predictor (tuned on sandbox seeds 100-109).
- `disturbances.py`: scan disturbance schedule (`make_schedule`, `apply_disturbances`; time windows) and `OdometryErrorModel` + `make_slip_schedule` (per-run scale errors, gyro bias random walk, wheel-slip ZONES defined by route distance 10-13.5 m and 21.5-25.5 m). Seeded and shared by sandbox and ROS nodes.
- `blockage.py`: `EvidenceGrid` (log-odds on unexpected-short returns, gated by r >= 0.5), per-edge gap analysis; blocked if gap < 0.50 m.
- `graph.py`: `Graph` (loads network_graph yaml, `locate`). `route_cost.py`: `RouteModel.plan/waypoints`, `CostParams` (uncertainty weight w_unc=2.0, belief-roadmap-style covariance propagation using map observability from `observability.py`).
- `pursuit.py`: `pursuit_target` (aims at corners), `health_speed_factor` (1.0 / 0.5 / 0, creep 0.3x after 8 s CRITICAL).
- `collisions.py`: `partner_model`, `CollisionCounter` (floor ignored; one collision per obstacle model after >= 1 s of no contact). `metrics.py`: `pos_error`, `recovery_stats`, `track_metrics`, `mean_ci` (95 % Student-t).
- `gen_network.py`: builds everything from `NETWORKS` tables; `Decor` class enforces the LiDAR-safety rule for decorations.

## 6. Data models
- Occupancy map: PGM + YAML (5 cm, walls only, blockages NOT in the map). `OccupancyMap` has `raycast`, `dist`.
- Graph yaml: `nodes {name: x, y, type}`, `edges [{from,to,length}]`, `start`, `goal`, `corridor_width` 1.0.
- Truth yaml (evaluation only): `blockages {B1: edge, kind full|partial, free_gap, blocks_route, pieces[(box|cyl,...)]}`.
- Networks: `sewer_network` (nodes A,J1,D,J2,M,E,J3,J4; B1 full on J1-J4, B2 partial 0.6 m gap on J1-J2) and `sewer_network_v2` (adds node F(16,10), edges E-F and F-J3, B3 full on M-J3). File suffix rule: name minus `sewer_network` ('' or `_v2`). Every world is named `sewer_network` inside.
- JSON strings on topics: `localization_health` (state, score, per-signal stress), `edge_gaps`, `route_info`, `collisions` (total, wall, debris, other, contact_time, events).

## 7. APIs / interfaces
- Launch files (args): `robot_sim` (world, headless, x, y, yaw), `localization` (filter both|baseline|reliability, disturbed, severity, seed, n_particles, odom_disturbed, odom_severity, odom_scale, network), `mission` (use_reliability, goal, follow, w_unc, health_response, system reliability|baseline, network), `view_b` (rviz, network), `demo` (preset clean|water_debris|water_debris_slip, headless, network default v2, record, bag, seed).
- Topics: `/scan /odom /imu /ground_truth/odom /cmd_vel` (bridged), `/<ns>/estimated_pose`, `/<ns>/particle_cloud`, `/<ns>/localization_health`, `/reliability/reliability_scan`, `/route`, `/blockages`, `/edge_gaps`, `/mission_status`, `/collisions`, `/contacts/{chassis,left_wheel,right_wheel}`.
- Gazebo's contact sensors ignore a custom `<topic>`; `robot_sim.launch.py` builds bridge arguments from the world name (`test_arena` or `sewer_network`).
- CLI tools: `python3 sandbox/run_eval.py --seeds 30`, `python3 src/sewer_gazebo/scripts/gen_network.py --out src/sewer_gazebo [--network all|...] [--preview-dir d]`.

## 8. Configuration and environment
- No secrets or API keys in the project. Env vars used by scripts: `NETWORK` (sewer_network|sewer_network_v2), `CHK_T` (mission timeout, sim s), `LOC_ARGS`, `MISSION_ARGS`, `RATE`, `REC_T`; `DISPLAY`/`WAYLAND_DISPLAY` from WSLg.
- Fixed per `CLAUDE.md`: ROS Jazzy, Gazebo Harmonic, WSL2 Ubuntu, `use_sim_time:=true` everywhere, run headless for verification, clean up processes after every run.
- External: none (offline). GitHub remote `origin` = `https://github.com/Omletteinme/PerceptionLiDAR.git` (set, push not done: needs the user's GitHub credentials).

## 9. Set up, run, test, build, deploy (inside Ubuntu: `wsl -d Ubuntu`)
```bash
source /opt/ros/jazzy/setup.bash
cd ~/sewer_ws && colcon build --symlink-install && source install/setup.bash
# unit tests (no ROS/Gazebo needed), ~40 s, 64 tests
cd ~/sewer_ws/sandbox && python3 -m pytest -q tests
# live demo with windows (v2 map, disturbance seed 3)
ros2 launch sewer_bringup demo.launch.py preset:=water_debris_slip headless:=false
ros2 launch sewer_bringup view_b.launch.py network:=sewer_network_v2        # RViz in a second terminal
# one scripted live mission with pass/fail checks
NETWORK=sewer_network_v2 CHK_T=330 bash tests/mission_verify.sh myrun true 2 3   # args: label disturbed severity seed
# 30-seed sandbox evaluation -> logs/eval/sandbox/{results.csv,tables.md,rmse_vs_severity.png}
python3 sandbox/run_eval.py --seeds 30 --procs 8
# regenerate worlds/maps/graphs
python3 src/sewer_gazebo/scripts/gen_network.py --out src/sewer_gazebo
# record demo video pieces (about 20 min): tests/demo_all.sh water_debris_slip
```
Deployment: none (research/demo project). After pushing new files into `src/`, run `colcon build --symlink-install` again (new launch/world files are only linked at build time).

## 10. Conventions and quirks
- One module at a time; every module ships complete files, a build and a check that was actually run. Code comments mark "CUSTOM" vs "SIMULATED". Algorithms stay in pure-NumPy modules; ROS nodes are thin wrappers.
- Same seeds, world and disturbances for both filters; tuning seeds (100+) disjoint from reported seeds (0-29).
- Quirks that bite newcomers:
  - GPU LiDAR ray-casts **visual** geometry at z about 0.13 m: any visual in that band above free space becomes a phantom obstacle; decorations must be visual-only and above/below it. Beacon mast sits above the scan plane.
  - Collision shapes of any new model would be counted as collisions (`sewer_eval/collisions.py` categorises by model-name prefix `wall*`, `debris*`, else `other`).
  - Robot chase camera starts behind the end wall at A (first 2.5 s of frames are skipped).
  - DDS discovery is flaky: `ros2 daemon stop` between runs. **Stale nodes survive stopped launches** (cost one hour once): check `ps -eo pid,etime,cmd | grep sewer_ws/install` before each test. `pkill -f <pattern>` inside `bash -c "...<pattern>..."` kills your own shell: put commands in a script file.
  - Files staged from Windows can get CRLF line endings, which break shell scripts: strip with `tr -d '\r'`.
  - `gz sim` can hang: `pkill -9 -f "gz sim"` fallback is in the scripts.
  - Particle filter details that matter: noise floors only while moving; roughening floors 5 mm / 0.1 deg; particles may not end within 0.10 m of a wall (`wall_block`), otherwise odometry overshoot hijacks the estimate.

## 11. Current state
- **Done & verified:** robot, two networks, both filters (sandbox + ROS), scan and odometry disturbances, reliability scoring, blockage detection, uncertainty-aware planner/follower, health state + speed response, contact-sensor collision counting, RViz view, richer visuals, demo launch presets, 64 passing tests.
- **Results (sandbox, 30 seeds per condition, mean ± 95 % CI, full tables in PLAN.md):** dead reckoning RMSE 0.31 m (clean) to 0.62 m (x5). Clean/x1/x2: both filters hold the estimate (0/30 runs exceed 0.5 m), baseline slightly better by 5-10 mm (x2: 0.097 vs 0.107 m). x3: equal in mean. x4: reliability-aware better (1.25 vs 1.92 m, better in 24/30 seeds) but both lose track. x5: both lost. Slip is the cause of the mild-condition penalty (with scan disturbances only, reliability-aware wins 9/10 seeds); the exact mechanism is not isolated.
- **Live (Gazebo) runs:** classic clean ALL PASS (goal in 83 s, 0 collisions); v2 clean ALL PASS (2 blockages detected, 2 replans, 108 s); v2 with scan x2 + slip x2, seed 3 ALL PASS (139 s, 0 collisions). Counterexamples kept: seed 21 (same severity) lost the estimate near debris B2, ended NO_ROUTE and scraped walls for 207 s; seed 1 touched a wall for 0.2 s in the narrow B2 gap. Single runs, no live statistics.
- **Not done:** Gazebo batch evaluation (only 4 of 40 planned missions), demo video recorded on v2 (scripts `tests/demo_record.sh`, `demo_viewb.sh`, `demo_compose.py`, `demo_all.sh` are ready), PLAN.md final close-out/run instructions, real-data validation, cleanup of `src/sewer_gazebo/worlds/tmp_*.sdf` (5 temporary files from a LiDAR investigation; untracked, must be deleted before committing).
- **Git:** local commit `1b30ee6` plus uncommitted work (v2 files, launch changes, tests, PLAN.md); remote not pushed. No TODO/FIXME comments exist in the code.
- **Known limitations/debt:** health state does not predict divergence (most CRITICAL time is false alarm); blockage B2 evidence is fragile under heavy water disturbance (6-7 cells vs min cluster 6); evidence grid is polluted when localization error is large; the compound heavy scan + slip case defeats both filters; two papers cited in the user's source PDF could not be verified.

## 11b. Development timeline (chronological; dates from PLAN.md, 2026)
The project was built in roughly one week of AI-assisted sessions. A 14-day roadmap (Days 1-14) was planned; Days 1-10 were done as planned, Days 11-14 were re-planned on 11-01.

**10-27 (Days 1-8): environment, robot, network, filters**
- Day 1: used the existing WSL2 Ubuntu 24.04.4; installed ROS 2 Jazzy, Gazebo Harmonic (gz sim 8.15), Nav2, colcon via apt; passwordless sudo enabled by the user. Finding: the Nav2 sample bridges `/cmd_vel` as `Twist` (not `TwistStamped`).
- Days 2-3: robot model (chassis 0.30 x 0.18 x 0.08 m, 4 kg; wheel r 0.06 m; track 0.26 m; two ball casters), 2D LiDAR (360 beams, 0.12-8 m, 10 Hz, sigma 0.01 m), IMU, DiffDrive odometry, bridge config, `test_arena.sdf`. Findings: a flat-cylinder wheel collision scrubbed when pivoting (real track 0.219 m), fixed by sphere wheel collisions; wheel odometry has no slip model.
- Days 4-5: sewer network generator `gen_network.py` (box-section corridors, 1 m wide, 8 nodes, blockages B1 full on J1-J4 and B2 partial 0.6 m gap on J1-J2); one source of truth for world, map, graph and blockage truth. Finding: corridors are featureless, so the along-corridor position is weakly observable between junctions.
- Days 6-8: baseline particle filter, then the reliability-aware filter, first in the 2D sandbox (`sim2d.py`) with tests, sharing a pure-NumPy core. Preliminary result (5 seeds): dead reckoning 0.376 m RMSE; filters ~0.035 m when clean; reliability-aware 5-20 % better at x2-x4, large gain only where the baseline lost track (x4: 0.170 vs 0.050 m, outlier-driven).

**10-28: ROS wrappers and RViz**
- `localizer_node` (both filters), `scan_disturbance_node`, launch files; live Gazebo results: wheel odometry drifted only ~0.19 m RMSE, filters ~0.07 m, no baseline failure even at x4, so live runs did not yet show a filter gap (motivated the later odometry error model).
- View B (RViz) built and verified headlessly with Xvfb screenshots. Bug found and fixed: a parked robot random-walked 0.28 m because motion-noise floors applied while idle; noise floors now apply only while moving (regression test, 13 tests passing). Known issue noted: particle sigma overconfident.

**10-29 (Days 9-10): perception, planning, live missions**
- Added `sewer_perception` (evidence-grid blockage detector with cluster filter) and `sewer_planning` (graph, map-observability, uncertainty-aware route cost, pure-pursuit follower). Deviation from the original plan: Nav2 planners/costmaps are not used.
- Results: the cluster filter, not reliability weighting, prevents phantom blockages; real debris found (B1 blocked, B2 passable, gap 0.55-0.60 m). 10 + 9 live missions reached the goal; B1 detected about 15-17 s in, replan 0.5 s later.
- Bugs fixed: `Graph.locate` chose the wrong corridor beside junctions (robot hunted 5-15 s); pure pursuit cut corners; two bugs in the mission checker. A hung `gz sim` taught the scripts a SIGKILL fallback.

**10-30: 3D renders (View A)**
- Cameras added (robot, chase, oblique, top-down); headless render pipeline produced `mission_3d.mp4`, camera clips and stills in `renders/`.

**10-30 to 11-01: reassessment (exact dates not recorded)**
- The user asked whether the work followed a reference PDF paper; a novelty check was done; two cited papers could not be verified. The user then asked to focus the plan on the actual project (the demo), and the paper-driven items (learned reliability combiner, extra cues, strong-baseline suite, real data, novelty write-up) were set aside.

**11-01: revised plan approved; M1-M3 done**
- M1: roughening after resampling (so sigma is credible; first floors 2 cm/0.5 deg made the baseline lose track and were reduced to 5 mm/0.1 deg). Correction: the earlier headline "0.050 vs 0.170 m at x4" was mostly a baseline particle-depletion failure. A planned change (blockage detector ignores reliability) failed in live A/B (phantom blockages, NO_ROUTE) and was reversed.
- M2: `OdometryErrorModel` (scale errors, gyro bias walk, wheel-slip zones) + `odom_disturbance_node`; dead reckoning now drifts 0.7-0.9 m. Bug found: odometry overshoot pushed particles into walls and the estimate ended inside a wall; fixed with `wall_block`. Compound x3 scan + x3 slip case defeats both filters.
- M3: `health.py` (R, scan-fit, wheel-vs-gyro signals) with follower response (0.5x / stop / creep). Calibration showed no signal predicts divergence (lost filters are confidently wrong), so it is documented as a hostile-conditions detector. Slip zones converted from time windows to route distances so slowing down cannot shorten slip exposure. Live runs on 3 seeds: state changed in the disturbance windows, goal reached each time; one run touched a wall at the B2 gap.

**11-02: M4, M5, M6 tooling, world upgrade, GitHub**
- M4: contact sensors + `sewer_eval` collision counter. Gotcha: Gazebo ignores a custom contact-sensor topic; bridge built per world name. Verified in the arena and in missions.
- M5: shared metrics (`sewer_eval/metrics.py`) and the 30-seed sandbox evaluation (results in section 11). A 40-mission Gazebo batch was started and then cut at 4 finished runs because of its ~3.5 h duration.
- M6: demo launch presets, rosbag recording, RViz replay capture, video composer written and tested on stand-in clips. First demo recording with seed 21 failed (estimate lost near B2, walls scraped 207 s), kept as a documented failure; demo seed changed to 3. The recording chain was then stopped by the user ("not now").
- Local git repository created with one commit; remote set to the user's GitHub repository, push pending the user's credentials.
- World upgrade: richer visuals plus `sewer_network_v2` (second loop through F, blockage B3 on M-J3); decorations restricted to visual-only geometry outside the LiDAR band; generalised mission checker; 64 tests; live classic and v2 missions pass. Debugging lesson: stale nodes from a stopped run drove the parked robot away for an hour.

**11-03:** this context file written; v2 demo video still not re-recorded; Gazebo batch evaluation and PLAN.md close-out remain.

## 12. Open questions and constraints
- Do not change the fixed environment (ROS Jazzy, Gazebo Harmonic, WSL2, no `/mnt/c` work) without asking. Do not delete outside `~/sewer_ws`. Keep ground truth evaluation-only.
- Pending decisions: how many Gazebo seeds to run for the live evaluation (user declined the ~3 h batch for now); when to record the final demo video; whether to isolate the slip penalty of reliability weighting; what to publish on GitHub (logs, bags and videos are excluded by `.gitignore`).
- Claims the data support: reliability weighting helps in the hostile regime (significant at x4), costs a few mm in mild conditions. Claims it does NOT support: general superiority, failure prediction.
- Performance: Gazebo runs about real time with software rendering; a mission takes 3-5 wall minutes including about 45 s of startup; the sandbox suite takes about 40 s.

## 13. How to help me
You are helping with the project described above. You cannot see the code, so ask for specific files or snippets (for example `particle_filter.py` or `route_cost.py`) before changing them. Match the existing conventions: pure-NumPy core modules with thin ROS wrappers, "CUSTOM vs SIMULATED" labels, seeded and repeatable experiments, ground truth used only for scoring. Give complete, working code (no placeholders), say which commands to run and what output to expect, and be explicit about what you have not verified. Be honest about negative results; do not overstate what the evaluation shows.
