# PLAN.md — Sewer-Inspection Robot Digital Twin

## Goal

Build a **3D digital twin** of a sewer-inspection robot in professional robotics simulation software (not a 2D animation). The demo must let someone:

1. See the robot's physical components (chassis, wheels, LiDAR, IMU placement).
2. Watch it enter an underground sewer network.
3. See its sensors detect walls and debris.
4. See its **estimated position change when readings become unreliable**.
5. Watch it **select a safer route around a blockage**.

Priority: a technically credible digital twin with real robotics behind it, not a visually impressive scene with nothing underneath.

## Research contribution

A **reliability-aware particle filter** that fuses IMU, wheel odometry and 2D LiDAR, and weights each range measurement by its **temporal and geometric consistency**, reducing the influence of unreliable readings caused by water, debris and sensor noise. Compared against a **baseline** (standard particle filter or AMCL) under identical conditions.

Also custom: sensor-consistency scoring, blockage detection, uncertainty estimation, uncertainty-aware route cost, and the experiments.

Division of labour:
- **Gazebo** simulates the robot and world.
- **ROS 2** connects the systems.
- **Our Python algorithms** make the project original.

## Tech stack

| Layer | Tool | Role |
|---|---|---|
| Physics & environment | Gazebo Harmonic | Pipe network, robot motion, collisions, obstacles, simulated sensors |
| Middleware | ROS 2 Jazzy | Sensors, controllers, localization, perception, navigation as nodes/topics |
| Visualization & navigation | RViz2 + Nav2 | Sensor data, pose, maps, planned routes; Nav2 as navigation infrastructure |
| Robot model | URDF/Xacro (+ Blender/CAD optional) | Chassis, wheels, joints, sensor mounts, frames |
| Algorithms & evaluation | Python, NumPy, pandas, Matplotlib | Filters, scoring, blockage detection, experiments |

Platform: Windows 11 → WSL2 → Ubuntu 24.04, VS Code via WSL extension. ROS installed inside Ubuntu.

## Component design

| Component | Implementation |
|---|---|
| Robot model | URDF/Xacro, differential-drive joints |
| Physics and sensors | Gazebo Harmonic |
| Wheel odometry and IMU | Simulated sensor plugins → ROS 2 topics |
| LiDAR | Simulated 2D range scanner |
| Baseline localization | Standard particle filter or AMCL |
| Proposed localization | Custom particle filter with reliability-aware range weighting |
| Obstacle detection | LiDAR-based geometric detection |
| Navigation | Nav2 + custom uncertainty-aware planner or cost layer |
| Evaluation | Python, NumPy, pandas, Matplotlib |

Key technical points:
- AMCL does **not** implement temporal reliability weighting. The proposed filter must explicitly calculate and use measurement reliability.
- IMU + wheel odometry alone drift. Range-based updates must correct the drift, and the demo must **show** the difference rather than assume localization is always accurate.

## Final demo: two views, recorded side by side

**View A — 3D sewer simulation (Gazebo)**
Robot chassis, wheels, LiDAR, IMU placement, tunnel geometry, water/debris obstacles, pipe junctions, blockages. Robot physically follows its route with collisions enabled.

**View B — localization and navigation (RViz2)**
True vs. estimated position, LiDAR rays, particle cloud, uncertainty, blocked edges, planned route. This is where the algorithm is shown doing something meaningful.

Later: a small dashboard with live numerical metrics.

## 14-day roadmap

| Days | Area | Milestone | Done when |
|---|---|---|---|
| 1 | Environment | Install stack, run the official Nav2 sample robot | Gazebo, RViz and robot movement all work; sensor topics verified |
| 2–3 | Robot | Differential-drive chassis in URDF/Xacro; wheel joints; IMU + 2D LiDAR | Robot drives in Gazebo; `/odom`, `/imu`, `/scan` publish; TF tree correct; collision geometry checked |
| 4–5 | Environment | Sewer network: connected pipes, bends, junctions, dead ends, blockages | Robot can navigate the world; geometry simple enough for reliable navigation |
| 6–8 | Algorithms | Baseline particle filter, then reliability-aware filter; simulated noisy/inconsistent range returns | Both filters run; reliability scores visibly drop under disturbances |
| 9–10 | Navigation | Nav2 basics; custom route cost accounting for localization uncertainty; blockage detection and replanning | Robot reroutes around a blockage; route cost reflects uncertainty |
| 11–12 | Evaluation | Compare both filters under identical conditions | Plots/tables for localization error, collisions, recovery time, route cost |
| 13–14 | Presentation | Materials, lighting, sensor & confidence overlays, repeatable demo scenarios | Final recorded run |

**Fallback:** if ROS 2/Gazebo setup drags, build and validate the localization algorithm in pure Python (`sandbox/`) while setup is resolved. Installation issues must not consume the deadline.

Don't spend the first two days custom-building the robot. Get the sample simulation working first.

## Setup sequence (Day 1)

1. Check hardware in Task Manager: RAM, GPU, storage. 16 GB RAM is a reasonable starting point; 8 GB may struggle with the full 3D stack.
2. Install WSL2 + Ubuntu 24.04 (Microsoft's official WSL guide).
3. Connect VS Code to Ubuntu (WSL extension); keep the project in the Linux filesystem.
4. Install ROS 2 Jazzy (official Jazzy Ubuntu install guide).
5. Install Gazebo Harmonic + Nav2 integration (Gazebo/Nav2 setup guide).
6. Run the official Nav2 simulation quickstart; confirm Gazebo, RViz and motion work.

Keep the official **Nav2 robot setup guide** open throughout (transforms, robot description, odometry, sensors, localization, navigation).

## Evaluation metrics

- Localization error over time (estimated vs. ground truth: position RMSE, max error, heading error)
- Number of collisions
- Recovery time after a disturbance (time for error to fall back below a threshold)
- Route cost / path length / mission success rate
- Same seeds, world and disturbance schedule for both filters

## Open decisions (to settle before or during the relevant milestone)

- [x] Robot dimensions, wheel radius and track width (decided; see Decisions made)
- [x] Pipe diameter(s), segment lengths, network layout (how many junctions, dead ends, blockages) (decided; see Decisions made)
- [x] LiDAR specs: range, field of view, angular resolution, rate (decided; see Decisions made)
- [x] Noise/disturbance models: water reflections, debris, dropouts, spurious returns, and when they occur (decided; see Decisions made)
- [x] Reliability score formula: how temporal consistency and geometric consistency are measured and combined (decided; see Decisions made)
- [x] Definition of a "blockage" and the detection threshold (decided; see Decisions made)
- [x] Baseline choice: Nav2 AMCL vs. our own standard particle filter (or both) (decided; see Decisions made)
- [x] How uncertainty enters the route cost (cost layer vs. custom planner) (decided; see Decisions made)
- [x] Thresholds for "recovered" and "failure" in evaluation (decided; see Decisions made)

## Decisions made

_(Record each decision here with the date and a one-line reason.)_

- 2026-10-27 — Day 1: used existing WSL2 distro `Ubuntu` (24.04.4); ignored `Ubuntu-22.04` (Humble) and `docker-desktop`.
- 2026-10-27 — Day 1: Stack installed via apt: ROS 2 Jazzy desktop, `ros-jazzy-ros-gz` (Gazebo Harmonic, gz sim 8.15), Nav2 + nav2_bringup, turtlebot3 packages, colcon, rosdep, numpy/pandas/matplotlib/scipy.
- 2026-10-27 — Day 1: Passwordless sudo enabled by user (`/etc/sudoers.d/90-claude-dev`) so installs can run unattended.
- 2026-10-27 — Day 1 finding: Jazzy Nav2 sample bridges `/cmd_vel` as `geometry_msgs/Twist` (not TwistStamped); odom/imu/scan/clock are present. Keep this in mind for our robot's bridge config.
- 2026-10-27 — Day 1 finding: WSL sees ~7 GB of the 15.7 GB host RAM (default 50%). Fine for headless runs; raise via `.wslconfig` if GUI Gazebo+RViz struggles.

## Day 1 checklist
- [x] Hardware check (15.7 GB RAM, RTX 4060 Laptop, 12 cores)
- [x] WSL2 Ubuntu 24.04, WSLg present
- [x] ROS 2 Jazzy + Gazebo Harmonic + Nav2 installed
- [x] Nav2 sample (TurtleBot3 waffle, headless) runs; `/clock /scan /odom /imu /cmd_vel /tf` present; `/scan` ~5 Hz; `/clock` advancing; driving 0.2 m/s for 8 s moved `/odom` x by 1.56 m
- [ ] Visual confirmation of Gazebo + RViz windows (needs user; not yet done)
- [x] `tf2_tools view_frames` check (done Day 2-3); `rosdep init/update` was not needed

- 2026-10-27 — Day 2-3 robot dimensions (open decision settled): chassis 0.30 x 0.18 x 0.08 m, 4 kg; wheel radius 0.06 m; track width 0.26 m; two frictionless ball casters (front/rear, r 0.03). Small enough for ~0.6-1.2 m pipes; revisit when the pipe diameter is fixed on Days 4-5.
- 2026-10-27 — Day 2-3 LiDAR spec (open decision settled): 2D gpu_lidar, 360 samples over 360 deg (1 deg), range 0.12-8.0 m, 10 Hz, Gaussian noise sigma 0.01 m. Mounted 0.05 m forward, 0.07 m above base_link. IMU: 100 Hz, gyro sigma 0.002 rad/s, accel sigma 0.017 m/s^2. Gazebo noise is only baseline sensor noise; the water/debris/dropout disturbances will be injected by our own seeded node so both filters see identical scenarios.
- 2026-10-27 — Day 2-3 TF/odom design: Gazebo DiffDrive publishes `odom -> base_footprint` (wheel odometry, 50 Hz) and `/odom`; robot_state_publisher publishes the rest from the URDF; `/joint_states` from Gazebo. `/ground_truth/odom` (OdometryPublisher, frame `world`) is evaluation-only. `map -> odom` is deliberately absent until a localizer is added.
- 2026-10-27 — Day 2-3 finding: flat-cylinder wheel collision scrubs at its inner edge when pivoting, so the real turning track was 0.219 m not 0.26 m (true yaw rate 0.591 vs 0.5 commanded, odom 16 deg short over a 90 deg turn). Fixed by using sphere collision for the wheels (visual stays a cylinder); odom yaw then matches ground truth to 0.1 deg. Sphere wheels should also suit round pipes.
- 2026-10-27 — Day 2-3 finding: wheel odometry has no slip model, so when the robot is pressed against a wall odom keeps integrating (measured +5.6 m error after 20 s). Good, real source of drift for the localization demo; no extra noise injected yet.
- 2026-10-27 — Packages created: `sewer_description` (Xacro), `sewer_gazebo` (test_arena.sdf world + bridge.yaml), `sewer_bringup` (robot_sim.launch.py, `headless:=true|false`, spawn x/y/yaw args). Verification scripts in `~/sewer_ws/tests/` (`day2_verify.sh`, `day2_robot_check.py`, `diag_rotate.py`, `run_diag.sh`).

## Day 2-3 checklist
- [x] Xacro model: chassis, 2 drive wheels, 2 casters, IMU, 2D LiDAR; `check_urdf` OK; converts to valid SDF
- [x] Differential drive in Gazebo; `/odom` 50 Hz, `/imu` 100 Hz, `/scan` 10 Hz, `/joint_states`, `/tf`, `/clock`, `/ground_truth/odom`
- [x] TF tree: odom -> base_footprint -> base_link -> {left/right wheel, front/rear caster, imu_link, lidar_link} (`view_frames`)
- [x] Sensor sanity: IMU az = 9.800 at rest; LiDAR ranges within 1 cm of known wall distances at 0/+-90/180 deg
- [x] Odometry vs ground truth: straight 0.6 m within 5 mm; 90 deg turn within 0.1 deg
- [x] Collision: stopped by wall at the expected face (x = 2.950), did not climb or tip
- [ ] Visual confirmation of robot appearance in Gazebo GUI (needs user; `ros2 launch sewer_bringup robot_sim.launch.py headless:=false`)
- [x] Collision *counting* for the evaluation metrics: still open, planned as M4 in the revised plan below (RViz config done)

- 2026-10-27 — Day 4-5 pipe geometry (open decision settled): box-section channels, 1.0 m wide, 1.0 m high walls (0.2 m thick), flat floor, instead of circular pipes. Reason: a wheeled robot needs a flat floor, and a 2D LiDAR plane sees the same walls as in a round pipe. No ceiling (the scan plane is horizontal). Water/visual polish deferred to Days 13-14.
- 2026-10-27 — Day 4-5 network layout (open decision settled): 8 nodes, 8 edges, ~30 m of corridor in a 20 x 17 m area. Entry A(0,0) is a dead end; J1(8,0) 4-way junction; D(8,-5) and E(12,10) dead ends; J2(8,6), J3(16,6) and J4(16,0) bends; M(12,6) T-junction. Start = A, goal = J4. Short direct route A-J1-J4 = 16 m; alternative loop A-J1-J2-M-J3-J4 = 28 m (8 m to J1, then 6+4+4+6 = 20 m).
- 2026-10-27 — Day 4-5 blockage definition (open decision settled): a blockage is debris leaving a free gap narrower than 0.50 m (robot width 0.30 m incl. wheels + 2 x 0.10 m clearance). B1 = full blockage (gap 0) at x ~ 12 on J1-J4. B2 = partial debris (0.60 m gap, passable but tight) at y ~ 3 on J1-J2. Debris is NOT in the a-priori map, so perception must detect it. Detection threshold for the perception module is still to be set on Days 9-10.
- 2026-10-27 — Day 4-5 single source of truth: `sewer_gazebo/scripts/gen_network.py` generates worlds (`sewer_network.sdf` with blockages, `sewer_network_clear.sdf`), the a-priori map (`maps/sewer_map.{pgm,yaml}`, 0.05 m, walls only), `config/network_graph.yaml` (topology the planner may use) and `config/blockages_truth.yaml` (EVALUATION ONLY). Regenerate with `python3 src/sewer_gazebo/scripts/gen_network.py --out src/sewer_gazebo --preview logs/network_preview.png`, then `colcon build`. World walls and map use the same 5 cm grid.
- 2026-10-27 — Day 4-5 finding: the corridors are featureless straight runs between junctions, so a range-based filter will have along-corridor ambiguity there; junctions/bends are the informative places. Good stress case for the reliability weighting.
- 2026-10-27 — Day 4-5 finding: wheel-odometry-only driving through the loop (27.5 m, 5 turns) drifted 0.156 m from ground truth and growing (0.015 m on the 16 m straight route). This is the baseline drift the filters must correct.
- 2026-10-27 — Spawn the robot in the network with: `ros2 launch sewer_bringup robot_sim.launch.py world:=sewer_network.sdf x:=0.0 y:=0.0 yaw:=0.0`.

## Day 4-5 checklist
- [x] Connected sewer network: junction, bends, T-junction, dead ends, loop
- [x] Blockages: B1 full, B2 partial (unmapped debris), clear and blocked world variants
- [x] A-priori occupancy map generated from the same description as the world
- [x] World == map: clear world, ray-cast of the map from the true pose vs live LiDAR, mean 97.4% of rays within 0.15 m (min 95.3%)
- [x] Navigable: robot drove the direct route (46 s) and the long loop incl. the 0.6 m gap beside B2 (86 s) with no stuck/timeouts; closest LiDAR return 0.26 m
- [x] Blockage B1 visible: from 7.6 m away; at (10.5, 0) the LiDAR sees it at 1.13 m where the map predicts 6.10 m (forward-sector agreement 8%)
- [x] Top-down render from the world's own camera inspected (`logs/topdown_blocked.png`): layout, robot and debris as designed
- [ ] Visual check of the 3D look in the Gazebo GUI (needs user)
- [ ] Collision counting (contact sensor) for the evaluation metrics, still open from Day 2-3

- 2026-10-27 — Day 6-8 architecture (deviation from CLAUDE.md layout, with reason): the filter code lives ONCE in `src/sewer_localization/sewer_localization/` as pure NumPy/SciPy (`map_utils.py`, `reliability.py`, `particle_filter.py`, no ROS imports). `sandbox/` (2D simulator `sim2d.py`, `run_experiment.py`, `tests/`) imports it by path, so the sandbox runs without ROS and the "sandbox in sync with ROS" rule holds by construction (one implementation). ROS node wrappers are NOT written yet.
- 2026-10-27 — Day 6-8 baseline choice (open decision settled): our own standard PF, not AMCL. It shares everything with the proposed filter (motion model, resampler, RNG stream, likelihood-field range model in AMCL's functional form `z_hit*exp(-d^2/2s^2) + z_rand/zmax`); the ONLY difference is the per-beam reliability weight. Invariant tested: with every r_i = 1 the proposed filter reproduces the baseline exactly. Nav2 AMCL stays an optional extra reference for Days 11-12.
- 2026-10-27 — Day 6-8 filter parameters (defaults, not tuned): 500 particles; every 6th LiDAR beam; sigma_hit 0.15 m; z_hit 0.8, z_rand 0.2; log-likelihood tempering kappa 0.25; systematic resampling at N_eff < 0.5 N; tracking mode from the known entry pose (init std 0.1 m, 0.05 rad). Motion: wheel-odometry distance + heading fused 0.8 IMU gyro / 0.2 wheel yaw; noise 3% of distance + 3 mm, 5% of rotation + 0.03 rad/m + 2 mrad per step. Max-range / inf beams are skipped (as AMCL does).
- 2026-10-27 — Day 6-8 reliability score (open decision settled): per beam r_i = clip(g_i * t_i, 0.02, 1) for valid returns. Uses only the scan and odometry, never the map, particles or ground truth. g_i (geometric): distance of the beam endpoint to the best of three lines through its neighbours (i-2,i-1), (i+1,i+2), (i-1,i+1); g = exp(-0.5 (res/sigma_g)^2), sigma_g = 0.03 + 0.01 z; no usable neighbours -> 0.5. t_i (temporal): the previous scan is motion-compensated with the fused odometry/IMU increment and interpolated at beam i's bearing (gaps > 2.5 deg or range jumps > 0.4 m are not interpolated); t = exp(-0.5 (|z - zhat|/sigma_t)^2), sigma_t = 0.05 + 0.01 z + 0.1 |dtheta| z; no prediction available -> 0.8. Scan-level reliability R = mean r_i, logged for monitoring and the uncertainty estimate. The two terms are complementary: a smooth water reflection passes g but fails t (tested).
- 2026-10-27 — Day 6-8 disturbance models (open decision settled), injected by our own seeded model in `sim2d.py` (same data for every estimator): `spurious` (each beam replaced w.p. p by a short random return), `water` (a 70 deg sector sees a reflection: ranges scaled by a factor random-walking 1.15-2.2 from scan to scan, +4 cm noise: smooth within a scan, inconsistent across scans), `dropout` (beam loses its return w.p. p), `burst` (extra 10 cm range noise). Base schedule (sim s): spurious p=.35 [8,16] entry corridor; water left [28,38] leg past B2; dropout p=.5 + spurious p=.2 [46,56] J2-M corridor; water right + burst [62,72] last leg. Severity multiplier s scales p, sector width (cap 330 deg) and burst sigma; sweep 1, 2, 3, 4 defined before looking at results. Sandbox sensors: LiDAR as the Gazebo model (360 beams, 8 m, 10 Hz, sigma 1 cm); wheel odometry distance scale error N(0,1%), wheel-yaw scale error N(0,3%), gyro bias N(0, 1 mrad/s) + random walk, drawn per run.
- 2026-10-27 — Day 6-8 evaluation thresholds (open decision settled, tentative until Days 11-12): "recovered" = position error < 0.15 m for 1 s; "failure" = position error > 0.50 m. Metrics: position RMSE / max / final, heading RMSE, fraction of time above failure, recovery time per disturbance window that pushed the error above 0.15 m (censored at end of run).
- 2026-10-27 — Day 6-8 RESULT (sandbox, loop route, 5 seeds, identical recorded data for all estimators; preliminary, NOT the Days 11-12 study): dead reckoning (fused odometry+IMU) RMSE 0.376 m, max 0.716 m, heading RMSE 3.1 deg, 22% of the time above 0.5 m: range updates are needed. Clean run: baseline 0.035 m, proposed 0.034 m (no difference, as expected). Position RMSE baseline vs proposed by severity: x1 0.037 vs 0.036 (proposed better in 3/5 seeds); x2 0.048 vs 0.041 (5/5); x3 0.071 vs 0.062 (5/5); x4 0.170 vs 0.050 (5/5). The x4 mean is outlier-driven: baseline per-seed RMSE 0.043, 0.058, 0.139, 0.556 (lost track, max 1.13 m), 0.052 vs proposed 0.034, 0.052, 0.060, 0.060, 0.047. Typical gain 5-20%; large gain only where the baseline fails. At the base schedule the baseline is NOT hurt, so the benefit depends on disturbance severity. 5 seeds is too few to call it statistically.
- 2026-10-27 — Day 6-8 RESULT, reliability scores: scan reliability R is ~0.95 on clean scans and drops to 0.4-0.87 inside every disturbance window (`logs/day6/reliability.png`). Mean per-beam r by injected class at x1: clean 0.90, burst 0.42, spurious 0.26, water 0.43. Clean beams are still not exactly 1 (0.90): corners and range discontinuities cost some weight.

## Day 6-8 checklist
- [x] Baseline PF and reliability-aware PF implemented in a shared pure-NumPy core
- [x] 2D sandbox simulator with seeded, identical-for-all disturbances (spurious, water, dropout, burst) and unmapped debris
- [x] 11 unit tests pass (`cd ~/sewer_ws/sandbox && python3 -m pytest -q tests`): raycast/map, clean scans reliable, spike flagged geometrically, scan-to-scan shift flagged temporally, both filters track a clean run, deterministic, proposed == baseline when r_i = 1
- [x] Experiment runner: `python3 run_experiment.py --seeds 5 --severities 1,2,3,4 --out ../logs/day6` writes `results.csv`, `error_vs_time.png`, `reliability.png`, `trajectories.png` (all inspected)
- [x] Reliability scores visibly drop under disturbances
- [x] `sewer_localization` builds with colcon and imports through the ROS install
- [x] ROS 2 node wrappers for both filters, run in Gazebo (done: see ROS wrapper checklist)
- [ ] Larger seed count (>= 20) and a second route / world for Days 11-12; AMCL as extra reference; collision metric needs the ROS side
- [x] Uncertainty in the route cost and the blockage detection threshold (done Days 9-10)

- 2026-10-28 — ROS wrappers (Day 6-8, ROS side): `sewer_localization` now has two nodes, `localizer_node` (parameter `filter:=baseline|reliability`; the same pure-NumPy filters as the sandbox) and `scan_disturbance_node` (republishes `/scan` as `/scan_disturbed` with the shared seeded disturbance models; the schedule clock starts when wheel odometry first shows motion; `/scan_disturbance_kind` carries the per-beam labels for EVALUATION ONLY). The disturbance models moved into the shared core (`sewer_localization/disturbances.py`) so sandbox and ROS inject identical corruption; refactor verified: the sandbox reproduces the earlier numbers exactly.
- 2026-10-28 — Localizer node design: inputs `/scan`, `/odom`, `/imu` (sensor QoS); ds and wheel heading from the odometry pose nearest the scan stamp, gyro heading integrated from `/imu` angular_velocity.z and fused 0.8/0.2 with wheel yaw (same as the sandbox); tracking mode from the known entry pose (init_x/y/yaw params); `map -> odom` computed from the filter estimate and published at 20 Hz only if `publish_tf` is true. Ground truth is never read. Outputs (relative names, so a namespace gives `/baseline/...`): `estimated_pose` (PoseWithCovarianceStamped), `particle_cloud` (PoseArray), `scan_reliability` (Float32MultiArray, per-beam r_i), `scan_reliability_R` (Float32, scan-level R). The last two exist for the reliability filter only.
- 2026-10-28 — Launch: `ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=sewer_network.sdf x:=0.0 y:=0.0 yaw:=0.0` then `ros2 launch sewer_bringup localization.launch.py filter:=both disturbed:=true severity:=3.0 seed:=0`. With `filter:=both` the baseline runs in `/baseline` (no TF) and the reliability-aware filter in `/reliability` (publishes `map->odom`): same live data, never two TF publishers. `filter:=baseline|reliability` runs one localizer with plain topic names.
- 2026-10-28 — LIVE RESULT (Gazebo sewer network, loop route driven on wheel odometry, 80 sim s, one run per row, both filters on the same live data; NOT statistically meaningful): clean: wheel odom RMSE 0.188 m (max 0.43, final 0.42), baseline 0.069, proposed 0.068, R = 0.95 everywhere. Severity x3: odom 0.184, baseline 0.077, proposed 0.074, R = 0.94 outside / 0.47 inside the windows (min 0.10). Severity x4: odom 0.195, baseline 0.083, proposed 0.085, R = 0.94 / 0.46 (min 0.08). Both filters publish at 10.0 Hz and the TF `map->base_footprint` matches the estimate to < 1 mm.
- 2026-10-28 — LIVE finding 1: in Gazebo the wheel odometry has no slip model and drifts only ~0.19 m RMSE (the sandbox odometry error model gives ~0.38 m), and a single run shows NO baseline failure even at x4: it matches the sandbox seeds where the baseline also coped. So the live runs confirm the pipeline and the reliability scoring, but do NOT yet reproduce the sandbox gap between the filters. The real comparison needs many seeds and a harsher, repeatable odometry error model (to do for Days 11-12; consider an odometry-noise node).
- 2026-10-28 — LIVE finding 2: live filter RMSE (~0.07 m) is about twice the sandbox's (~0.035 m). Likely causes, not yet investigated: nearest-sample timestamp sync of odometry/IMU to the scan, the scan arriving while the robot moves, and the Gazebo world vs 5 cm map discretisation. Not a blocker.
- 2026-10-28 — Environment note: DDS/`ros2 topic list` discovery under WSL is flaky (stale daemon showed missing topics once, which made the first live check fail). The verification scripts now run `ros2 daemon stop` between runs and wait for data on wall-clock time (90 s) rather than sim time.

## ROS wrapper checklist
- [x] `localizer_node` and `scan_disturbance_node` build with colcon (`ros2 pkg executables sewer_localization`)
- [x] Both filters run live against Gazebo `/scan`, `/odom`, `/imu` and publish `estimated_pose`, `particle_cloud`, `scan_reliability`, `scan_reliability_R` at 10 Hz
- [x] `map -> odom` from one localizer; TF consistent with the estimate (< 1 mm)
- [x] Reliability score drops live under injected disturbances (0.94 -> ~0.47)
- [x] Disturbance models shared between sandbox and ROS; sandbox results unchanged (11 tests pass)
- [x] RViz config for View B (done)
- [ ] Odometry error model in ROS and a many-seed live comparison (Days 11-12); AMCL as extra reference
- [ ] Investigate live vs sandbox RMSE gap (timestamp sync)

- 2026-10-28 — View B (RViz) built: `sewer_bringup/launch/view_b.launch.py` (+ `config/view_b.rviz`) starts Nav2's `map_server` (a-priori map on `/map`), a static identity `map -> world` transform (only so the TRUE pose from `/ground_truth/odom` can be drawn; the localizers never read it) and `rviz2` (`rviz:=false` for headless). Displays: prior map (walls only, debris is NOT in it), robot model, raw `/scan` (white), the scan the reliability filter used coloured by per-beam reliability (red 0 -> green 1), both particle clouds (blue = baseline, red = reliability-aware), both estimates with 1-sigma ellipses, TRUE pose (green arrow), and status text next to each estimate (`sigma`, and `R` for the reliability-aware filter, text turns yellow below R 0.8 and red below 0.5). Default camera follows the robot (top-down, 110 px/m); a saved "Overview (whole network)" view is in the Views panel.
- 2026-10-28 — New localizer outputs for the view: `/<ns>/reliability_scan` (LaserScan, intensities = r_i, stamp-aligned with the scan it scored) and `/<ns>/status_marker` (Marker text).
- 2026-10-28 — Run it: terminal 1 `ros2 launch sewer_bringup robot_sim.launch.py headless:=true world:=sewer_network.sdf x:=0.0 y:=0.0 yaw:=0.0` (or `headless:=false` for the Gazebo window); terminal 2 `ros2 launch sewer_bringup localization.launch.py filter:=both disturbed:=true severity:=3.0 seed:=0`; terminal 3 `ros2 launch sewer_bringup view_b.launch.py`; then drive (e.g. `python3 ~/sewer_ws/tests/ros_localizer_check.py 0.5`, which drives the loop route on wheel odometry).
- 2026-10-28 — How it was verified (no screen needed): RViz ran in a virtual X display (Xvfb + software GL, `~/sewer_ws/tests/view_verify.sh`) and screenshots were captured and inspected (`logs/day6/rviz/f*.png`). Command checks: `/map` 388x368 at 0.05 m; TF `map->world` and `map->base_footprint` resolve; both status markers publish; `reliability_scan` carries frame `lidar_link` and per-beam intensities. Clean beams are all green (R 0.95); in a disturbance window the corrupted beams turn red/yellow, R drops (0.36-0.40 seen) and the status text turns red; reflected water returns appear outside the walls. A real-window check by the user is still open (see checklist). RViz logs one harmless shader error under software GL (`indexed_8bit_image.vert`); the map still renders.
- 2026-10-28 — BUG FOUND AND FIXED while verifying the view: with the localizers idle for ~1 min, the estimate drifted 0.28 m off along the corridor. Cause: the motion model added constant noise floors even when the robot reported no motion, so a parked robot random-walked along the weakly observable corridor axis (sandbox, 90 s parked: baseline -0.21 m, proposed -0.30 m). Fix: the floors now apply only while moving (|ds| > 1 mm or |dtheta| > 1 mrad per step). Regression test added (`test_parked_robot_does_not_drift`, both filters); 13 tests pass. Sandbox experiment re-run: every number identical to the earlier table (clean 0.035/0.034; x3 0.071/0.062; x4 0.170/0.050), so no moving-robot result changed. Live idle offset after the fix: 0.118 m (was 0.276 m).
- 2026-10-28 — KNOWN ISSUES (not fixed, matter for the uncertainty-aware planner): (1) the particle cloud is overconfident: status text shows sigma 0.02-0.05 m while a parked robot in the entry corridor was 0.12 m off; resampling with no process noise collapses the cloud, so the reported sigma understates the true error (needs roughening or adaptive resampling before sigma is used in a route cost). (2) Along-axis position in a featureless corridor is weakly observable, as expected. (3) Running RViz with software rendering loads the CPU and the live filter RMSE rose to ~0.09 m (was ~0.07 m without it; odom 0.19 m in both); on a real GPU this should be less.

## View B checklist
- [x] `view_b.launch.py` + `view_b.rviz`; map server, TF, robot model, scans, particles, estimates, uncertainty, status text, true pose
- [x] Reliability visible per beam (red -> green) and per scan (R in the status text), verified in screenshots
- [x] Verified headless in a virtual display with screenshots inspected; command checks pass
- [x] Idle-drift bug found, fixed, regression-tested; sandbox results unchanged
- [ ] User check in the real RViz window under WSLg (launch the three commands above; see what to look for in the summary)
- [ ] Overconfident sigma: roughening / adaptive resampling before uncertainty enters the route cost
- [x] `/blockages` markers and planned route display in RViz (done Days 9-10)

- 2026-10-29 — Day 9-10 architecture: three new packages. `sewer_perception` (blockage detection: pure-NumPy core `blockage.py` + `blockage_detector_node`), `sewer_planning` (`graph.py`, `observability.py`, `route_cost.py`, `pursuit.py` + `route_planner_node`, `route_follower_node`), and launch files `mission.launch.py` (detector + planner + follower) in `sewer_bringup`. Sandbox tests import the cores by path (27 tests pass: `cd ~/sewer_ws/sandbox && python3 -m pytest -q tests`).
- 2026-10-29 — DEVIATION FROM PLAN.md tech stack (flagged for the user): Nav2's grid planners, costmaps and controllers are NOT used. Navigation is a custom graph-level planner plus a small pure-pursuit follower that drives on the ESTIMATED pose. Reason: in a 1 m corridor network with a known topology the research contribution (uncertainty-aware route cost, reroute around blockages) lives at route level; Nav2's local planners add integration risk and little else. Nav2 is still used for `map_server`/`lifecycle_manager` (the a-priori map for RViz). Wrapping the route as Nav2 goals is possible later if wanted.
- 2026-10-29 — Blockage detection (open decision settled): a trusted beam (per-beam reliability r_i >= 0.5) whose return is >= 0.30 m SHORTER than the prior map predicts from the estimated pose, and whose endpoint is > 0.15 m from any mapped wall (pose error / grazing beams), adds +1.0 log-odds to its endpoint cell; trusted rays subtract 0.2 along the free space before the hit; log-odds clamped to [-2, 6]; a cell is obstacle at >= 2.0; evidence clusters smaller than 6 cells (~0.3 m of surface) are ignored (found necessary by the ablation below: a single stray cell near the centre line tripped the gap rule). Per edge, the corridor is cut into 5 cm cross-sections; gap = widest free lateral interval (cells dilated 7.5 cm); BLOCKED if the smallest gap < 0.50 m (the Day 4-5 definition: robot 0.30 m + 2 x 0.10 m); the free gap's lateral centre is reported as the offset to drive at. Unobserved edges are assumed free (optimistic: blockages are discovered by approaching). Published: `/edge_gaps` (JSON, `A-B` keys) and `/blockages` (MarkerArray).
- 2026-10-29 — Uncertainty in the route cost (open decision settled; custom planner, not a Nav2 cost layer): cost(route) = length + w_unc * integral(sigma ds) + narrow-gap penalty. sigma(s) is the largest std. dev. of a 2x2 position covariance propagated along the route like a Kalman filter: drift q_along 0.005 m^2/m (about 1.5% of distance, from the Gazebo wheel-odometry drift 0.42 m over 27.5 m) along the travel direction and q_lat 0.0005 laterally; every 0.5 m an update P <- (P^-1 + R * I(s))^-1 where I(s) is the information the PRIOR MAP gives at that spot (ray-cast 360 beams, surface normals from the smoothed map gradient, weight kappa/sigma_hit^2/stride = 0.25/0.15^2/6, the same likelihood field the localizer uses) and R is the live scan reliability (smoothed ~1 s, floored at 0.1). So featureless stretches let uncertainty grow, junctions/bends pull it back, and falling reliability raises every stretch's cost. Start covariance = localizer covariance + (0.05 m)^2 floor because particle clouds are overconfident. Defaults: w_unc 2.0 m per m^2, narrow penalty 5 m per metre of gap below 0.80 m, off-centre driving when an obstruction leaves < 0.90 m, hysteresis 1.0 m, replanning 2 Hz. Routes are enumerated exhaustively (tiny graph; cost depends on the whole route).
- 2026-10-29 — Follower defaults: vmax 0.35 m/s, lookahead 0.5 m, goal tolerance 0.2 m; rotates in place when the heading error exceeds 0.6 rad; aims at a sharp corner point until within 0.2 m of it instead of cutting the corner.
- 2026-10-29 — RESULT, route cost (sandbox/planner, nothing blocked, A -> J4): direct route length 16.0 m, uncertainty integral 1.28 m^2, total 18.6; loop length 28.0 m, integral 1.99, total 32.0. Lowering R from 0.95 to 0.20 raises the direct route's cost 18.57 -> 19.58 (sigma at the goal 0.045 -> 0.081 m). Along J1 -> J4 sigma grows 0.055 -> 0.09 m in the straight corridor and falls to ~0.05 at the J4 bend. FINDING: in THIS network the uncertainty term never changes which route wins (the loop is both longer and more uncertain, so no weight flips it); it changes the cost, and it would matter in a network with alternatives of comparable length. A second parallel route would be needed to demonstrate a decision change.
- 2026-10-29 — RESULT, blockage detection ablation (sandbox, empty network, loop route): BEFORE the cluster filter, at 4x severity a phantom BLOCKED edge appeared in 2/5 seeds with reliability weighting and 2/5 without (stray obstacle cells 7 vs 19). AFTER the filter, on fresh seeds: 0/10 phantom blocked edges at both 3x and 4x for both variants, and 0/5 at 6x (stray cells 68 with reliability vs 48 without). Phantom partial clusters: 0/6 seeds at 1x, 3x, 4x for both. So the cluster filter, not the reliability weighting, is what protects against phantoms; I could NOT show a benefit of reliability weighting for detection in this setup. Real debris is found: B1 flagged blocked (gap ~0.2 m, truth 0), B2 passable with gap 0.55-0.60 m (truth 0.60) and offset -0.2.
- 2026-10-29 — RESULT, live Gazebo missions (robot starts at A, blockage unknown, drives on the ESTIMATED pose; localizers + detector + planner + follower; disturbance node feeding corrupted scans in the disturbed runs): the planner starts on the direct route, B1 is flagged when the robot is at x = 5.2-5.8 (about 15-17 s after it starts moving), the route switches to the loop 0.5 s later, the robot threads the B2 gap and reaches J4 (final true position within 0.2 m of (16, 0)). 10 missions with the first version (clean x4, 3x with seeds 0,1,2 incl. repeats, 4x seeds 0,1): all reached the goal, but took 93-131 s with 5-15 s idle spells at J1 and M, and one 4x run came within 0.00 m of the J2 inner corner (clearance computed with a conservative 0.20 m robot radius). 9 missions with the FINAL code (clean x2, 3x x5 over disturbance seeds 0,1,2, 4x x2 over seeds 0,1; one of the 3x runs was in RViz): all reached the goal in 83-84 s, no phantom blockage, minimum clearance 0.05-0.10 m to walls/debris, replanned within 0.5 s of detection; in the 5 runs analysed in detail (clean, 3x seed 0 and 2, 4x seeds 0 and 1) there were no idle spells and the position error of the estimate stayed below 0.19 m.
- 2026-10-29 — BUGS FOUND AND FIXED during the live missions: (1) `Graph.locate` picked the corridor by lateral offset only and allowed corridors to extend 0.5 m past their ends, so beside a junction the planner thought the robot was on the dead-end spur (at J1) or behind a node (at M) and sent it back to the node it had just left; the robot hunted for 5-15 s. Now it picks the nearest corridor by true distance to the centre-line segment; regression tests added (`test_locate_...`, `test_route_never_doubles_back_...`), verified that the old rule reproduces the wrong edge. (2) Pure pursuit cut the J2 corner to within the robot radius of the inner wall corner; `pursuit.py` now aims at the corner point first; unit-tested. (3) The mission check itself had two bugs (it read the last route event instead of the replan event; it required the very first route to start at A); fixed.
- 2026-10-29 — KNOWN ISSUES / CAVEATS: (1) The B2 gap is 0.60 m for a robot 0.30 m wide incl. wheels: the minimum clearance was only 0.05-0.10 m with a 0.20 m conservative radius, so passing B2 is tight and depends on the estimate staying within ~0.1 m. (2) One run (3x, with RViz software rendering loading the CPU) showed a phantom partial obstruction on A-J1 (gap 0.65 m, never blocked); it did not appear in 3 later live runs nor in the sandbox; cause not established (CPU load / estimate lag suspected). It only adds off-centre waypoints and a narrow-gap penalty. (3) Steering is jerkier under disturbances: angular command sign flips 69-83 per run vs 32 clean. (4) Everything was verified on ONE network, one start and one goal, ~9 missions, so these are working-demo checks, not a statistical study. (5) The uncertainty weights (q_along, w_unc, narrow penalty) are reasoned defaults, not calibrated by experiment. (6) The overconfident particle-cloud sigma (View B known issue) is only handled by the 0.05 m floor.

## Day 9-10 checklist
- [x] Blockage detection from the reliability-weighted LiDAR with a log-odds evidence grid; per-edge gap analysis; `/edge_gaps`, `/blockages`
- [x] Uncertainty-aware route cost: covariance propagated along routes with map observability and live reliability; route cost reflects uncertainty (R 0.95 -> 0.20 raises cost); blocked edges removed; narrow-gap penalty
- [x] Replanning around a blockage found on the way: the robot reroutes via the loop and reaches J4 (9 live missions with the final code, clean and disturbed)
- [x] RViz: planned route (yellow), detected obstacle cells and edge labels in View B; whole-network screenshots inspected (`logs/day9/rviz/`)
- [x] 27 unit tests (detection, false positives, route cost, locate regressions, pursuit)
- [ ] Show the uncertainty term changing a route DECISION (needs an alternative route of similar length)
- [ ] Larger live study (many seeds, other goals/starts) and calibration of the cost weights for Days 11-12
- [ ] Optional: wrap the route as Nav2 goals (not done: see deviation)
- [ ] Collision counting (contact sensor) for the evaluation metrics; so far only clearance computed from ground truth

- 2026-10-29 — Environment note: a `gz sim` server can hang during shutdown after SIGTERM and linger (one lived 52 minutes, idle at 0% CPU, orphaned). The test scripts' `cleanup()` now follows `pkill -f "gz sim"` with `pkill -9 -f "gz sim"`. A fresh 4x seed 0 mission on a clean machine state afterwards passed again (goal reached in 84 s, minimum clearance 0.05 m, obstruction clusters only on J1-J4 and J1-J2).

- 2026-10-30 — 3D renders (View A): the Gazebo world and robot now carry cameras, rendered headless. Robot: `robot_cam` (front, 640x360, fov 1.5 rad) and `chase_cam` (0.75 m behind, 0.55 m above, pitched 0.5 rad down, 640x360, fov 1.2); world (generated by `gen_network.py`): `oblique_cam` (960x540 at (-6,-10,12), looks at the middle of the network) next to the existing `topdown_cam`. Topics `robot_cam/image`, `chase_cam/image`, `oblique_cam/image` (gz); bridge them on demand with `parameter_bridge '/<name>/image@sensor_msgs/msg/Image[gz.msgs.Image'`. Cameras render only while something subscribes. Look: sun with shadows and ambient light, robot yellow, debris red-brown, plus a beacon (1.6 m white mast with a green ball, visual only) so the robot can be found in the overview above the 1 m walls. The mast starts 0.2 m above base_link, above the LiDAR plane, because the GPU LiDAR ray-casts against VISUAL geometry; verified by `tests/scan_probe.py` (robot parked at A: range straight behind 0.54 m = the end wall, nothing closer than 0.49 m).
- 2026-10-30 — Render pipeline: `bash ~/sewer_ws/tests/render_mission.sh [disturbed true|false] [severity]` runs the verified mission (sim + localizers + detector + planner + follower, `mission_check` observing), records every camera with `record_frames.py` (JPEG per frame plus its sim time), resamples to a constant 10 fps by sim time with `build_sequence.py` (holds the latest frame; software rendering delivers only 4.6-6.9 Hz of sim time per camera, so a naive 10 fps encode would run the clips too fast and out of sync), and encodes H.264 with ffmpeg (first 2.5 s skipped: the chase camera starts behind the entry end wall). Outputs in `~/sewer_ws/logs/day10/clips/` and copied to the project folder `renders/`: `mission_3d.mp4` (oblique + chase + robot side by side, labelled), `oblique_overview.mp4`, `chase_camera.mp4`, `robot_camera.mp4` and five stills. The rendered mission itself passed all checks (reached J4 in 83 s, rerouted around B1, min clearance 0.05 m).
- 2026-10-30 — 3D render limits: the scene is plain boxes with flat colours (square channels, no ceiling, water, textures or pipe curvature); the oblique camera is static and the robot is small in it (hence the beacon); frame rate is uneven (max gap 1.0-1.2 s in the overview) so motion is a little steppy; the Gazebo GUI window itself has not been looked at by the user. A Blender version (round pipes, water, lighting) is not built; the Blender tools exist in this session but a connection was never checked.


## Revised Days 11-14 (approved 2026-11-01; replaces the paper-driven plan)

The project goal is unchanged: a credible 3D digital twin whose demo shows (1) the robot's parts, (2) it entering the sewer, (3) sensors seeing walls and debris, (4) the estimated position changing when readings become unreliable, (5) a safer route around a blockage; recorded as View A (Gazebo 3D) + View B (RViz) side by side, backed by an evaluation of both filters under identical conditions. An earlier, paper-driven plan (novelty claims, strong-baseline suite, real data) was set aside; only items that serve the demo were kept.

Why: the Day 9-10 live missions pass, but demo goal 4 is weak live (Gazebo wheel odometry drifts only ~0.19 m RMSE, so both filters look alike), Days 11-12 evaluation and Days 13-14 presentation are not done, and two defects remain (see M1).

Decisions confirmed by the user: keep a LEAN localization-health state (reliable / degraded / critical, robot slows or stops); View B for the final video = a recorded bag replayed into RViz; collisions = Gazebo contact sensors; no Nav2 AMCL reference for now.

Out of scope (deferred, paper-only): learned reliability combiner, extra reliability cues, persistence filter and silt, random multi-network generation, global localization, real-data validation, novelty write-up, strong-baseline suite.

- [x] **M1 Fixes and housekeeping** (done 2026-11-01). Roughening after resampling (small floors, only while moving); PLAN.md housekeeping. The planned change "detector no longer gates evidence by per-beam reliability" was TRIED AND REVERSED: it failed in live 3x missions (see the M1 findings below).
- [x] **M2 Odometry/IMU error model in ROS** (done 2026-11-01). Shared `OdometryErrorModel` + slip schedule in `disturbances.py` (sandbox bit-exact vs the old numbers), `odom_disturbance_node`, `localizer_node` `tf_odom_topic`, launch `odom_disturbed:=true`. Dead reckoning drifts 0.7-0.9 m RMSE (1.2-1.8 m at the end) and both filters correct it in the clean-scan conditions; the compound 3x scan + 3x slip case still defeats both filters (see M2 findings below).
- [x] **M3 Lean health state** (done 2026-11-01). `health.py` (signals R, scan-to-map fit, wheel-vs-gyro disagreement; smoothed, worst-stress state machine with hysteresis) -> RELIABLE/DEGRADED/CRITICAL; published as `<ns>/localization_health`, shown in the RViz status text colour; the follower (`health_topic`, launch `health_response:=true|false`) drives at 0.5x when DEGRADED, waits when CRITICAL and creeps at 0.3x after 8 s. It is a hostile-conditions detector, NOT a divergence predictor (see M3 findings below).
- [x] **M4 Collision counting** (done 2026-11-02). Gazebo contact sensors on the chassis and both wheels, `gz-sim-contact-system` in all worlds and in `gen_network.py`, the three sensor topics bridged to `/contacts/{chassis,left_wheel,right_wheel}` by `robot_sim.launch.py`, and the new `sewer_eval` package (`collisions.py` + `collision_counter_node` -> `/collisions` JSON). `mission_check.py` now judges collisions by the contact count; ground-truth clearance stays as a secondary figure.
- [ ] **M5 Evaluation (Days 11-12)**: `sewer_eval` package (shared metrics: error, collisions, recovery time, route cost, success, mission time, time per health state); Gazebo batches {clean, disturbed x2, x3} x 10 seeds with both filters driving; sandbox 30 seeds per severity on tuning seeds disjoint from reported seeds; tables with mean +- 95% CI; negative results reported as they are.
- [ ] **M6 Presentation (Days 13-14)**: named demo presets; record a bag during the render mission; View A = Gazebo camera frames, View B = bag replayed into RViz and captured under Xvfb; captions from mission/health events and a small metrics strip; optional visual-only water plane below the LiDAR plane.
- [ ] **M7 Close-out**: PLAN.md decisions and results, run instructions, final video and tables copied to the Windows `renders/` folder.

Estimate: about 8 work-days (+-40%); evaluation and recording compute runs in the background.


### M1 findings (2026-11-01)

- Roughening (sigma credibility). Measured on tuning seeds 100-104 (disjoint from the reported seeds 0-4) as the fraction of steps with the true error within 1 / 2 / 3 reported sigma (`sandbox/check_sigma.py`; an honest 2D estimate covers about 40-60% / 85-95% / >97%). Before: clean 0.54-0.57 / 0.91-0.92 / 1.00 with median sigma 0.023 m, i.e. the clean sandbox cloud was ALREADY calibrated; disturbed x3: baseline 0.35 / 0.72 / 0.90, reliability-aware 0.41 / 0.85 / 0.96. The overconfidence seen live was mostly the parked robot (weak corridor axis, repeated identical scans) and the disturbed runs. Final setting: spread-based jitter k = 0.2 with floors of 5 mm and 0.1 deg, applied only while moving: clean 0.56 / 0.94 / 1.00; x3 baseline 0.36 / 0.79 / 0.92, reliability-aware 0.41 / 0.87 / 0.98; RMSE unchanged or slightly better (x3: baseline 0.061 -> 0.058, reliability-aware 0.054 -> 0.053). The residual under-coverage of the baseline under disturbance is inherent: it cannot know its beams are bad. Lesson: my first choice of floors (2 cm / 0.5 deg) made the baseline lose track under disturbance (x3 RMSE 0.057 -> 0.291 m on the tuning seeds): at nearly every resample the jitter random-walks the estimate along the weak corridor axis.
- CORRECTION to the Day 6-8 result. With roughening the baseline no longer loses track in the severity-4 seeds (per-seed RMSE, seeds 2 and 3: 0.139 and 0.556 m before, 0.067 and 0.079 m after). Mean RMSE on the reported seeds 0-4, baseline / reliability-aware, before -> after roughening: clean 0.035 / 0.034 -> 0.035 / 0.035; x1 0.037 / 0.036 -> 0.037 / 0.036; x2 0.048 / 0.041 -> 0.049 / 0.043; x3 0.071 / 0.062 -> 0.070 / 0.061; x4 0.170 / 0.050 -> 0.061 / 0.054. The earlier headline "0.050 vs 0.170 at 4x severity" was mostly a baseline particle-depletion failure, not an effect of reliability weighting. The honest picture: no difference when clean or at x1; the reliability-aware filter is about 10-25% better at x2-x4 (better in 4 of 5 seeds at x4, one tie).
- Detector gating decision REVERSED by a live A/B. The M1 change (the detector counts every valid beam, following the paper's rule "reliability is not environmental importance") failed in live 3x missions in 2 of 2 disturbance seeds: NO_ROUTE after 230 s with phantom blockages on A-J1 and on J1-J2 (the corridor with the real debris B2), and a second run still driving at 230 s with a phantom blockage on A-J1; the gated detector passed 2 of 2 (goal in 84 s, all checks). Corrupted beams create phantom obstacles, and water returns (longer than the truth) let rays erase real evidence. The sandbox ablation had missed this because it only counted phantom blocked edges; there, B2 under 3x disturbance was already fragile (6-7 cells vs 40 clean, lost in 2 of 6 seeds ungated vs 1 of 6 gated). Default restored: `use_reliability` true in `blockage_detector_node.py` and `mission.launch.py`; `use_reliability:=false` stays as the ablation. Decoupling obstacle evidence from reliability would need a persistence mechanism first (deferred).
- Risk to watch: B2's evidence under heavy water disturbance is only a few cells, close to the 6-cell cluster threshold; a hardening step (for example unreliable rays not erasing evidence) is a candidate if the evaluation shows lost B2 detections.
- Tests: 29 sandbox tests pass (new: sigma credibility at the 2-sigma >= 0.80 and 3-sigma >= 0.97 bars, since a single 780-step recording is noisy: seed 3, reliability-aware, gave 0.849; over five tuning seeds the clean coverage is 0.94). `tests/mission_verify.sh` now accepts MISSION_ARGS (for example `MISSION_ARGS="use_reliability:=false"`); `tests/m1_ab.sh` is the live A/B.


### M2 findings (2026-11-01)

- What was built. `sewer_localization/disturbances.py`: `OdometryErrorModel` (per-run distance scale error N(0,1%), wheel-yaw scale error N(0,3%), gyro bias N(0,1 mrad/s) with a random walk, wheel noise only while moving, gyro noise) and `make_slip_schedule(severity)` (wheel-slip bursts in the windows 28-38 s and 62-72 s: the wheels report +5% distance and -5% rotation per unit severity, capped at 30%; the gyro is unaffected). `sandbox/sim2d.py: simulate()` uses the same model (`odom_slip=False` reproduces the old numbers bit for bit: verified against reference arrays for 2 seeds x clean/disturbed). New `odom_disturbance_node` (/odom -> /odom_disturbed, /imu -> /imu_disturbed, seeded, slip clock starts when the robot first moves). `localizer_node` has a new `tf_odom_topic` so `map->odom` is composed against Gazebo's TRUE odometry while the motion model reads the disturbed one. `localization.launch.py`: `odom_disturbed:=true odom_severity:=<slip severity> odom_scale:=<error scale>`; `tests/mission_verify.sh` takes `LOC_ARGS`. The mission observer now logs dead reckoning and the baseline filter (columns `dr_*`, `bl_*`) and prints all three error statistics.
- LIVE RESULT (scan severity / slip severity, disturbance seed 0, one mission each; position error vs ground truth):
  - clean scans, slip x1: dead reckoning RMSE 0.70 m (final 1.24 m); baseline 0.056 m; reliability-aware 0.056 m; mission passes. 
  - clean scans, slip x3: dead reckoning 0.94 m (final 1.78 m); baseline 0.097 m (max 0.40); reliability-aware 0.111 m (max 0.45); mission passes.
  - scans x3 AND slip x3: dead reckoning 4.5 m; baseline 5.2 m (max 14.8); reliability-aware 8.0 m (max 12.3): both filters lose track completely, the mission ends in NO_ROUTE with a collision. The slip bursts and the water windows overlap, so the odometry is wrong exactly when the scans are unreliable and nothing is left to hold the estimate. Demo goal 4 is met by the first two conditions; the third is the case the M3 health state must catch (stop instead of driving blind), and the evaluation grid must grade scan x odometry severities rather than start at the corner.
- With clean scans the two filters are equal (0.056 vs 0.056 m; at slip x3 the baseline is even slightly better, 0.097 vs 0.111 m, single run): reliability weighting only matters when the scans are disturbed.
- BUG FOUND AND FIXED while verifying (shared by both filters). Realistic odometry made the estimate jump to 1 m error in the last two seconds and end 0.5 m INSIDE the wall behind J4. Cause: odometry that overshoots pushed particles through the wall, and a particle inside a wall has every beam endpoint at distance 0 from an obstacle, i.e. the best possible likelihood, so it hijacked the estimate. Fix: walls block the particles (`PFParams.wall_block` = 0.10 m: a move that would end within 10 cm of an obstacle is rejected). Regression test `test_walls_block_the_particles` (without the fix the estimate lands at x = -1.60 m inside the wall; with it at -0.44 m in free space). Effect live: clean scans / slip x3 worst error 1.0 m -> 0.40-0.45 m; compound case worst error 33 m -> 12-15 m (still lost).
- Side effect to remember: when the localization error is large the blockage evidence grid is polluted (slip x3, clean scans: the J1-J4 obstruction cluster smears to s_max 8.0 m; compound case: phantom blocked edges M-J3 and J3-J4 and then NO_ROUTE). Obstacle detection is only as good as the pose it is placed with.
- Tests: 38 sandbox tests pass (new: 7 for the odometry model, 2 for wall blocking). Sandbox results from before M2 are not comparable with the 'disturbed' scenario any more (it now includes slip bursts); M5 re-runs everything.


### M3 findings (2026-11-01)

- What was built. `sewer_localization/health.py` (pure Python, 9 unit tests): each signal is smoothed (tau 1 s) and mapped to a stress (0 at `ok`, 1 at `bad`); the state follows the worst stress (DEGRADED at 0.5, CRITICAL at 1.0) with hysteresis (0.5 s to step up, 2 s below 0.8x threshold to step down) and a 3 s warm-up. Thresholds (ok, bad) set on the sandbox tuning seeds 100-109: fit (0.12, 0.18) m, R (0.90, 0.45), gyro disagreement (0.06, 0.15) rad/s; each `ok` sits just beyond the worst clean value (clean peaks: fit 0.115, R never below 0.93, gyro 0.054). `localizer_node` publishes `localization_health` (JSON) and colours the status text. `sewer_planning.pursuit.health_speed_factor` + `route_follower_node` (`health_topic`): 1.0 / 0.5 / 0 (stop), after 8 s of continuous CRITICAL creep at 0.3x so the robot never waits forever. `mission.launch.py health_response:=false` switches the response off (for the evaluation). `tests/mission_check.py` logs the health state and prints time per state.
- Calibration finding (honest limit). The first guess also used the particle spread (sigma) and the innovation rate (scan correction per metre travelled). The tuning data showed sigma is ANTI-informative (a lost filter is confidently wrong: AUC 0.24-0.26 for "error > 0.5 m") and the others are weak (fit 0.70-0.71, R 0.68, gyro 0.56, innovation 0.53-0.57); R median is about 0.7 at every severity from 2 to 5, so it cannot tell "disturbed but fine" from "failing". sigma and innovation were dropped. Result with the final thresholds, 10 tuning seeds per condition (sandbox, distance-based slip): clean runs are RELIABLE 100% of the time for both filters; the reliability-aware filter is non-RELIABLE 41% / 51% / 58% / 56% of the time at severity 2 / 3 / 4 / 5 (baseline, which has no R: 9% / 12% / 19% / 16%). When CRITICAL, only 0-12% of the steps at severity 2-3 and 53-63% at severity 4-5 actually had error > 0.5 m, and CRITICAL covers only 47% / 29% of the failing steps at severity 4 / 5 (86% at severity 3, but from very few failing steps), 14-16 s after the failure starts. So the state means "the scans or the odometry are being corrupted right now": it does not predict that the filter has lost track. We make no divergence-prediction claim; whether slowing/waiting helps is a question for the M5 evaluation (health response on vs off).
- Validity change made on the way: the odometry slip zones are now DISTANCES along the route (10-13.5 m and 21.5-25.5 m travelled, true distance) instead of time windows, so slowing down or waiting cannot trivially shorten the slip exposure. The scan disturbances stay time-based (transient conditions: waiting them out is legitimate). All earlier sandbox numbers that include slip change slightly; M5 re-runs everything.
- LIVE (Gazebo, scan severity 2, slip severity 2, health response on, disturbance seeds 1 / 2 / 3; one mission each): the state changed exactly in the three scan-disturbance windows (DEGRADED at about 9 s, CRITICAL 10.7-18.4 s, again 47-58 s and 63-74.5 s, identical in all three seeds: it is driven by the scan schedule), the robot stopped inside the windows and went on afterwards, the goal was reached every time, mission time 112-114 s (84 s with the response off, seed 1 only), reliability-aware RMSE 0.112 / 0.087 / 0.101 m against dead reckoning 0.57 / 0.26 / 0.72 m. Clean scans, no odometry error: the state never left RELIABLE (83 s mission, all checks pass).
- Open points, NOT resolved by a single run each: (a) seed 1, response on: the circumscribed-clearance check beside B2 read 0.00 m (0.10 m with the response off, 0.05 m on seeds 2 and 3 and on the clean run); the 0.7 m gap is tight so this is probably run-to-run variance, but contact sensors (M4) must say if it was a real collision. (b) seed 2, response on: B2 was flagged "blocked" (gap 0.45 m < 0.50) at the end although the robot drove past it; B2 evidence under heavy disturbance is the known fragile point, and a robot waiting parked in a disturbance window collects repeated corrupted returns, so the response may make it worse. Both are for the M5 batch (10 seeds, response on vs off) to settle.
- Tests: 49 sandbox tests pass (new: 9 health + follower speed factor, odometry tests rewritten for distance zones, parity of slip with place not clock).


### M4 findings (2026-11-02)

- What was built. Contact sensors (`type="contact"`) on `base_link` and both wheel links in `sewer_robot.urdf.xacro` (the wheels stick out 0.1 m beyond the chassis, so they touch a side wall first; the casters sit inside the chassis outline). The contact system is a world plugin, so it was added to `test_arena.sdf`, `sewer_network.sdf`, `sewer_network_clear.sdf` and the template in `gen_network.py`. `sewer_eval/collisions.py` (pure Python, 5 tests): a contact pair is attributed to the obstacle model (`walls`, `debris_*`, `obstacle_box`); the floor (`floor`, `ground`) and robot-only pairs are ignored; a collision is counted per obstacle model when it touches after 1 s without touching it, so a long scrape is one collision plus a contact time.
- Gotcha found while verifying. Gazebo ignores the `<topic>` element of a contact sensor: each sensor publishes on `/world/<world>/model/sewer_robot/link/<link>/sensor/<link>_contact/contact`, which contains the world name. A fixed bridge entry for `/contacts` therefore delivered nothing (the first arena test counted 0 while the robot was pushing against the box). `robot_sim.launch.py` now builds the three bridge arguments from the world file name (`test_arena` or `sewer_network`) and remaps them to `contacts/<name>`.
- Verification. Arena: driving 2 m on open floor with the wheels in permanent floor contact produced 0 events; pushing into the arena box produced exactly 1 event (`obstacle_box`) with the contact time growing while it pushed. Live missions: clean scans, no odometry error -> 0 collisions, clearance 0.05 m, goal in 83 s. Scan x2 + slip x2, seed 1, health response on -> goal reached but 1 wall collision (0.23 s of contact at (8.3, 3.35), the narrow gap beside the real debris B2, clearance reading 0.00 m): the earlier 0.00 m clearance reading in M3 was a real touch, not rounding. The same seed with the health response off passed with 0.10 m clearance, seeds 2 and 3 with the response on showed 0.05 m; the 0.7 m gap leaves about 0.16 m per side for a 0.38 m wide robot, so a 0.15 m localization error is enough to touch the wall. Whether the health response changes this rate is for the M5 batch to measure.
- Tests: 54 sandbox tests (5 new for collision counting).


### M5 findings (2026-11-02): sandbox evaluation, 30 seeds

`sandbox/run_eval.py` (shared metrics in `sewer_eval/metrics.py`, 5 tests): seeds 0-29 per condition (the health thresholds were tuned on seeds 100-109, so there is no overlap), clean plus severity 1-5 where the scan disturbances and the wheel-slip zones scale together. Every estimator reads the same recording and the two filters share the particle-noise stream. Mean ± 95 % confidence interval over seeds (Student t); full data in `logs/eval/sandbox/results.csv`, plot `rmse_vs_severity.png`.

### Localization error (mean ± 95 % CI over seeds)

| condition | estimator | RMSE [m] | max error [m] | time above 0.5 m | recovery [s] | runs that ever exceed 0.5 m |
|---|---|---|---|---|---|---|
| clean | dead reckoning | 0.307 ± 0.064 | 0.587 ± 0.122 | 0.138 ± 0.063 | n/a | 18/30 |
| clean | baseline PF | 0.038 ± 0.002 | 0.107 ± 0.009 | 0.000 ± 0.000 | n/a | 0/30 |
| clean | reliability-aware PF | 0.039 ± 0.002 | 0.106 ± 0.009 | 0.000 ± 0.000 | n/a | 0/30 |
| x1 | dead reckoning | 0.323 ± 0.062 | 0.631 ± 0.120 | 0.164 ± 0.063 | 15.721 ± 2.223 | 19/30 |
| x1 | baseline PF | 0.050 ± 0.002 | 0.125 ± 0.007 | 0.000 ± 0.000 | 0.000 ± 0.000 | 0/30 |
| x1 | reliability-aware PF | 0.055 ± 0.002 | 0.127 ± 0.007 | 0.000 ± 0.000 | 0.000 ± 0.000 | 0/30 |
| x2 | dead reckoning | 0.372 ± 0.058 | 0.709 ± 0.114 | 0.211 ± 0.075 | 15.837 ± 1.986 | 20/30 |
| x2 | baseline PF | 0.097 ± 0.004 | 0.218 ± 0.011 | 0.000 ± 0.000 | 2.210 ± 0.440 | 0/30 |
| x2 | reliability-aware PF | 0.107 ± 0.004 | 0.258 ± 0.013 | 0.000 ± 0.000 | 3.179 ± 0.479 | 0/30 |
| x3 | dead reckoning | 0.442 ± 0.057 | 0.812 ± 0.107 | 0.299 ± 0.076 | 17.006 ± 1.549 | 26/30 |
| x3 | baseline PF | 0.276 ± 0.118 | 0.747 ± 0.367 | 0.053 ± 0.022 | 4.098 ± 0.689 | 28/30 |
| x3 | reliability-aware PF | 0.214 ± 0.008 | 0.573 ± 0.022 | 0.044 ± 0.012 | 4.435 ± 0.389 | 29/30 |
| x4 | dead reckoning | 0.525 ± 0.059 | 0.933 ± 0.100 | 0.407 ± 0.055 | 17.098 ± 1.609 | 30/30 |
| x4 | baseline PF | 1.920 ± 0.335 | 5.403 ± 0.834 | 0.330 ± 0.039 | 11.587 ± 1.167 | 30/30 |
| x4 | reliability-aware PF | 1.250 ± 0.552 | 3.217 ± 1.278 | 0.237 ± 0.057 | 7.913 ± 2.072 | 30/30 |
| x5 | dead reckoning | 0.617 ± 0.061 | 1.065 ± 0.096 | 0.470 ± 0.040 | 17.486 ± 1.449 | 30/30 |
| x5 | baseline PF | 4.043 ± 0.498 | 9.172 ± 0.793 | 0.515 ± 0.029 | 16.852 ± 1.331 | 30/30 |
| x5 | reliability-aware PF | 4.680 ± 0.294 | 10.164 ± 0.470 | 0.553 ± 0.016 | 18.342 ± 0.884 | 30/30 |

### Paired difference, baseline minus reliability-aware RMSE (positive = reliability-aware better)

| condition | mean difference [m] ± 95 % CI | reliability-aware better in | median per-seed ratio (rel./base) |
|---|---|---|---|
| clean | -0.0010 ± 0.0006 | 7/30 seeds | 1.02 |
| x1 | -0.0054 ± 0.0011 | 0/30 seeds | 1.10 |
| x2 | -0.0097 ± 0.0018 | 0/30 seeds | 1.10 |
| x3 | 0.0614 ± 0.1163 | 22/30 seeds | 0.97 |
| x4 | 0.6700 ± 0.4559 | 24/30 seeds | 0.70 |
| x5 | -0.6368 ± 0.4997 | 13/30 seeds | 1.00 |

Reading of the numbers (honest summary):
- Clean, x1, x2: both filters hold the estimate (0 of 30 runs ever exceed 0.5 m) and are within 5-10 mm of each other; the baseline is slightly BETTER at x1 and x2 (paired difference -5 mm and -10 mm, the baseline wins in 30 of 30 seeds at both). Dead reckoning alone is 0.31-0.37 m RMSE, so both filters remove about 70-90 % of the drift (clean 88 %, x2 73 %).
- x3: equal in the mean (reliability-aware 0.214 vs baseline 0.276 m, paired difference not significant), but the baseline is much less consistent (max error 0.75 ± 0.37 m against 0.57 ± 0.02 m). x4: the reliability-aware filter is better (1.25 vs 1.92 m RMSE, paired +0.67 ± 0.46 m, better in 24 of 30 seeds, median per-seed ratio 0.70) but BOTH filters lose track in every seed (max error 3-5 m). x5: both are lost, the reliability-aware one slightly worse, not significant.
- So the claim that survives: reliability weighting helps in the hostile regime (significantly at x4, more consistent but not better on average at x3) and does not help, it costs a few millimetres, when conditions are mild. The earlier M1 numbers ("10-25 % better at x2-x4", 5 seeds, before the slip zones became distances) were too optimistic.
- Diagnosis of the x2 penalty (10 seeds, `sandbox/diag_x2.py`): with scan disturbances ONLY the reliability-aware filter is better (RMSE 0.0466 vs 0.0525 m, better in 9 of 10 seeds); adding the wheel-slip zones flips it (0.1061 vs 0.1001 m, better in 0 of 10). Which part of the reliability machinery loses under slip was not isolated (candidates: the temporal-consistency term compares consecutive scans through the slipping odometry, and down-weighted beams slow the correction exactly when the odometry is wrong). Left as a documented limitation; no tuning was done on the reported seeds.


### World upgrade and network v2 (2026-11-02)

Request: a more complex map, "a bit of layout, more of looks". Decisions: add a second loop and a second full blockage (layout), richer visuals, and the visuals are decoration only (the wheel slip stays distance-based, so the sandbox results above stay valid).

- Generator (`src/sewer_gazebo/scripts/gen_network.py`): two networks from `NETWORKS`. `sewer_network` is the classic one (map, graph and blockage-truth files byte-identical to before, only the world files changed their looks). `sewer_network_v2` adds node F (16,10), edges E-F and F-J3, and a second full blockage B3 on M-J3 (x about 14); the map outline did not grow. File names follow one rule: suffix = name minus `sewer_network`. Every world is still called `sewer_network` inside (the contact-sensor bridge needs that name).
- Looks (visual only, no collision shapes): two-tone walls with a wet lower band and coping, grimy floor slabs, a translucent water channel, sludge patches, ladders and manhole collars at the dead ends, inflow pipes, lamps at junctions and bends, rubble on the debris. Procedural textures were not tried.
- LiDAR rule: the GPU LiDAR ray-casts VISUAL geometry at z about 0.13 m, so a decoration crossing that plane above free space would be a phantom obstacle. All decoration goes through `Decor`, which raises an error for any shape reaching z in [0.05, 0.25] m over a free cell (unit-tested). Live check (`tests/scan_vs_map.py`: live scans against the range the map predicts from the true pose, in the debris-free worlds): the new worlds show exactly the same 1.1 % short beams as the ORIGINAL world from git, all within +-5 deg of straight ahead (the robot's own front hardware, now excluded from the check): the decoration adds no phantom returns.
- Plumbing: `network:=sewer_network|sewer_network_v2` on `localization`, `mission`, `view_b` and `demo` launch files (demo defaults to v2); `NETWORK=` for `tests/mission_verify.sh`. `tests/mission_check.py` now judges generically from the graph and truth files: goal reached, the first route went through the unknown blocking debris, every blocking edge detected from at least 2 m, replan within 3 s of each detection, no phantom blocked edges, passable debris seen as passable, 0 collisions. `tests/demo_compose.py` builds its captions from the events (detections and replans) instead of fixed text.
- Tests: 64 sandbox tests pass (5 new in `test_network_v2.py`: v2 routes A-J1-J4 / loop / double loop, graph and truth contents, map outline, LiDAR-band rule, world name and no decorative collisions).
- Live results (Gazebo): classic clean: ALL PASS (goal in 83 s, 0 collisions), so the new looks changed nothing about the behaviour. v2 clean: ALL PASS, both blockages detected, two replans (A-J1-J4 > J1-J2-M-J3-J4 > J2-M-E-F-J3-J4), goal in 108 s, 0 collisions. v2 with scan x2 + wheel slip x2, disturbance seed 3: ALL PASS, goal in 139 s, 0 collisions, health went DEGRADED / CRITICAL in the three scan-disturbance windows; reliability-aware RMSE 0.093 m (baseline 0.088, dead reckoning 0.290). Single runs, not statistics. Seed 3 stays the demo seed (`demo.launch.py`).
- Debugging lesson: four stale project nodes (two localizers, planner, follower) left over from an earlier stopped run drove the parked robot away in every later simulator and made scan tests meaningless for an hour. Always list what is left (`ps -eo pid,etime,cmd | grep sewer_ws/install`) before a test; `tests/` cleanup scripts only kill by name.
- Open: the demo video was not re-recorded on v2 (the scripts `demo_record.sh`, `demo_viewb.sh`, `demo_compose.py` are ready); five temporary test worlds (`tmp_*.sdf`) from the LiDAR investigation are still in `src/sewer_gazebo/worlds/` and should be deleted; nothing of this is committed or pushed.



### Account 1, Phase 1: exploration + coverage (2026-11-08)

Build plan: two-account extension (Account 1 = mapping, navigation, mission control). Additive only: localization, reliability, disturbances, EvidenceGrid, RouteModel and the existing launch files are unchanged.

- Decisions: exploration runs on the known corridor graph (no SLAM, as the build plan says for Phase 1). Coverage unit = 0.5 m bin along a corridor, covered when the ESTIMATED pose passes within 0.6 m while localization is not CRITICAL. A detected full blockage splits its corridor into two dead-end stubs; the robot stops 0.6 m short of the debris. Frontier choice = cheapest entry by the existing uncertainty-aware RouteModel cost (only the 3 nearest entry nodes by plain path length are fully costed, for speed). Done = every accessible bin explored, then return to the entrance (COMPLETE), or INCOMPLETE with a reason if bins had to be skipped.
- Files: `sewer_planning/{coverage_tracker,exploration,exploration_node}.py`, `sewer_bringup/launch/exploration.launch.py`, `sandbox/explore_sim.py` (closed loop: PF -> detector -> explorer -> pure pursuit; truth only for sensors and offline scoring), `sandbox/tests/test_exploration.py`. `route_follower_node` got an opt-in `rearm_on_new_goal` (default false).
- Results (sandbox, clean sensors): v2 blocked seeds 0 and 1, v2 clear seed 2, classic blocked seed 0: all COMPLETE, 100 % tracker and truth-scored coverage, exactly the true blocking edges detected, min wall clearance >= 0.25 m. 78 sandbox tests pass (64 + 14).
- Not verified: the ROS node and launch file were not run (no ROS on the machine used); exploration under scan disturbance / wheel slip not evaluated yet.

### Account 1, Phase 3: Mission Manager and task prioritization (2026-11-08)

- Decisions: one MissionManager (pure NumPy) owns every state of build-plan section 5, including recovery/safety states; INSPECT and UPDATE_MAP run continuously on every tick instead of being blocking states. It embeds the Phase 1 explorer, outputs routes for the existing follower (still the only /cmd_vel publisher) and tasks on /mission/task. Pickup/disposal success is accepted only from verified status flags. LOCALIZATION_RECOVERY keeps the follower's existing stop-then-creep response and blocks new tasks; > 60 s CRITICAL, a stale pose, a lost manipulator status while handling an object, or a controller fault -> SAFE_STOP.
- Priority rule: hard filters with reasons (stale, confidence, ungraspable, attempts, unreachable, no_return, carry), then 0.4 size + 0.3 obstruction + 0.2 accessibility + 0.1 graspability; size is relative to the largest feasible candidate (absolute volume let distance dominate, found by the unit test). Stand-off 0.25 m (0.35 m left objects just out of a 0.65 m reach in the first closed-loop run).
- Interfaces: PROPOSED JSON-in-String schemas in `docs/mission_interfaces.md`, parsed/validated in `mission_interfaces.py` (map frame only). Gripper limits are placeholders until Account 2 designs the arm.
- Results (sandbox, clean sensors, Account-2 STAND-INS): v2 seed 0 with the first S3 grasp forced to fail: MISSION_COMPLETE, S1, S3, S2 picked up, carried, disposal verified (simulator agrees), L1 and the B1/B2/B3 piles rejected as ungraspable, 4 defects reported, coverage 100 %, back at A. 15 new tests.
- Not verified: mission_manager_node / mission_manager.launch.py not run (no ROS on this machine); no real Account 2 nodes yet; not run under scan disturbance or slip.

### Account 1, Phase 5: disposal, resumption and robustness (2026-11-08)

- Carrying: on the way back the Mission Manager treats corridors narrower than max(robot 0.30 m, object width) + 2 x 0.05 m as blocked (`carry_status`); targets with no wide-enough route back are rejected up front. With the placeholder gripper this never exceeds the detectors 0.50 m blocking threshold, so it only matters for a wider carry. Station and gripper placeholders now live in `sewer_planning/config/mission.yaml`. Report gains `resumptions`, `holding`, `station`.
- Sandbox now runs the shared scan-disturbance schedule (repeated every 100 s), wheel-slip zones, the real `HealthMonitor` and the followers health speed response, so LOCALIZATION_RECOVERY is exercised closed-loop.
- Bug found by the first disturbed run (x2 scan + x2 slip, seed 0) and fixed: the detector re-flagged passable debris B2 as a full blockage after the robot had driven past it; accessibility was computed from the entrance only, so the robots own surroundings became "unreachable", coverage read 18/18 m = 100 % and the mission ended MISSION_COMPLETE without visiting half the network. Fixes (Account 1 code only, detector unchanged): (1) the explorer remembers stretches it actually drove; a blockage it drove through counts as passable-narrow; (2) accessibility = reachable from the entrance OR from the robot; (3) no route home is a new non-success state NO_ROUTE_HOME: wait up to 60 s for the evidence to clear, then SAFE_STOP (same for carrying back).
- Evaluation (`sandbox/run_mission_eval.py`, v2, seeds 0-4, all runs kept; `logs/eval/mission/`): clean 5/5 complete, 15/15 objects removed; x2 scan + x2 slip 5/5, 15/15; x3 + x3 4/5, 13/15. 0 false successes, 0 scrapes in all 15. The x3 seed 1 failure: localization error reached 0.58 m, the robot believed it was at the station but was outside the 0.6 m zone, disposal was correctly not verified twice and the mission SAFE_STOPped while still holding S3 (reported).
- Tests: 100 pass + 1 opt-in (`SEWER_SLOW_TESTS=1`, disturbed mission, passes). ROS nodes still not run (no ROS on this machine).

### Account 2: perception, inspection and manipulation (2026-11-10)

- Decisions: depth camera added on the existing camera_link (small debris is below the 2D LiDAR scan); classical baseline (frame-to-wall registration, height-band clustering, tracker, L-shape fit), no deep learning. Graspable only when seen up close and from two directions. 5-joint arm (turret +-1.9 rad, shoulder/elbow/wrist pitch, wrist yaw) + two-finger gripper with contact sensors on a mast above the LiDAR. Grasp verified from both finger contacts on the same object + finger gap; never from a command acknowledgement, no teleporting. Messages follow docs/mission_interfaces.md exactly (Account 2 adapted to Account 1's schema). The arm waits for the base to stop before a pickup and can reach objects beside the robot (the Mission Manager may already be level with an object that only became graspable late).
- Files: see ACCOUNT2_README.md section 1. No Account 1 file changed.
- Results (offline, NOT Gazebo, end-to-end with Account 1's Mission Manager): seeds 0 and 1 clean: MISSION_COMPLETE, coverage 100 %, 4/4 graspable objects disposed and confirmed in the tray, 0 failures, debris P/R 1.0/1.0. Seed 0 with x2 scan noise + x2 slip: MISSION_COMPLETE, 3/4 disposed, S2 reported failed (no finger contact). 0 false successes in all runs. Bugs found by these runs and fixed: objects beside the robot (turret +-1.9, arc moves, odometry-carried sighting), pickup while still rolling (wait for rest), disposal verified with the tray mislocated by a 20 cm localization error (tray located from its rims by the depth camera; sim pose can veto).
- Not verified: full pick-and-place and full mission in Gazebo with real physics; first laptop run grasped S1 with both contacts verified.
