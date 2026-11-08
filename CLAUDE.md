# CLAUDE.md

Project: **Reliability-Aware Localization and Path Planning for Sewer Inspection** (robotics minor project).
Full plan, roadmap and demo requirements: see `PLAN.md`. Read it at the start of every session.

## Fixed environment (do not change without asking)

- Windows 11 host, **Ubuntu 24.04 in WSL2**. All work happens inside the Linux filesystem (`~/...`), never `/mnt/c/...`.
- **ROS 2 Jazzy**, **Gazebo Harmonic** (via `ros_gz`), **Nav2**, **RViz2**.
- Python 3 from the system / ROS install. Do not create a separate Windows Python environment. Do not `pip install` packages that ROS already provides via apt.
- Build with `colcon` from the workspace root. Source `/opt/ros/jazzy/setup.bash` and `install/setup.bash`.
- VS Code connected through the WSL extension.

## Autonomy: you run everything

**You execute all commands yourself.** Installs, `apt`, `rosdep`, `colcon build`, `ros2 launch`, `ros2 run`, `ros2 topic`, `gz sim`, tests, experiments, plotting. Do not hand me commands to run. Run them, read the output, fix problems, and re-run until it works.

The only things you ask me for:
- A visual confirmation of the Gazebo/RViz windows when a milestone needs it (tell me exactly what to look for).
- Decisions listed under "Open decisions" in `PLAN.md`.
- Anything outside Ubuntu (Windows settings, WSL config, GPU drivers on Windows).
- If `sudo` asks for a password, stop and tell me; don't try to work around it.

### Running ROS in a non-interactive shell

- Every shell command starts fresh. Prefix ROS commands with `source /opt/ros/jazzy/setup.bash &&` (and `source ~/sewer_ws/install/setup.bash &&` once the workspace is built).
- **Never block on a long-running process.** Run `ros2 launch`, `gz sim` and nodes in the background, redirect output to `~/sewer_ws/logs/<name>.log`, then inspect with `tail`/`grep`. For a quick check, wrap in `timeout`, e.g. `timeout 20 ros2 topic echo --once /scan`.
- **Default to headless** for verification and experiments: `gz sim -s -r <world>` (server only), Nav2 launch files with `headless:=True`, no RViz. Only open GUIs when I'm doing a visual check or for the final demo.
- **Clean up after every run**: kill what you started (`pkill -f "gz sim"`, `pkill -f ros2`, `pkill -f rviz2`), and run `ros2 daemon stop` if topics look stale. Before a new run, check nothing from the last run is still alive.
- Use `use_sim_time:=true` everywhere.
- Install missing dependencies with `rosdep install --from-paths src -y --ignore-src`, or `sudo apt install -y ros-jazzy-<pkg>`.

### Verifying without seeing the screen

Check every milestone with commands, not assumptions:
- `ros2 topic list`, `ros2 topic hz /scan`, `ros2 topic echo --once /odom`
- `ros2 run tf2_tools view_frames` (inspect the generated PDF)
- `check_urdf` on the expanded Xacro output
- `ros2 bag record` during runs, then a Python script that plots trajectory, estimated vs. ground-truth pose, particle spread and reliability scores to PNG. **Open those PNGs yourself** and judge whether the result is right.
- For scene checks, add a camera to the world, save frames to PNG, and look at them.

## How to work

1. **One module at a time.** Before writing code, state which module and milestone you're working on. Do not generate the whole project at once.
2. **Every module ships with**: complete files (no "..." placeholders), a build and run that you have actually executed, and a test or check that you have actually run and passed.
3. **Never assume something works.** Report what you ran and what the output showed. If only a GUI check can confirm it, say so and tell me exactly what to look for.
4. **When fixing errors**, change the smallest set of files possible, explain the cause in one or two lines, and re-run the failing step to prove the fix.
5. At the end of each milestone, update `PLAN.md` ("Decisions made" and the checklist) and give me a short summary: what works, what was verified and how, what's next.
6. **Algorithmic correctness and measurable comparison before visual polish.** No time on photorealistic water or fancy meshes until Days 13–14.
7. **Label what is simulated vs. custom.** In code comments and docs, clearly mark which parts come from Gazebo/Nav2/AMCL and which are our own algorithms.
8. If a requirement in `PLAN.md` is ambiguous (see "Open decisions" there), ask before choosing, or choose a sensible default and record it in `PLAN.md` under "Decisions made".

## Proposed workspace layout

```
sewer_ws/
  src/
    sewer_description/    # URDF/Xacro: chassis, wheels, IMU, 2D LiDAR, frames
    sewer_gazebo/         # worlds (pipe network, junctions, blockages), models, ros_gz bridge config
    sewer_localization/   # baseline particle filter + reliability-aware particle filter (Python)
    sewer_perception/     # LiDAR-based wall/debris/blockage detection (Python)
    sewer_planning/       # uncertainty-aware cost layer or planner, replanning around blockages
    sewer_bringup/        # launch files, params (Nav2, filters), RViz configs
    sewer_eval/           # experiment runners, logging, metrics, plots (NumPy, pandas, Matplotlib)
  sandbox/                # pure-Python fallback: 2D simulator + filters, no ROS needed
```

Adjust only with a stated reason.

## Coordinate frames (REP 105)

`map -> odom -> base_footprint -> base_link -> {imu_link, lidar_link, left_wheel_link, right_wheel_link}`

- `odom -> base_footprint`: from wheel odometry (drifts).
- `map -> odom`: published by the active localizer (baseline OR reliability-aware filter, never both at once).

## Topic interfaces (initial; update here if they change)

| Topic | Type | Source |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` (or `TwistStamped`, match Nav2 config) | Nav2 / teleop |
| `/odom` | `nav_msgs/Odometry` | Gazebo diff-drive plugin via bridge |
| `/imu` | `sensor_msgs/Imu` | Gazebo IMU sensor via bridge |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo 2D LiDAR via bridge |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo; all nodes use `use_sim_time:=true` |
| `/ground_truth/odom` | `nav_msgs/Odometry` | Gazebo true pose (evaluation only, never fed to the filter) |
| `/particle_cloud` | `geometry_msgs/PoseArray` (or `nav2_msgs/ParticleCloud`) | localizer |
| `/estimated_pose` | `geometry_msgs/PoseWithCovarianceStamped` | localizer |
| `/scan_reliability` | custom or `std_msgs/Float32MultiArray` | reliability-aware filter |
| `/blockages` | `visualization_msgs/MarkerArray` | perception |

## Hard rules for the research algorithm

- The proposed filter must **explicitly compute** a reliability weight per range measurement (temporal + geometric consistency) and use it in the measurement update. AMCL is only the baseline.
- Ground truth is for evaluation only. Never use it in localization or planning.
- Both filters must be runnable under **identical, repeatable** scenarios (fixed seeds, same world, same disturbance schedule).
- Keep the `sandbox/` version of each filter in sync with the ROS version so algorithm work never depends on the simulator running.

## Limits to keep in mind

- You can't see live Gazebo or RViz windows. Use the verification methods above; ask me for a screenshot only when nothing else will do.
- GUI apps under WSL2 depend on WSLg. If a window doesn't appear, check `echo $DISPLAY $WAYLAND_DISPLAY` and WSLg before debugging the code.
- Never delete anything outside `~/sewer_ws` and never touch `/mnt/c` (the Windows drive).

## Topic interface additions (Day 6-8, ROS wrappers)

| Topic | Type | Source |
|---|---|---|
| `/scan_disturbed` | `sensor_msgs/LaserScan` | `scan_disturbance_node` (seeded disturbances on `/scan`); localizers read this when `disturbed:=true` |
| `/scan_disturbance_kind` | `std_msgs/Int8MultiArray` | `scan_disturbance_node`; per-beam injected class, EVALUATION ONLY |
| `/<ns>/estimated_pose`, `/<ns>/particle_cloud` | `PoseWithCovarianceStamped`, `PoseArray` | `localizer_node`; `<ns>` = `baseline` or `reliability` with `filter:=both`, empty with a single filter |
| `/<ns>/scan_reliability` | `std_msgs/Float32MultiArray` | per-beam r_i (reliability filter only) |
| `/<ns>/scan_reliability_R` | `std_msgs/Float32` | scan-level reliability R (reliability filter only) |

With `filter:=both` only the reliability-aware localizer publishes `map -> odom`.

## Topic interface additions (Day 9-10, perception + planning)

| Topic | Type | Source |
|---|---|---|
| `/edge_gaps` | `std_msgs/String` (JSON `{"A-B": {gap, blocked, s_min, s_max, offset, observed, cells}}`) | `blockage_detector_node`; planner input |
| `/blockages` | `visualization_msgs/MarkerArray` | `blockage_detector_node`: unmapped-obstacle cells and a label per obstructed edge |
| `/route` | `nav_msgs/Path` (map frame) | `route_planner_node`: uncertainty-aware route from the robot to the goal |
| `/route_info` | `std_msgs/String` (JSON) | `route_planner_node`: chosen route, cost breakdown, alternatives, smoothed R, blocked/narrow edges |
| `/mission_status` | `std_msgs/String` | `route_follower_node`: WAITING, RUNNING, NO_ROUTE, GOAL_REACHED |
| `/cmd_vel` | `geometry_msgs/Twist` | `route_follower_node` (pure pursuit on the ESTIMATED pose) |

Run: `ros2 launch sewer_bringup mission.launch.py` after `robot_sim.launch.py` and `localization.launch.py filter:=both` (the /reliability/* topics are inputs).

## Topic interface additions (Account 1, Phase 1: exploration + coverage)

| Topic | Type | Source |
|---|---|---|
| `/route` | `nav_msgs/Path` (map frame) | `exploration_node` (replaces `route_planner_node` in exploration missions; still one motion commander: the follower) |
| `/exploration/status` | `std_msgs/String` (JSON: state INIT/EXPLORING/RETURNING/COMPLETE/INCOMPLETE, reason, frontiers, active_segment, target, candidates, localization) | `exploration_node` |
| `/coverage/status` | `std_msgs/String` (JSON: coverage, confidence, explored/accessible/blocked/unreachable/skipped/total metres, segments, per-edge states) | `exploration_node` |
| `/coverage/markers` | `visualization_msgs/MarkerArray` | `exploration_node`: corridor bins coloured explored/unexplored/blocked/skipped |

Run: `ros2 launch sewer_bringup exploration.launch.py network:=sewer_network_v2` after `robot_sim.launch.py` and `localization.launch.py filter:=both`. The follower runs with `rearm_on_new_goal:=true` (default false elsewhere). Sandbox: `python3 sandbox/explore_sim.py --seed 0`.
