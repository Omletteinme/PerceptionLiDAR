# Account 2 — Perception, Inspection & Manipulation

Workstream B of the build plan: object-level debris, structural defects, a simulated arm with a two-finger gripper
doing verified pick → carry → release, and the status messages Account 1's Mission Manager consumes
(`docs/mission_interfaces.md`, accepted in `docs/account2_interface_review.md`). Simulation only.

Built on top of `main` (Account 1's exploration, Mission Manager and disposal work). **No Account 1 file was changed.**

## Status at a glance

| Build-plan item (Account 2) | Status | How it was checked |
|---|---|---|
| Camera stream (and depth) integrated and validated | done; depth camera added | Gazebo on the WSL2 laptop: `/robot_depth` 320×180 `32FC1`, `/robot_cam/image`, transforms; offline camera-model tests |
| Separate debris objects (not one blockage) | done | offline evaluation + end-to-end mission (section 5) |
| Dimensions, map position, category, confidence, removal feasibility | done | same; feasibility checked against the scenario design |
| Cracks, holes, deformation, damaged joints | done | offline evaluation: all 9 found, correct type |
| Severity + location + evidence image | done | evidence PNGs written by the node; checked offline and on the laptop |
| Controlled synthetic dataset from Gazebo scenes, simple baseline first | done; classical baseline, no deep learning | offline renderer dataset; Gazebo recorder + labeller + `--dataset` evaluation (to be run on the laptop) |
| Fixed-base arm + two-finger gripper, design from robot geometry | done; 5 joints (see 2.2) | URDF ↔ kinematics test; Gazebo: arm joints and controllers alive, real grasp reached on the laptop |
| Approach / grasp / lift / carry / release, verified from contact/state evidence | done | kinematic stand-in tests; end-to-end mission with Account 1's Mission Manager; **Gazebo: grasp verified, full cycle still to be re-run** |
| Clean status/results to the Mission Manager, no state machine | done | every message passes Account 1's own parser; end-to-end mission |

**What still needs Gazebo on the laptop:** the full pick-and-place and full mission with real physics (stages `pick`
and `mission` in section 4). The first `pick` run on the laptop detected S1, drove to it and **grasped it with both
finger contacts verified**; it then reported a slip during the lift, which came from a too-strict contact check that
has since been fixed (flickering contact messages). Expect some tuning (controller gains, friction).

---

## 1. Files (only two existing files changed, both Account 2's)

| Area | Files |
|---|---|
| Test scene | `sewer_gazebo/scripts/inspection_scene.py` (one description of debris, defects and tray), `gen_inspection_world.py` → `worlds/sewer_inspection.sdf`, `config/inspection_truth.yaml` (**evaluation only**), `config/inspection_station.yaml` (a-priori station config) |
| Perception (`sewer_perception`) | `camera_model.py`, `debris_detector.py`, `object_tracker.py`, `feasibility.py`, `defect_detector.py`, `inspection_core.py` (whole pipeline, no ROS), `interfaces.py` (documented messages), `inspection_node.py`, `dataset_recorder_node.py`, `ros_utils.py`; changed: `setup.py`, `package.xml` |
| Arm (`sewer_manipulation`, new package) | `arm_kinematics.py`, `manipulation_controller.py`, `manipulation_bridge.py` (task/status glue, no ROS), `manipulation_node.py`, `scan_self_filter_node.py`, `launch/inspection_sim.launch.py`, `launch/inspection.launch.py`, `config/bridge_inspection.yaml` |
| Robot | `sewer_description/urdf/sewer_robot_inspection.urdf.xacro` (includes the base robot unchanged) |
| Sandbox | `render3d.py` (offline depth/RGB renderer), `run_inspection_eval.py`, `inspection_scoring.py`, `label_dataset.py`, `mission_sim_account2.py` (end-to-end with Account 1), `fake_arm_world.py`, `tests/test_inspection.py`, `tests/test_manipulation.py` |
| Gazebo checks | `tests/account2_live_check.sh` (topics / pick / mission / dataset), `tests/account2_pick_place_check.py`, `tests/account2_mission_check.py` |
| Docs | this file, `docs/account2_interface_review.md` |

The ROS nodes, the offline evaluation and the end-to-end mission all run the same `InspectionCore` and
`ManipulationBridge` classes, so what is tested offline is what runs in ROS.

---

## 2. Design

### 2.1 Perception
- **Depth camera** (320×180, on the existing `camera_link`, same view as `robot_cam`): the 2D LiDAR scans at 0.13 m,
  so debris small enough to pick up is below the scan and invisible to it.
- **Debris**: each depth frame is first aligned to the a-priori walls (frame-to-wall registration, removes a few cm /
  degree of localization error), points above the floor film and clear of the walls are clustered, and a tracker
  fuses clusters into objects with stable IDs and dimensions (L-shape rectangle fit). Large piles also come from the
  existing LiDAR evidence grid (unchanged); faces of one pile seen from both sides are joined through never-observed
  space. An object is `graspable` only if it fits the gripper (width ≤ 9 cm, height 3-15 cm, length ≤ 32 cm,
  estimated mass ≤ 0.5 kg from volume × assumed 600 kg/m³), is clear of the wall, and has been seen from within
  1.2 m (from far away the low camera sees only the front face).
- **Defects**: LiDAR deviations from the mapped wall (after a robust per-wall line fit) → recess (hole if > 8 cm
  wide, else damaged joint) or bulge (deformation); camera: thin dark lines on the upper wall (morphological
  black-hat) → cracks, length measured in 3D from the depth image. Severity = documented size rule. Evidence: the
  camera frame with the crack boxed, or the frame where the LiDAR defect was in view.

### 2.2 Arm (5 joints + 2 fingers)
Mast on top of the LiDAR housing → turret (±1.9 rad) → shoulder, elbow, wrist pitch (links 0.32 + 0.30 m, gripper
kept horizontal) → **wrist yaw** (±1.7 rad) → two sliding fingers (opening 0-11 cm), contact sensor on each finger.
Why this design: folded, nothing crosses the LiDAR scan plane; it reaches 0.25-0.60 m ahead, 0.3 m to the side and
objects right beside the robot, which covers where the Mission Manager stops (0.25 m before the object, ±0.25 m, or
already level with it when the object only became graspable late). Arm moves swing round the turret axis.
The arm starts a pickup or disposal only once the robot has stopped (the Mission Manager sends the task while the
robot may still be rolling) and locates the object again from there; if the base moves during a grasp it aborts. The wrist yaw was added after the
end-to-end test showed that, because the Mission Manager always stops facing along the corridor, a 4-joint arm could
not grip objects lying at an angle or across the corridor. The gripper is lowered onto the object from above.

### 2.3 Verification (no teleporting, no attach-by-script, never from a command acknowledgement)
- **grasp**: both finger contact sensors touch the same object (not wall/floor/robot) and the fingers stalled at a gap
  matching the object.
- **lift / carry**: the finger gap stays at the object's width (if it slips out the fingers close further) and both
  fingers keep reporting contact (gaps under 1 s tolerated: Gazebo's contact messages flicker); optionally Gazebo's
  object pose rose ≥ 8 cm (simulated state, allowed by the plan, `use_sim_pose`).
- **release**: no finger contact after opening. **disposal**: released, tray within reach, and the depth camera sees
  new points inside the tray, which it first locates from its rims (so a localization error cannot move the check
  off the tray), and/or the simulated object pose is in the tray. If the simulated pose says outside, it is never
  verified.

---

## 3. Messages (docs/mission_interfaces.md, unchanged)

Account 2 publishes exactly the documented fields; details and extra fields in `docs/account2_interface_review.md`.
Tasks handled: `PICKUP` (grasp + lift → `HOLDING`, `verified`), `DISPOSE` (release into the tray → `/disposal/status`
`released`, `verified`), `STOP` (freeze, keep holding). Controller faults start with `controller:` (SAFE_STOP),
disposal failures with `failed:`.

**Values Account 1 should put in `sewer_planning/config/mission.yaml`** (their file): `gripper_max_width: 0.11`,
`gripper_max_length: 0.32`, `gripper_max_height: 0.15`; keep `use_station_xy: false` (the tray is reachable from
anywhere within the 0.25 m arrival tolerance of node A, facing the end wall).

---

## 4. Run it on the laptop (WSL2)

```bash
cd ~/sewer_ws && colcon build --symlink-install && source install/setup.bash
cd sandbox && python3 -m pytest -q                       # all sandbox tests (Account 1 + Account 2)
python3 mission_sim_account2.py --seed 0                  # end-to-end with Account 1's Mission Manager, offline
python3 run_inspection_eval.py --scenario all             # perception evaluation, offline
bash ~/sewer_ws/tests/account2_live_check.sh topics       # Gazebo stage 1: sensors, arm, bridges
bash ~/sewer_ws/tests/account2_live_check.sh pick         # Gazebo stage 2: detect → pick → carry → tray → verify
bash ~/sewer_ws/tests/account2_live_check.sh mission      # Gazebo stage 3: full mission with the Mission Manager
bash ~/sewer_ws/tests/account2_live_check.sh dataset      # Gazebo: mission + recorded, labelled dataset + evaluation
```
Results go to `~/sewer_ws/logs/account2/` and `~/sewer_ws/logs/evidence/`. If `inspection_scene.py` changes,
regenerate with `python3 src/sewer_gazebo/scripts/gen_inspection_world.py --out src/sewer_gazebo` (a test fails if
the generated files are stale).

**Tune in Gazebo if needed:** arm oscillates → lower `p_gain` / raise `d_gain` in the xacro controllers; arm sags →
raise `i_gain`; object slips → raise finger `mu1/mu2` or `CtlParams.squeeze`; `/sim/dynamic_poses` empty → disposal
is still verified by the camera (the topics check prints Gazebo's own pose names to fix the bridge).

---

## 5. Results (offline, NOT Gazebo)

End-to-end mission, offline (`sandbox/mission_sim_account2.py`): Account 1's real Mission Manager, SimRobot and
particle filter + Account 2's real perception and arm logic; only the arm physics is a kinematic stand-in (an object
is caught when its footprint lies between the open fingers).

| run | result | coverage | removed (simulator confirms in tray) | failed | debris P/R | defects P/R |
|---|---|---|---|---|---|---|
| seed 0, clean | MISSION_COMPLETE | 100 % | S1, S2, S4, S5 (4/4 graspable) | none | 1.0 / 1.0 | 1.0 / 1.0 |
| seed 1, clean | MISSION_COMPLETE | 100 % | S1, S2, S4, S5 (4/4 graspable) | none | 1.0 / 1.0 | 0.9 / 1.0 |
| seed 0, x2 scan noise + x2 wheel slip | MISSION_COMPLETE (22 localization recoveries) | 100 % | S1, S4, S5 | S2 (no finger contact, 2 tries; reported, not disposed) | 0.9 / 1.0 | 0.82 / 1.0 |

In every run S3 (too wide) and S6 (too heavy) were rejected as not graspable, the large piles as ungraspable, and
**every object reported removed really lay in the tray (0 false successes)**.

Bugs found by these runs and fixed (each has a test in `sandbox/tests/test_manipulation.py`):
- objects become graspable only once seen from two directions, so the robot is often already level with them:
  turret widened to ±1.9 rad, arm moves swing round the turret axis;
- the robot can still be rolling when PICKUP arrives: the arm waits until it has stopped;
- object beside the robot, out of the camera's view, while the pose estimate drifts: the last camera sighting
  (fused footprint centre, not the near faces of one view) is carried forward with wheel odometry (`/odom`);
- with a 20 cm localization error at the station an object was dropped beside the tray and wrongly verified: the
  depth camera now locates the tray from its rims, the drop aims at the tray it sees, and the simulated object pose
  (when used) can only veto a disposal, never be overruled.

Sandbox tests: 140 pass (Account 1 + Account 2), 1 opt-in slow test of Account 1's not run.

---

## 6. Known limitations

- Full pick-and-place and the full mission have not yet run in Gazebo with real physics (section 4).
- With heavy localization disturbance (x2 noise + x2 slip) one of four graspable objects was missed twice (reported
  as failed, not as removed).
- The arm physics in the offline tests is a kinematic stand-in (joints follow commands, fingers stop at the object).
- Debris dimensions come from the faces the camera saw; a cylinder seen from one side measures too narrow; shape
  (box vs round) is unreliable from one viewpoint.
- Perception precision drops when the pose estimate is poor (offline: 10 cm / 2.6° error → many extra objects, none
  marked graspable). Recommendation: trust new detections only while localization health is RELIABLE.
- Small debris is invisible to the LiDAR; the route follower does not avoid it (Account 1's decision, see review).
- Crack detection is a classical filter tuned on the offline renderer; Gazebo lighting produced 2 false cracks in the
  first laptop run; threshold to be checked on the recorded dataset.
- Mass is estimated from an assumed density; hole vs damaged joint is decided by width only; severity is a reporting
  rule, not an engineering assessment.
- The disposal tray is just inside the entrance (the plan says outside; lead decision).
- Nothing has been validated on real data.
