# Account 2 review of `docs/mission_interfaces.md`

Status: **Account 2 accepts the proposed schemas as written** and implements them exactly
(`src/sewer_perception/sewer_perception/interfaces.py`). Every Account 2 message is checked against Account 1's own
parser in `sandbox/tests/test_inspection.py::test_messages_pass_account1_parser`, and the two sides have been run
together end to end offline (`sandbox/mission_sim_account2.py`, results below). Lead: please freeze the interface
when you agree.

## What Account 2 publishes

All documented required and optional fields, with these values:

| field | Account 2 value |
|---|---|
| debris `size`, `size_sigma` | measured footprint length/width and height from the depth camera; sigma 0.015 / 0.015 / 0.01 m (0.10 m on height when only the LiDAR saw the object) |
| debris `pos_sigma` | 0.04 m |
| debris `obstruction` | 1.0 if the free gap left beside the object is under 0.50 m, otherwise the fraction of the corridor width it occupies |
| debris `graspable` | Account 2's own check against the real gripper (width, height, length, estimated mass, wall clearance, seen from within 1.2 m) |
| debris `grasp_score` | width margin in the gripper, 0.2-1.0 (0 when not graspable) |
| debris `stamp` | last time the object was observed |
| defect `severity` | 0-1 score from the measured size (crack length, hole/joint width, deformation depth) |
| defect `extent` | crack length, or hole/joint/deformation width along the wall [m] |
| defect `evidence` | path of the saved camera frame (PNG), empty if none yet |
| manipulation `verified` | `HOLDING`: both finger contacts on the same object, finger gap matching it, lift verified. `RELEASED`: no finger contact after opening |
| disposal `verified` | released, tray within reach of where the robot stopped, and the object seen in the tray by the depth camera (the tray is located from its rims, so a localization error cannot move the check) or, if enabled, Gazebo's object pose in the tray; never if Gazebo's pose says outside |

Extra fields (ignored by the parser, useful in reports): `state`, `feasibility.reasons`, `feasibility.approach_poses`,
`obstruction_detail`, `severity_label`, `size` dict, `evidence_detail`, `detail_state`, `verification`,
`arm_in_scan_plane`.

Behaviour details:
- `lifting` is reported as `GRASPING`; `HOLDING` is only sent once the lift is verified, so `verified` is already true.
- Held and removed objects are not listed in `/perception/debris` any more (the Mission Manager keeps their last record).
- Failure reasons that are controller faults start with `controller:` (`controller: arm_timeout ...`). All others are retryable.
- Disposal failures start with `failed:` (`failed: drop point out of reach`, `failed: no evidence the object is in the tray`, ...).
- `STOP` freezes the arm where it is and keeps holding (state stays `HOLDING` if an object is held).
- `/mission/task` is subscribed TRANSIENT_LOCAL; a task ID seen before, or a task older than 10 s, is ignored.
- PICKUP / DISPOSE start once the robot has stopped (up to 8 s wait); the object is located again from there.
- If a PICKUP names an ID that perception has since merged into another track, the object at `target` (within 0.25 m) is used.

## Placeholders to replace in `sewer_planning/config/mission.yaml` (Account 1's file: please change)

| key | placeholder | Account 2 value | why |
|---|---|---|---|
| `gripper_max_width` | 0.15 | **0.11** | finger opening. Your check adds 1 sigma itself; Account 2's `graspable` keeps 1 cm per side in reserve |
| `gripper_max_length` | 0.30 | **0.32** | longest object carried folded over the robot |
| `gripper_max_height` | 0.15 | **0.15** | finger pad position |
| reach from stand-off | ≥ 0.65 m assumed | **0.25-0.60 m ahead, ±0.3 m to the side, and beside the robot** | covers your stand-off (0.25 m before the near face, arrival tolerance 0.25 m); checked in `test_mission_manager_stand_off_and_station_tolerance`, `test_object_beside_the_robot_is_picked` and the end-to-end run |
| disposal station | node A, 0.6 m radius | **keep `use_station_xy: false`** | the tray is against the entrance end wall (x −0.44…−0.24, `sewer_gazebo/config/inspection_station.yaml`); from anywhere within your 0.25 m arrival tolerance of A, facing the end wall, the tray is in reach and the arm aims at it |

## Points for Account 1 (not blocking)

1. Small debris (lower than 13 cm) is invisible to the 2D LiDAR, so the blockage detector and `/edge_gaps` do not
   know it. In the inspection world it lies off the centre line, but the route follower could drive into one that does
   not. Using `/perception/debris` positions as soft obstacles in the follower is your call.
2. `graspable` becomes true only once the object has been seen up close and from two directions (≥ 20° apart), so
   that a beam seen end-on is not mistaken for a small block. For objects beside the centre line that is often just
   after the robot has passed your stand-off point; you then send PICKUP right away and the arm reaches beside the
   robot, so nothing is needed on your side. Turning back to the stand-off point would only be nicer, not required.
3. While the arm is lowered it is in the LiDAR scan plane for a few seconds. `scan_self_filter_node` publishes
   `/scan_self_filtered` with those beams removed. Whether the localizers read it is a shared-launch decision for the lead.

## Decisions for the lead

1. Robot description: the inspection robot (`sewer_robot_inspection.urdf.xacro`) adds a depth camera and the 5-joint
   arm with gripper to the unchanged base robot. Approve before it replaces the base robot in shared launch files.
2. The build plan places the disposal station "outside the sewer entrance". The v2 map has a closed end wall at A, so
   the tray is just inside it. Moving it outside needs an opening in the shared map and world.
3. `/scan_self_filtered` for the localizers (point 3 above).
