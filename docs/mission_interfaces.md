# Mission interfaces: Account 1 ↔ Account 2 (PROPOSED)

Status: **proposed by Account 1, not yet agreed.** Account 2 and the lead should review this before either side
depends on it. After agreement, freeze it and record any change here.

All messages are JSON in `std_msgs/String`, the same convention the repo already uses for `edge_gaps`,
`route_info` and `localization_health`. This avoids a new message package before the schemas are frozen. The
consumer-side parser and validator is `src/sewer_planning/sewer_planning/mission_interfaces.py`. The Mission
Manager drops any message it rejects and lists it under `input_errors` in `/mission/report`.

## General rules (build plan §4)

- Every message has `frame_id` and `stamp` (sim time, seconds). **Only `frame_id: "map"` is accepted.**
  Perception transforms into `map` before publishing. Sensor-frame coordinates are never sent to the mission.
- Object IDs are **stable strings**. The same debris or defect keeps its ID across frames, so it isn't counted twice.
- Estimates carry `confidence` and uncertainty (`pos_sigma`, `size_sigma`). Dimensions are estimates, not exact values.
- Ground-truth labels (`blockages_truth*.yaml` or world metadata) are **never** used as live input. They are for evaluators only.
- "detected", "graspable", "holding", "released" and "disposal verified" are separate fields and states.

## Account 2 → Account 1

### `/perception/debris`
Publish at 1–5 Hz, containing every object currently tracked (not only the ones in view).
```json
{"frame_id": "map", "stamp": 123.4, "objects": [
  {"id": "D7", "x": 4.02, "y": 0.31, "z": 0.0, "yaw": 0.0,
   "size": [0.14, 0.10, 0.08], "size_sigma": [0.01, 0.01, 0.01], "pos_sigma": 0.03,
   "category": "brick", "confidence": 0.9, "obstruction": 0.1,
   "graspable": true, "grasp_score": 0.7, "stamp": 123.2, "evidence": "cam/000412.png"}]}
```
| field | required | meaning |
|---|---|---|
| `id`, `x`, `y`, `size` [l, w, h], `confidence` | yes | stable ID, map-frame centre, estimated size in metres, 0–1 |
| `stamp` (per object) | no (defaults to message `stamp`) | when the object was last observed. The mission rejects objects older than 30 s as stale |
| `size_sigma`, `pos_sigma` | recommended | 1-sigma uncertainty. The gripper check uses size + 1 sigma |
| `obstruction` | recommended | 0 = no obstruction, 1 = closes the corridor |
| `graspable` | optional | Account 2's own grasp check. `false` is a hard reject. `true` or absent still goes through the mission's size check |
| `grasp_score` | optional | 0–1. Used as the graspability term if present |
| `category`, `z`, `yaw`, `evidence` | optional | reported only |

### `/perception/defects`
```json
{"frame_id": "map", "stamp": 123.4, "defects": [
  {"id": "C3", "type": "crack", "x": 6.0, "y": -0.5, "severity": 0.4, "confidence": 0.85,
   "extent": 0.3, "stamp": 120.1, "evidence": "cam/000377.png"}]}
```
`type` must be one of `crack`, `hole`, `deformation` or `damaged_joint`. `severity` is 0–1. Defects are recorded and
reported only, never turned into tasks. Defect severity is kept separate from debris priority.

### `/manipulation/status`
Publish at ≥ 2 Hz at all times (the Mission Manager stops safely if the status is older than 5 s while it is
handling an object).
```json
{"state": "HOLDING", "stamp": 130.0, "object_id": "D7", "verified": true, "task_id": "T4", "reason": ""}
```
- `state`: `IDLE`, `APPROACHING`, `GRASPING`, `HOLDING`, `RELEASING`, `RELEASED` or `FAILED`.
- `verified` is **true only from simulated physical evidence**, such as contacts on both fingers or a lift check. A command acknowledgement doesn't count, and objects must never be teleported.
- `task_id` echoes the `/mission/task` request this status answers.
- `reason` explains a failure. A `reason` starting with `controller` (for example `controller: joint limit fault`) makes the Mission Manager go to SAFE_STOP. Any other failure is retried, up to 2 attempts per object.
- The mission waits for `IDLE` during INITIALIZE (20 s by default). Without it, the mission runs as inspection only.

### `/disposal/status`
```json
{"stamp": 140.0, "object_id": "D7", "task_id": "T5", "station_reached": true,
 "released": true, "verified": true, "reason": ""}
```
- `verified` = the object is no longer held **and** lies inside the disposal zone.
- A `reason` starting with `failed` ends the attempt early.

## Account 1 → Account 2

### `/mission/task` (latched, published once per request)
```json
{"task_id": "T4", "type": "PICKUP", "stamp": 128.0, "object_id": "D7",
 "target": {"x": 4.02, "y": 0.31, "yaw": 0.0, "frame_id": "map"}, "action": "grasp_and_lift"}
```
| type | when | Account 2 should |
|---|---|---|
| `PICKUP` | robot stopped at the stand-off point, about 0.25–0.45 m in front of the object, facing it | do the fine approach with the arm, grasp and lift, then report `HOLDING` + `verified` |
| `DISPOSE` | robot at the disposal station (`target` = station pose) | release, then check the object is inside the zone and report on `/disposal/status` |
| `STOP` | SAFE_STOP | stop the arm at once and keep holding whatever is held |

A retry always gets a new `task_id`. While a PICKUP or DISPOSE is open, the robot base does not move (the Mission
Manager publishes an empty route).

### `/mission/state`, `/mission/report`
State and reason at 2 Hz. The report (every 2 s, and once at the end) contains coverage, debris detected, removed,
failed, unreachable and rejected (with reasons), the last ranking with its score terms, defects, counts, the event
timeline and time spent in each state.

## Proposed placeholder limits (Account 2 to replace with the real arm design)

| parameter | value | used for |
|---|---|---|
| gripper max opening (`gripper_max_width`) | 0.15 m | the object's smaller horizontal size + 1 sigma must fit |
| max carried length (`gripper_max_length`) | 0.30 m | longer objects are rejected |
| max object height (`gripper_max_height`) | 0.15 m | |
| reach from stand-off point | ≥ 0.65 m assumed | stand-off is 0.25 m before the near face, and the follower stops within 0.2 m of its goal |
| disposal station | entrance node `A` (0, 0), zone radius 0.6 m assumed | the robot drives there carrying the object |

## Priority rule (Account 1, `task_prioritizer.py`)

1. Hard filters, in this order, each with its recorded reason: stale, low confidence (< 0.5), ungraspable (from perception or size), pickup already failed twice, unreachable, no route back to the station, does not fit through the narrowest gap on the way back.
2. `priority = 0.4·size + 0.3·obstruction + 0.2·accessibility + 0.1·graspability`, where:
   - `size` = volume / volume of the largest feasible candidate,
   - `accessibility` = exp(−route cost / 20 m), using the existing uncertainty-aware route cost,
   - `graspability` = (`grasp_score` or width margin) × confidence.

   This is a tunable baseline heuristic, not a proven optimal policy.
