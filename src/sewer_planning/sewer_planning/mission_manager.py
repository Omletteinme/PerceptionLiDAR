"""Mission Manager: the single authority that sequences the sewer mission.

CUSTOM code (ours), pure NumPy, no ROS imports (shared by sandbox/mission_sim.py and mission_manager_node).

States (build plan, section 5):
  INITIALIZE -> EXPLORE -> PRIORITIZE_TASKS -> NAVIGATE_TO_DEBRIS -> PICKUP -> VERIFY_PICKUP -> RETURN_TO_DISPOSAL
  -> DISPOSE -> VERIFY_DISPOSAL -> RESUME_EXPLORATION -> EXPLORE ... -> MISSION_COMPLETE
  recovery / safety: LOCALIZATION_RECOVERY, PICKUP_FAILED, NAVIGATION_FAILED, DISPOSAL_FAILED, UNREACHABLE_TARGET,
  SAFE_STOP.
INSPECT and UPDATE_MAP are not separate blocking states here: coverage, blockage evidence, debris and defect records are
updated on every tick in every state (the robot inspects while it drives). They appear in the report as activities.

What it does NOT do: it never runs perception or the arm (Account 2), never publishes wheel commands (route_follower_node
does, from the route this module outputs), and never reads ground truth. Success of a pickup or a disposal is only
accepted from the verified flags in the manipulation / disposal status, never from having sent a request.

tick() returns dict(state, reason, route, task, target, ...):
  route None  -> keep the follower's current route;  [] -> stop (empty path);  [pts] -> drive this polyline
  task        -> a new MissionTask to publish on /mission/task (or None)
"""
import math
from dataclasses import dataclass

import numpy as np

from .exploration import COMPLETE, INCOMPLETE, NO_ROUTE_HOME, Explorer
from .mission_interfaces import MissionTask
from .point_route import avoid_debris, plan_to_point
from .task_prioritizer import TaskPrioritizer, carry_status

INITIALIZE, EXPLORE, PRIORITIZE_TASKS = 'INITIALIZE', 'EXPLORE', 'PRIORITIZE_TASKS'
NAVIGATE_TO_DEBRIS, PICKUP, VERIFY_PICKUP = 'NAVIGATE_TO_DEBRIS', 'PICKUP', 'VERIFY_PICKUP'
RETURN_TO_DISPOSAL, DISPOSE, VERIFY_DISPOSAL = 'RETURN_TO_DISPOSAL', 'DISPOSE', 'VERIFY_DISPOSAL'
RESUME_EXPLORATION, MISSION_COMPLETE = 'RESUME_EXPLORATION', 'MISSION_COMPLETE'
LOCALIZATION_RECOVERY, PICKUP_FAILED, NAVIGATION_FAILED = 'LOCALIZATION_RECOVERY', 'PICKUP_FAILED', 'NAVIGATION_FAILED'
DISPOSAL_FAILED, UNREACHABLE_TARGET, SAFE_STOP = 'DISPOSAL_FAILED', 'UNREACHABLE_TARGET', 'SAFE_STOP'

TERMINAL = (MISSION_COMPLETE, SAFE_STOP)
HEALTH_LEVEL = {'RELIABLE': 0, 'DEGRADED': 1, 'CRITICAL': 2}
MOVING = (EXPLORE, NAVIGATE_TO_DEBRIS, RETURN_TO_DISPOSAL)
CARRYING = (VERIFY_PICKUP, RETURN_TO_DISPOSAL, DISPOSE)


@dataclass
class MissionParams:
    require_manipulator: bool = True    # wait for manipulation IDLE in INITIALIZE (else inspection only)
    init_timeout: float = 20.0          # [s] then continue without the manipulator (reported), or SAFE_STOP without pose
    arrive_tol: float = 0.25            # [m]
    pose_timeout: float = 2.0           # estimated pose older than this -> SAFE_STOP
    manip_timeout: float = 5.0          # manipulation status older than this while carrying -> SAFE_STOP
    pickup_timeout: float = 30.0        # [s] from PICKUP request to HOLDING
    verify_timeout: float = 5.0         # [s] for the verified flag after HOLDING / release
    dispose_timeout: float = 30.0       # [s] from DISPOSE request to a disposal result
    max_pickup_attempts: int = 2        # per object
    max_dispose_attempts: int = 2
    max_nav_failures: int = 3           # consecutive ticks without a route before a target is unreachable
    critical_max: float = 60.0          # [s] of continuous CRITICAL localization before SAFE_STOP
    reprioritize_period: float = 2.0    # [s] between debris re-evaluations while exploring
    stranded_timeout: float = 60.0      # [s] waiting for a route back to the entrance (evidence may clear) -> SAFE_STOP
    lidar_plane: float = 0.13           # debris lower than this is invisible to the 2D LiDAR: route around it explicitly
    detections_max_health: str = 'RELIABLE'   # ingest new debris/defect records only at this health or better: with a
                                              # poor pose perception reports phantom objects (Account 2's finding)


class MissionManager:
    def __init__(self, graph, model, explorer=None, prioritizer=None, params=None, home=None, station_xy=None):
        self.g, self.model = graph, model
        self.p = params or MissionParams()
        self.home = home or graph.start
        self.explorer = explorer or Explorer(graph, model, home=self.home)
        self.prio = prioritizer or TaskPrioritizer(model)
        self.station_xy = np.asarray(station_xy if station_xy is not None else graph.nodes[self.home], float)
        self.state, self.reason, self.t_state, self.t0 = INITIALIZE, 'starting', None, None
        self.resume_state, self.critical_since = None, None
        self.debris, self.defects = {}, {}              # id -> latest record
        self.manip = self.disposal = None
        self.target = None                              # ranked item being handled
        self.task, self.task_n = None, 0
        self.attempts, self.dispose_attempts, self.nav_failures = {}, 0, 0
        self.removed, self.failed, self.unreachable, self.rejections = {}, {}, {}, {}
        self.last_eval, self.t_eval = dict(ranked=[], rejected=[]), -1e9
        self.events, self.state_time, self.distance = [], {}, 0.0
        self.manipulation_enabled = True
        self.recoveries, self.input_errors, self.pickup_requests = 0, [], 0
        self.resumptions, self.carry_route, self.stranded_since = [], None, None
        self.detections_ignored = 0
        self.detoured, self.unavoidable = set(), set()
        self._last_xy, self._last_t, self.explore_result = None, None, None

    # ------------------------------------------------------------------ helpers
    def _go(self, now, state, reason=''):
        if state != self.state:
            self.events.append(dict(t=round(now, 2), frm=self.state, to=state, reason=reason))
        self.state, self.reason, self.t_state = state, reason, now

    def _elapsed(self, now):
        return now - (self.t_state if self.t_state is not None else now)

    def _new_task(self, now, kind, object_id='', target=None, action=''):
        self.task_n += 1
        self.pickup_requests += kind == 'PICKUP'
        self.task =MissionTask(task_id=f'T{self.task_n}', type=kind, stamp=now, object_id=object_id,
                                target=target or {}, action=action)
        return self.task

    def _status_for_task(self, s):
        return s is not None and self.task is not None and (s.task_id == self.task.task_id or
                                                            (not s.task_id and s.object_id == self.task.object_id))

    def _done_ids(self):
        return set(self.removed) | set(self.failed) | set(self.unreachable)

    def _prioritize(self, now, xy, cov, R, status):
        loc = self.g.locate(*xy)
        self.last_eval = self.prio.evaluate(list(self.debris.values()), loc, status, R,
                                            self.model.initial_covariance(cov), now, self.home,
                                            attempts=self.attempts, done=self._done_ids())
        self.t_eval = now
        for r in self.last_eval['rejected']:          # keep the most informative reason ever seen per object
            if r['reason'] != 'stale' or r['id'] not in self.rejections:
                self.rejections[r['id']] = r
        for r in self.last_eval['ranked']:
            self.rejections.pop(r['id'], None)
        return self.last_eval['ranked']

    # ------------------------------------------------------------------ main loop
    def tick(self, now, xy=None, pose_stamp=None, cov=None, R=0.9, health='RELIABLE', status=None,
             debris=None, defects=None, manip=None, disposal=None):
        """One decision step. xy = ESTIMATED (x, y) in the map frame; debris/defects = parsed records (lists)."""
        status = status or {}
        if self.t0 is None:
            self.t0, self.t_state = now, now
        if HEALTH_LEVEL.get(health, 2) <= HEALTH_LEVEL[self.p.detections_max_health]:
            for d in debris or ():
                self.debris[d.id] = d
            for d in defects or ():
                self.defects[d.id] = d
        elif debris or defects:
            self.detections_ignored += 1
        if manip is not None:
            self.manip = manip
        if disposal is not None:
            self.disposal = disposal
        if xy is not None:
            if self._last_xy is not None:
                self.distance += math.hypot(xy[0] - self._last_xy[0], xy[1] - self._last_xy[1])
            self._last_xy = (float(xy[0]), float(xy[1]))
        if self._last_t is not None:
            self.state_time[self.state] = self.state_time.get(self.state, 0.0) + (now - self._last_t)
        self._last_t = now
        out = dict(route=None, task=None)

        if self.state in TERMINAL:
            out['route'] = []
            return self._out(out)

        # ---- safety gates, every tick
        pose_ok = xy is not None and pose_stamp is not None and now - pose_stamp <= self.p.pose_timeout
        if self.state != INITIALIZE and not pose_ok:
            return self._safe_stop(now, out, 'estimated pose missing or stale')
        if self.state in CARRYING and (self.manip is None or now - self.manip.stamp > self.p.manip_timeout):
            return self._safe_stop(now, out, 'manipulation status lost while handling an object')
        if self.manip is not None and self.manip.state == 'FAILED' and self.manip.reason.startswith('controller'):
            return self._safe_stop(now, out, f'manipulator controller failure: {self.manip.reason}')
        if health == 'CRITICAL':
            self.critical_since = self.critical_since if self.critical_since is not None else now
            if now - self.critical_since > self.p.critical_max:
                return self._safe_stop(now, out, f'localization CRITICAL for more than {self.p.critical_max:.0f} s')
            if self.state in MOVING:
                self.resume_state = self.state
                self.recoveries += 1
                self._go(now, LOCALIZATION_RECOVERY, 'localization health CRITICAL: follower holds/creeps, no new tasks')
        else:
            self.critical_since = None
            if self.state == LOCALIZATION_RECOVERY:
                self._go(now, self.resume_state or EXPLORE, 'localization recovered')

        # the robot's own traversals override blockages it has already driven through (Explorer.effective_status)
        self.explorer.observe(xy, trusted=health != 'CRITICAL')
        status = self.explorer.effective_status(status)
        handler = getattr(self, '_s_' + self.state.lower())
        handler(now, xy, cov, R, health, status, out)
        if out['route']:
            out['route'] = self._avoid_small_debris(out['route'])
        return self._out(out)

    def _avoid_small_debris(self, route):
        """Detour around known debris lower than the LiDAR plane (invisible to the blockage detector)."""
        skip = self._done_ids() | ({self.target['id']} if self.target is not None else set())
        if self.manip is not None and self.manip.state == 'HOLDING':
            skip.add(self.manip.object_id)
        obs = [(d.id, d.x, d.y, 0.5 * math.hypot(d.size[0], d.size[1]) + d.pos_sigma)
               for d in self.debris.values() if d.id not in skip and d.size[2] < self.p.lidar_plane]
        route, det, bad = avoid_debris(self.g, route, obs)
        self.detoured.update(det)
        self.unavoidable.update(bad)
        return route

    def _out(self, out):
        out.update(state=self.state, reason=self.reason,
                   target=None if self.target is None else dict(id=self.target['id'], approach=self.target['approach'],
                                                                score=round(self.target['score'], 3)))
        return out

    def _safe_stop(self, now, out, why):
        self._go(now, SAFE_STOP, why)
        out['route'] = []
        out['task'] = self._new_task(now, 'STOP', action='stop')
        return self._out(out)

    # ------------------------------------------------------------------ state handlers
    def _s_initialize(self, now, xy, cov, R, health, status, out):
        pose_ok = xy is not None
        manip_ok = self.manip is not None and self.manip.state == 'IDLE'
        if pose_ok and health != 'CRITICAL' and (manip_ok or not self.p.require_manipulator):
            self.manipulation_enabled = manip_ok
            self._go(now, EXPLORE, 'sensors, localization, manipulator and disposal station ready' if manip_ok else
                     'manipulator not required: inspection only')
            return self._s_explore(now, xy, cov, R, health, status, out)
        if self._elapsed(now) > self.p.init_timeout:
            if not pose_ok:
                self._safe_stop(now, out, 'no estimated pose after initialisation timeout')
                return
            if health != 'CRITICAL':
                self.manipulation_enabled = False
                self._go(now, EXPLORE, 'manipulator not ready: continuing inspection only (no pickups)')
                return self._s_explore(now, xy, cov, R, health, status, out)
        self.reason = ('waiting for: ' + ', '.join(w for w, ok in (('pose', pose_ok), ('localization', health != 'CRITICAL'),
                                                                     ('manipulator IDLE', manip_ok)) if not ok))

    def _s_explore(self, now, xy, cov, R, health, status, out):
        res = self.explorer.step(xy, cov, R, status, trusted=health != 'CRITICAL')
        self.explore_result = res
        if self.manipulation_enabled and self.debris and now - self.t_eval >= self.p.reprioritize_period:
            if self._prioritize(now, xy, cov, R, status):
                self._go(now, PRIORITIZE_TASKS, 'feasible debris found')
                return self._s_prioritize_tasks(now, xy, cov, R, health, status, out, fresh=True)
        if res['state'] == NO_ROUTE_HOME:
            # nothing left to explore, but the way back is blocked: wait (blockage evidence can clear), never succeed
            self.stranded_since = now if self.stranded_since is None else self.stranded_since
            if now - self.stranded_since > self.p.stranded_timeout:
                self._safe_stop(now, out, f'stranded: {res["reason"]} for {self.p.stranded_timeout:.0f} s')
                return
            self.reason = f'waiting for a route back to the entrance ({now - self.stranded_since:.0f} s)'
            out['route'] = []
            return
        self.stranded_since = None
        if res['state'] in (COMPLETE, INCOMPLETE):
            if self.manipulation_enabled and self.debris and self.t_eval < now and self._prioritize(now, xy, cov, R, status):
                self._go(now, PRIORITIZE_TASKS, 'feasible debris left after exploration')
                return self._s_prioritize_tasks(now, xy, cov, R, health, status, out, fresh=True)
            self._go(now, MISSION_COMPLETE, 'exploration ' + res['state'] + (f': {res["reason"]}' if res['reason'] else
                                                                            ', all selected tasks resolved'))
            out['route'] = []
            return
        out['route'] = res['waypoints'] if res['waypoints'] else None

    def _s_prioritize_tasks(self, now, xy, cov, R, health, status, out, fresh=False):
        ranked = self.last_eval['ranked'] if fresh else self._prioritize(now, xy, cov, R, status)
        if not ranked:
            self.target = None
            self._go(now, EXPLORE, 'no feasible debris')
            return self._s_explore(now, xy, cov, R, health, status, out)
        self.target, self.nav_failures = ranked[0], 0
        self._go(now, NAVIGATE_TO_DEBRIS, f"target {self.target['id']} score {self.target['score']:.2f}")
        return self._s_navigate_to_debris(now, xy, cov, R, health, status, out)

    def _s_navigate_to_debris(self, now, xy, cov, R, health, status, out):
        self.explorer.tracker.set_status(status)
        self.explorer.tracker.update_pose(xy[0], xy[1], 0.0, health != 'CRITICAL')     # inspect while driving
        t = self.target
        d = self.debris.get(t['id'])
        if d is None or now - d.stamp > self.prio.p.stale_after:
            self._go(now, PRIORITIZE_TASKS, f"target {t['id']} lost (stale detection)")
            return self._s_prioritize_tasks(now, xy, cov, R, health, status, out)
        goal = self.g.point(t['edge'], t['s'])
        if math.hypot(goal[0] - xy[0], goal[1] - xy[1]) < self.p.arrive_tol:
            self._go(now, PICKUP, f"at stand-off point of {t['id']}")
            return self._s_pickup(now, xy, cov, R, health, status, out, first=True)
        r = plan_to_point(self.model, self.g.locate(*xy), status, R, self.model.initial_covariance(cov),
                          t['edge'], t['s'])
        if r is None:
            self.nav_failures += 1
            self._go(now, NAVIGATION_FAILED, f"no route to {t['id']} ({self.nav_failures}/{self.p.max_nav_failures})")
            return self._s_navigation_failed(now, xy, cov, R, health, status, out)
        self.nav_failures = 0
        out['route'] = self._waypoints(xy, r['legs'], goal, status)

    def _s_navigation_failed(self, now, xy, cov, R, health, status, out):
        out['route'] = []                                          # stop safely while deciding
        if self.target is None:
            self._go(now, EXPLORE, 'navigation failed without a target')
            return
        if self.nav_failures >= self.p.max_nav_failures:
            self._go(now, UNREACHABLE_TARGET, f"{self.target['id']}: no route after {self.nav_failures} tries")
            return self._s_unreachable_target(now, xy, cov, R, health, status, out)
        self._go(now, NAVIGATE_TO_DEBRIS, 'retrying route')       # next tick plans again (alternate routes)

    def _s_unreachable_target(self, now, xy, cov, R, health, status, out):
        self.unreachable[self.target['id']] = self.reason
        self.target = None
        self._go(now, PRIORITIZE_TASKS, 'select the next feasible task')
        return self._s_prioritize_tasks(now, xy, cov, R, health, status, out)

    def _s_pickup(self, now, xy, cov, R, health, status, out, first=False):
        out['route'] = []                                          # stand still while the arm works
        t = self.target
        if first or self.task is None or self.task.type != 'PICKUP' or self.task.object_id != t['id']:
            d = self.debris[t['id']]
            out['task'] = self._new_task(now, 'PICKUP', t['id'], dict(x=d.x, y=d.y, yaw=t['approach']['yaw'],
                                                                      frame_id='map'), 'grasp_and_lift')
            self.t_state = now
            return
        m = self.manip
        if self._status_for_task(m) and m.state == 'HOLDING':
            self._go(now, VERIFY_PICKUP, 'manipulator reports HOLDING')
            return self._s_verify_pickup(now, xy, cov, R, health, status, out)
        if self._status_for_task(m) and m.state == 'FAILED':
            self._go(now, PICKUP_FAILED, m.reason or 'manipulator reports FAILED')
            return self._s_pickup_failed(now, xy, cov, R, health, status, out)
        if self._elapsed(now) > self.p.pickup_timeout:
            self._go(now, PICKUP_FAILED, 'pickup timed out')
            return self._s_pickup_failed(now, xy, cov, R, health, status, out)

    def _s_verify_pickup(self, now, xy, cov, R, health, status, out):
        out['route'] = []
        m = self.manip
        if self._status_for_task(m) and m.state == 'HOLDING' and m.verified and m.object_id == self.target['id']:
            self.events.append(dict(t=round(now, 2), event='picked_up', id=self.target['id']))
            self.dispose_attempts = 0
            self._go(now, RETURN_TO_DISPOSAL, f"{self.target['id']} held (verified)")
            return self._s_return_to_disposal(now, xy, cov, R, health, status, out)
        if not (self._status_for_task(m) and m.state == 'HOLDING'):
            self._go(now, PICKUP_FAILED, f'object not held ({m.state if m else "no status"})')
            return self._s_pickup_failed(now, xy, cov, R, health, status, out)
        if self._elapsed(now) > self.p.verify_timeout:
            self._go(now, PICKUP_FAILED, 'HOLDING reported but never verified')
            return self._s_pickup_failed(now, xy, cov, R, health, status, out)

    def _s_pickup_failed(self, now, xy, cov, R, health, status, out):
        out['route'] = []
        oid = self.target['id']
        self.attempts[oid] = self.attempts.get(oid, 0) + 1
        if self.attempts[oid] < self.p.max_pickup_attempts:
            self._go(now, PICKUP, f'retry {self.attempts[oid] + 1}/{self.p.max_pickup_attempts}')
            return self._s_pickup(now, xy, cov, R, health, status, out, first=True)
        self.failed[oid] = f'pickup failed {self.attempts[oid]} times: {self.reason}'
        self.target = None
        self._go(now, PRIORITIZE_TASKS, f'{oid} given up')
        return self._s_prioritize_tasks(now, xy, cov, R, health, status, out)

    def _s_return_to_disposal(self, now, xy, cov, R, health, status, out):
        self.explorer.tracker.set_status(status)
        self.explorer.tracker.update_pose(xy[0], xy[1], 0.0, health != 'CRITICAL')
        m, oid = self.manip, self.target['id']
        if not (m.state == 'HOLDING' and m.object_id == oid):
            self.failed[oid] = f'dropped while carrying ({m.state}: {m.reason})'
            self.target = None
            self._go(now, RESUME_EXPLORATION, f'{oid} no longer held')
            return self._s_resume_exploration(now, xy, cov, R, health, status, out)
        if math.hypot(self.station_xy[0] - xy[0], self.station_xy[1] - xy[1]) < self.p.arrive_tol:
            self._go(now, DISPOSE, 'at the disposal station')
            return self._s_dispose(now, xy, cov, R, health, status, out, first=True)
        # carrying: corridors narrower than robot/object + clearance count as blocked on the way back
        cstatus = carry_status(status, self.prio.carry_width(self.target['debris']))
        plans = self.model.plan(self.g.locate(*xy), cstatus, R, self.model.initial_covariance(cov), self.home)
        if not plans:
            # hold the object and wait: blockage evidence can clear (same rule as an explorer with no route home)
            self.stranded_since = now if self.stranded_since is None else self.stranded_since
            if now - self.stranded_since > self.p.stranded_timeout:
                self._safe_stop(now, out, f'carrying {oid} but no route wide enough back to the disposal station '
                                          f'for {self.p.stranded_timeout:.0f} s')
                return
            out['route'] = []
            self.reason = f'no route to the station, waiting ({now - self.stranded_since:.0f} s)'
            return
        self.stranded_since = None
        self.nav_failures = 0
        out['route'] = self._waypoints(xy, plans[0]['legs'], self.station_xy, status)
        self.carry_route = plans[0]['nodes']

    def _s_dispose(self, now, xy, cov, R, health, status, out, first=False):
        out['route'] = []
        oid = self.target['id']
        if first or self.task is None or self.task.type != 'DISPOSE':
            out['task'] = self._new_task(now, 'DISPOSE', oid, dict(x=float(self.station_xy[0]),
                                                                   y=float(self.station_xy[1]), frame_id='map'),
                                         'release_in_zone')
            self.t_state = now
            return
        ds = self.disposal
        if self._status_for_task(ds) and ds.verified:
            self._go(now, VERIFY_DISPOSAL, 'disposal reported verified')
            return self._s_verify_disposal(now, xy, cov, R, health, status, out)
        if self._status_for_task(ds) and ds.reason.startswith('failed'):
            self._go(now, DISPOSAL_FAILED, ds.reason)
            return self._s_disposal_failed(now, xy, cov, R, health, status, out)
        if self._elapsed(now) > self.p.dispose_timeout:
            self._go(now, DISPOSAL_FAILED, 'disposal timed out')
            return self._s_disposal_failed(now, xy, cov, R, health, status, out)

    def _s_verify_disposal(self, now, xy, cov, R, health, status, out):
        out['route'] = []
        ds, m, oid = self.disposal, self.manip, self.target['id']
        if (self._status_for_task(ds) and ds.verified and ds.released and ds.object_id == oid
                and not (m.state == 'HOLDING' and m.object_id == oid)):
            self.removed[oid] = dict(t=round(now, 2), debris=self.debris[oid])
            self.events.append(dict(t=round(now, 2), event='disposed', id=oid))
            self.target = None
            self._go(now, RESUME_EXPLORATION, f'{oid} disposal verified')
            return self._s_resume_exploration(now, xy, cov, R, health, status, out)
        if self._elapsed(now) > self.p.verify_timeout:
            self._go(now, DISPOSAL_FAILED, 'disposal not verified (object still held or outside the zone)')
            return self._s_disposal_failed(now, xy, cov, R, health, status, out)

    def _s_disposal_failed(self, now, xy, cov, R, health, status, out):
        out['route'] = []
        oid = self.target['id']
        self.dispose_attempts += 1
        still_held = self.manip is not None and self.manip.state == 'HOLDING' and self.manip.object_id == oid
        # retrying only makes sense while the object is still in the gripper: once it has been let go outside the
        # verified zone another DISPOSE cannot change anything (seen in the end-to-end runs with Account 2's arm)
        if self.dispose_attempts < self.p.max_dispose_attempts and still_held:
            self._go(now, DISPOSE, f'retry {self.dispose_attempts + 1}/{self.p.max_dispose_attempts}')
            return self._s_dispose(now, xy, cov, R, health, status, out, first=True)
        self.failed[oid] = f'disposal failed {self.dispose_attempts} times: {self.reason}'
        if self.manip is not None and self.manip.state == 'HOLDING':
            self._safe_stop(now, out, f'{oid} still held after failed disposal')
            return
        self.target = None
        self._go(now, RESUME_EXPLORATION, f'{oid} disposal unresolved (reported)')
        return self._s_resume_exploration(now, xy, cov, R, health, status, out)

    def _s_resume_exploration(self, now, xy, cov, R, health, status, out):
        self._go(now, EXPLORE, 'resume the unfinished inspection')
        self.t_eval = -1e9
        self._s_explore(now, xy, cov, R, health, status, out)
        res = self.explore_result or {}
        self.resumptions.append(dict(t=round(now, 2), next=res.get('target'), explore_state=res.get('state'),
                                     remaining_m=self.explorer.tracker.summary()['remaining_m']))

    def _s_localization_recovery(self, now, xy, cov, R, health, status, out):
        out['route'] = None          # keep the route: the follower's own health response stops, then creeps

    def _waypoints(self, xy, legs, goal, status):
        pts = self.model.waypoints(xy, dict(legs=legs), status)
        if not legs:
            pts.append(np.asarray(goal, float))
        return pts

    # ------------------------------------------------------------------ report
    def report(self, now):
        cov = self.explorer.tracker.summary()
        ev = self.last_eval
        rej = {k: v for k, v in self.rejections.items() if k not in self.removed and k not in self.failed}
        fmt = lambda d: dict(id=d.id, xy=[round(d.x, 3), round(d.y, 3)], size=[round(v, 3) for v in d.size],  # noqa: E731
                             confidence=round(d.confidence, 3), category=d.category)
        return dict(
            state=self.state, reason=self.reason, duration=round(now - (self.t0 or now), 1),
            distance_m=round(self.distance, 2), manipulation_enabled=self.manipulation_enabled,
            coverage=dict((k, cov[k]) for k in ('coverage', 'confidence', 'explored_m', 'accessible_m', 'blocked_m',
                                                'unreachable_m', 'skipped_m', 'total_known_m', 'complete', 'segments',
                                                'edges')),
            debris=dict(detected=[fmt(d) for d in self.debris.values()],
                        removed=[dict(id=k, t=v['t']) for k, v in self.removed.items()],
                        failed=[dict(id=k, reason=v) for k, v in self.failed.items()],
                        unreachable=[dict(id=k, reason=v) for k, v in self.unreachable.items()],
                        rejected=[dict(id=k, reason=v['reason'], detail=v['detail']) for k, v in rej.items()
                                  if k not in self.removed],
                        last_ranking=[dict(id=r['id'], score=round(r['score'], 3), terms=r['terms'])
                                      for r in ev['ranked']]),
            defects=[dict(id=d.id, type=d.type, xy=[round(d.x, 3), round(d.y, 3)], severity=round(d.severity, 3),
                          confidence=round(d.confidence, 3), evidence=d.evidence)
                     for d in sorted(self.defects.values(), key=lambda d: -d.severity)],
            counts=dict(detected=len(self.debris), removed=len(self.removed), failed=len(self.failed),
                        unreachable=len(self.unreachable), rejected=len(rej), defects=len(self.defects),
                        pickup_requests=self.pickup_requests,
                        localization_recoveries=self.recoveries, detection_updates_ignored=self.detections_ignored),
            state_time={k: round(v, 1) for k, v in self.state_time.items()},
            holding=(self.manip.object_id if self.manip is not None and self.manip.state == 'HOLDING' else None),
            resumptions=self.resumptions,
            small_debris=dict(detoured=sorted(self.detoured), unavoidable=sorted(self.unavoidable - set(self.removed))),
            station=dict(xy=[round(float(v), 3) for v in self.station_xy], node=self.home),
            events=self.events, input_errors=self.input_errors[-20:])
