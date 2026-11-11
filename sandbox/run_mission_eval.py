"""Multi-seed evaluation of the full sandbox mission (Account 1). EVERY run is kept, failures included.

CUSTOM code (ours). Runs sandbox/mission_sim.py over seeds and conditions (scan-disturbance severity + wheel slip) and
scores each run against the scenario truth, which only this evaluator reads:
  completed      final state MISSION_COMPLETE (SAFE_STOP and timeouts are kept as failures, with their reason)
  coverage       tracker coverage of the accessible network (robot's own claim)
  removed_ok     objects the report says were removed AND the simulator shows disposed
  false_success  objects reported removed that the simulator does NOT show disposed (must be 0)
  feasible_left  objects that fit the gripper but were neither removed nor reported as failed/rejected
  defects        defects reported (4 exist in the scenario)
  scrape         the TRUE robot came closer than its half-width to a wall
Perception and the arm are the sandbox STAND-INS (mission_standins.py), so this measures the Mission Manager, explorer,
localization and navigation, not Account 2's algorithms.

  python3 sandbox/run_mission_eval.py --seeds 5 --conditions clean:0:0,disturbed:2:2 --procs 4
  -> logs/eval/mission/{runs.csv, summary.md}
"""
import argparse
import csv
import os
import sys
import time
from multiprocessing import Pool

sys.path.insert(0, os.path.dirname(__file__))

FIELDS = ['condition', 'seed', 'severity', 'slip', 'state', 'reason', 'sim_time', 'wall_time', 'coverage',
          'removed', 'removed_ok', 'false_success', 'failed', 'rejected', 'feasible_left', 'defects',
          'pickup_requests', 'recoveries', 'debris_contacts', 'min_clearance', 'scrape', 'pos_rmse', 'max_pos_err', 'critical_s',
          'degraded_s']


def one(args):
    import explore_sim
    import mission_sim
    cond, seed, sev, slip, t_max = args
    t0 = time.time()
    try:
        out = mission_sim.run(seed=seed, severity=sev, slip=slip, t_max=t_max)
    except Exception as e:                                # a crash is a result too: keep it
        return dict(condition=cond, seed=seed, severity=sev, slip=slip, state='CRASH', reason=repr(e)[:200],
                    wall_time=round(time.time() - t0, 1))
    rep, sc = out['report'], out['scenario']
    removed = {r['id'] for r in rep['debris']['removed']}
    disposed = {d.id for d in sc.debris if d.disposed}
    feasible = {d.id for d in sc.debris if sorted(d.size[:2])[0] <= 0.11 and d.size[2] <= 0.15}
    accounted = removed | {r['id'] for r in rep['debris']['failed']} | {r['id'] for r in rep['debris']['rejected']} | \
        {r['id'] for r in rep['debris']['unreachable']}
    return dict(condition=cond, seed=seed, severity=sev, slip=slip, state=rep['state'], reason=rep['reason'][:200],
                sim_time=round(out['sim_time'], 1), wall_time=round(time.time() - t0, 1),
                coverage=rep['coverage']['coverage'], removed=' '.join(sorted(removed)),
                removed_ok=len(removed & disposed), false_success=len(removed - disposed),
                failed=len(rep['debris']['failed']), rejected=len(rep['debris']['rejected']),
                feasible_left=len(feasible - accounted), defects=len(rep['defects']),
                pickup_requests=rep['counts']['pickup_requests'], recoveries=rep['counts']['localization_recoveries'],
                debris_contacts=len(out['debris_contacts']), min_clearance=round(out['min_clearance'], 3),
                scrape=int(out['min_clearance'] < explore_sim.ROBOT_HALF_W),
                pos_rmse=round(out['pos_rmse'], 3), max_pos_err=round(out['max_pos_err'], 3),
                critical_s=out['health_time'].get('CRITICAL', 0.0), degraded_s=out['health_time'].get('DEGRADED', 0.0))


def summarise(rows):
    lines = ['| condition | runs | completed | mean coverage | removed / feasible (3 per run) | false successes | '
             'scrapes | mean pose RMSE [m] | mean CRITICAL [s] | non-complete runs (seed: state, reason) |',
             '|---|---|---|---|---|---|---|---|---|---|']
    for cond in dict.fromkeys(r['condition'] for r in rows):
        rs = [r for r in rows if r['condition'] == cond]
        ok = [r for r in rs if r['state'] == 'MISSION_COMPLETE']
        num = [r for r in rs if r['state'] != 'CRASH']
        mean = lambda k: sum(float(r[k]) for r in num) / max(len(num), 1)  # noqa: E731
        bad = '; '.join(f"{r['seed']}: {r['state']}, {r['reason'][:60]}" for r in rs if r['state'] != 'MISSION_COMPLETE')
        lines.append(f"| {cond} | {len(rs)} | {len(ok)}/{len(rs)} | {mean('coverage'):.3f} | "
                     f"{sum(int(r.get('removed_ok', 0)) for r in num)}/{3 * len(rs)} | "
                     f"{sum(int(r.get('false_success', 0)) for r in num)} | {sum(int(r.get('scrape', 0)) for r in num)} | "
                     f"{mean('pos_rmse'):.3f} | {mean('critical_s'):.1f} | {bad or '-'} |")
    return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seeds', type=int, default=5)
    ap.add_argument('--first-seed', type=int, default=0)
    ap.add_argument('--conditions', default='clean:0:0,disturbed:2:2', help='name:severity:slip,...')
    ap.add_argument('--procs', type=int, default=4)
    ap.add_argument('--t-max', type=float, default=1800.0)
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'logs', 'eval', 'mission'))
    a = ap.parse_args()
    jobs = []
    for c in a.conditions.split(','):
        name, sev, slip = c.split(':')
        jobs += [(name, s, float(sev), float(slip), a.t_max) for s in range(a.first_seed, a.first_seed + a.seeds)]
    os.makedirs(a.out, exist_ok=True)
    with Pool(a.procs) as pool:
        rows = []
        for r in pool.imap_unordered(one, jobs):
            rows.append(r)
            print(f"[{len(rows)}/{len(jobs)}] {r['condition']} seed {r['seed']}: {r['state']} ({r.get('reason', '')[:70]})",
                  flush=True)
    rows.sort(key=lambda r: (r['condition'], r['seed']))
    with open(os.path.join(a.out, 'runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, FIELDS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, '') for k in FIELDS})
    md = summarise(rows)
    with open(os.path.join(a.out, 'summary.md'), 'w') as f:
        f.write(md + '\n')
    print(md)


if __name__ == '__main__':
    main()
