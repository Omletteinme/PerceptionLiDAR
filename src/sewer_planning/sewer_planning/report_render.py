"""Final mission report: Markdown summary and a map figure from MissionManager.report().

CUSTOM code (ours), no ROS imports. Used by mission_manager_node (written to disk when the mission ends) and by the
sandbox (sandbox/mission_sim.py --report-dir). Everything shown comes from the report, i.e. from what the robot
believed and what Account 2 verified; ground truth is not used here.
"""
import json
import math
import os

EDGE_COLOURS = {'complete': '#2e9e44', 'partial': '#e0a800', 'unexplored': '#9e9e9e', 'blocked': '#d62728',
                'unreachable': '#5a5a5a'}
DEBRIS_STYLE = {'removed': ('#2e9e44', 'o', 'removed (disposal verified)'),
                'failed': ('#ff7f0e', 'X', 'pickup/disposal failed'),
                'unreachable': ('#8c564b', 's', 'unreachable'),
                'rejected': ('#d62728', 'v', 'rejected (infeasible)'),
                'other': ('#1f77b4', 'D', 'detected, not handled')}


def debris_outcomes(rep):
    """id -> (outcome, detail) for every detected object."""
    d = rep['debris']
    out = {o['id']: ('other', '') for o in d['detected']}
    for k in ('rejected', 'unreachable', 'failed'):
        for o in d[k]:
            out[o['id']] = (k, o.get('reason', '') + (f": {o['detail']}" if o.get('detail') else ''))
    for o in d['removed']:
        out[o['id']] = ('removed', f"t = {o['t']} s")
    return out


def render_markdown(rep, title='Sewer mission report'):
    c, n = rep['coverage'], rep['counts']
    seg = c['segments']
    out = debris_outcomes(rep)
    det = {o['id']: o for o in rep['debris']['detected']}
    lines = [f'# {title}', '',
             f"**Result: {rep['state']}** ({rep['reason']})", '',
             f"- Duration {rep['duration']} s, distance driven {rep['distance_m']} m (estimated pose)",
             f"- Manipulation {'enabled' if rep['manipulation_enabled'] else 'DISABLED (inspection only)'}; "
             f"localization recoveries {n['localization_recoveries']}",
             '', '## Coverage', '',
             f"- **{100 * c['coverage']:.1f} %** of the accessible network inspected "
             f"({c['explored_m']} of {c['accessible_m']} m; confidence {100 * c['confidence']:.0f} %)",
             f"- Known network {c['total_known_m']} m: blocked {c['blocked_m']} m, unreachable {c['unreachable_m']} m, "
             f"skipped {c['skipped_m']} m",
             f"- Corridors: {seg['complete']} complete, {seg['partial']} partial, {seg['unexplored']} unexplored, "
             f"{seg['unreachable']} unreachable, {seg.get('obstructed', 0)} with a blockage",
             '', '## Debris', '',
             f"{n['detected']} detected, **{n['removed']} removed** (disposal verified), {n['failed']} failed, "
             f"{n['unreachable']} unreachable, {n['rejected']} rejected; {n['pickup_requests']} pickup requests.", '',
             '| id | position [m] | size l x w x h [m] | category | outcome | detail |', '|---|---|---|---|---|---|']
    for oid in sorted(out):
        o = det.get(oid, {})
        xy = o.get('xy', ['?', '?'])
        size = ' x '.join(f'{v:.2f}' for v in o.get('size', []))
        lines.append(f"| {oid} | ({xy[0]}, {xy[1]}) | {size} | {o.get('category', '')} | {out[oid][0]} | "
                     f"{out[oid][1]} |")
    if rep['debris'].get('last_ranking'):
        lines += ['', 'Last priority ranking (0.4 size + 0.3 obstruction + 0.2 accessibility + 0.1 graspability):', '']
        lines += [f"- {r['id']}: {r['score']} {r['terms']}" for r in rep['debris']['last_ranking']]
    sd = rep.get('small_debris', {})
    if sd.get('detoured') or sd.get('unavoidable'):
        lines += ['', f"Low debris (below the LiDAR) routed around: {', '.join(sd.get('detoured', [])) or '-'}; "
                      f"could not be avoided: {', '.join(sd.get('unavoidable', [])) or '-'}"]
    lines += ['', '## Structural defects (reported, not repaired)', '']
    if rep['defects']:
        lines += ['| id | type | position [m] | severity | confidence | evidence |', '|---|---|---|---|---|---|']
        lines += [f"| {d['id']} | {d['type']} | ({d['xy'][0]}, {d['xy'][1]}) | {d['severity']:.2f} | "
                  f"{d['confidence']:.2f} | {d['evidence'] or '-'} |" for d in rep['defects']]
    else:
        lines.append('None reported.')
    lines += ['', '## Timeline', '']
    for e in rep['events']:
        lines.append(f"- {e['t']} s: " + (f"{e['frm']} -> {e['to']} ({e['reason']})" if 'to' in e else
                                         f"{e['event']} {e.get('id', '')}"))
    if rep.get('holding'):
        lines += ['', f"**Still holding {rep['holding']} at the end.**"]
    if rep.get('input_errors'):
        lines += ['', '## Dropped input messages', ''] + [f'- {e}' for e in rep['input_errors']]
    lines += ['', '## Time per state [s]', '', ', '.join(f'{k} {v}' for k, v in sorted(rep['state_time'].items(),
                                                                                       key=lambda kv: -kv[1]))]
    return '\n'.join(lines) + '\n'


def render_map(rep, graph, edge_states, out_png, omap=None, path_xy=None, title=None):
    """Map figure. edge_states: {'A-B': state} (CoverageTracker.summary()['edges'][k]['state'])."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 7.5))
    if omap is not None:
        ext = (omap.x0, omap.x0 + omap.w * omap.res, omap.y0, omap.y0 + omap.h * omap.res)
        ax.imshow(omap.solid, origin='lower', extent=ext, cmap='Greys', alpha=0.35, interpolation='nearest')
    for e in graph.edges:
        st = edge_states.get(graph.key(e), 'unexplored')
        (x0, y0), (x1, y1) = graph.nodes[e[0]], graph.nodes[e[1]]
        ax.plot([x0, x1], [y0, y1], color=EDGE_COLOURS.get(st, '#9e9e9e'), lw=6, alpha=0.55, solid_capstyle='round')
    for k, (x, y) in graph.nodes.items():
        ax.annotate(k, (x, y), xytext=(4, 4), textcoords='offset points', fontsize=9)
    if path_xy is not None and len(path_xy):
        ax.plot([p[0] for p in path_xy], [p[1] for p in path_xy], color='#1f3b73', lw=0.8, label='robot path (est.)')
    out = debris_outcomes(rep)
    det = {o['id']: o for o in rep['debris']['detected']}
    seen = set()
    for oid, (kind, _) in out.items():
        if oid not in det:
            continue
        col, mk, lab = DEBRIS_STYLE[kind]
        x, y = det[oid]['xy']
        ax.scatter([x], [y], c=col, marker=mk, s=70, edgecolors='k', linewidths=0.5, zorder=5,
                   label=None if kind in seen else lab)
        ax.annotate(oid, (x, y), xytext=(4, -10), textcoords='offset points', fontsize=7, color=col)
        seen.add(kind)
    for i, d in enumerate(rep['defects']):
        ax.scatter([d['xy'][0]], [d['xy'][1]], marker='^', s=40 + 160 * d['severity'], c='#9467bd', edgecolors='k',
                   linewidths=0.5, zorder=6, label='defect (size = severity)' if i == 0 else None)
        ax.annotate(f"{d['type']}", d['xy'], xytext=(5, 5), textcoords='offset points', fontsize=7, color='#6a3d9a')
    st = rep.get('station', {}).get('xy')
    if st:
        ax.scatter([st[0]], [st[1]], marker='*', s=220, c='gold', edgecolors='k', zorder=7, label='disposal station')
    for s_, col in EDGE_COLOURS.items():
        ax.plot([], [], color=col, lw=6, alpha=0.55, label=f'corridor {s_}')
    c = rep['coverage']
    ax.set_title(title or f"{rep['state']}: coverage {100 * c['coverage']:.0f} %, removed {rep['counts']['removed']}, "
                          f"defects {rep['counts']['defects']}")
    ax.set_aspect('equal')
    ax.legend(fontsize=7, loc='upper left', framealpha=0.85)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    return out_png


def write_report(rep, out_dir, graph=None, edge_states=None, omap=None, path_xy=None):
    """report.json + report.md (+ map.png when the graph is given). Returns the list of files written."""
    os.makedirs(out_dir, exist_ok=True)
    files = [os.path.join(out_dir, 'report.json'), os.path.join(out_dir, 'report.md')]
    with open(files[0], 'w') as f:
        json.dump(rep, f, indent=1, default=str)
    with open(files[1], 'w', encoding='utf-8') as f:
        f.write(render_markdown(rep))
    if graph is not None:
        try:
            files.append(render_map(rep, graph, edge_states or {}, os.path.join(out_dir, 'map.png'), omap, path_xy))
        except ImportError:                       # matplotlib missing: the text report is still complete
            pass
    return files


def decimate(path, step=0.25):
    """Keep path points at least `step` metres apart (for storing the driven path)."""
    out = []
    for p in path:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= step:
            out.append((float(p[0]), float(p[1])))
    return out
