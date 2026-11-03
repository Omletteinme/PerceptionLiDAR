#!/usr/bin/env python3
"""M6, step 3: assemble the final demo video from the recorded pieces.

Layout (1920x1080, 10 fps, real time):
  top left   View A  Gazebo 3D oblique overview          top right  View B  RViz (replayed bag)
  middle     chase camera | robot camera | error-vs-time panel (with the localization-health band and a time cursor)
  bottom     caption bar: what is happening now (from the mission events), plus live numbers

Everything is derived from files that demo_record.sh and demo_viewb.sh wrote: traj.csv, traj_summary.json, sync.txt and
clips/*.mp4. The true pose in the CSV is used only to draw the error curves (evaluation display). Usage: demo_compose.py <demo_dir>
"""
import json
import os
import re
import subprocess
import sys

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

FPS = 10
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
STATE_COLOR = ['#2e9e4f', '#e0b020', '#d63a2f']
STATE_NAME = ['RELIABLE', 'DEGRADED', 'CRITICAL']
D = sys.argv[1]

t0, t1 = map(float, re.search(r'sim t = ([\d.]+) \.\. ([\d.]+)', open(f'{D}/sync.txt').read()).groups())
summary = json.load(open(f'{D}/traj_summary.json'))
events = json.load(open(f'{D}/traj_events.json'))
df = pd.read_csv(f'{D}/traj.csv')
offset = t0 - summary['t_move']                           # mission time = video time + offset
n_frames = int((t1 - t0) * FPS)
t_mis = df['t'].to_numpy()


def err(cols):
    d = df[cols].to_numpy() - df[['gt_x', 'gt_y']].to_numpy()
    return np.hypot(d[:, 0], d[:, 1])


series = {'dead reckoning': (err(['dr_x', 'dr_y']), 'tab:gray'), 'baseline PF': (err(['bl_x', 'bl_y']), 'tab:blue'),
          'reliability-aware PF': (err(['est_x', 'est_y']), 'tab:red')}
health, R = df['health'].to_numpy(), df['R'].to_numpy()

# ---- captions from the mission events (mission time, text)
caps = [(-1e9, 'The robot enters the sewer network')]
for edge, det in summary['blocked_first'].items():
    caps.append((det[0], f'LiDAR sees debris blocking the route: edge {edge} marked blocked'))
prev = None
for t, nodes, _, _ in events['route_events']:
    if nodes:
        if prev is not None and nodes != prev[-len(nodes):]:          # not just the old route getting shorter: a replan
            caps.append((t, 'Safer route planned around the blockage: ' + ' > '.join(nodes)))
        prev = nodes
last_state = 0
for t, state in summary['health_log']:
    name = {'DEGRADED': 'Readings becoming unreliable (water / spray / slip): robot slows down',
            'CRITICAL': 'Readings very unreliable: robot waits, the filter holds the estimate',
            'RELIABLE': 'Readings reliable again: robot continues at full speed'}[state]
    caps.append((t, name))
caps.append((summary['mission_time'] - 1.0, f"Goal {summary.get('route_nodes', ['J4'])[-1]} reached" if summary['goal_reached'] else 'Mission ended without reaching the goal'))
for t_sim, model in summary['collisions']['events']:
    caps.append((t_sim - summary['t_move'], f'Contact with {model}'))
caps.sort()
hits = np.array([e[0] - summary['t_move'] for e in summary['collisions']['events']])


def caption_at(t):
    return [c for c in caps if c[0] <= t][-1][1]


def piped(w, h, draw, out):
    """Render `draw(k)` for every frame into an H.264 clip via an ffmpeg rawvideo pipe."""
    p = subprocess.Popen(['ffmpeg', '-nostdin', '-y', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgba',
                          '-s', f'{w}x{h}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '18',
                          out], stdin=subprocess.PIPE)
    for k in range(n_frames):
        p.stdin.write(draw(k))
    p.stdin.close()
    p.wait()


# ---- error panel (960x270)
fig = plt.figure(figsize=(9.6, 2.7), dpi=100)
ax = fig.add_axes([0.07, 0.30, 0.91, 0.62])
band = fig.add_axes([0.07, 0.17, 0.91, 0.05])
x0, x1 = offset, offset + (t1 - t0)
for name, (e, c) in series.items():
    ax.plot(t_mis, np.maximum(e, 0.005), color=c, lw=1.6 if 'reliab' in name else 1.1, label=name)
ax.axhline(0.5, color='k', ls='--', lw=0.7)
ax.set_xlim(x0, x1)
ax.set_yscale('log')
ax.set_ylim(0.005, 3.0)
ax.set_ylabel('position error [m]', fontsize=8)
ax.tick_params(labelsize=7, labelbottom=False)
ax.legend(loc='upper left', fontsize=7, ncol=3, frameon=False)
ax.set_title('Estimated vs true position (true pose used only to draw this curve); dashed = 0.5 m failure level',
             fontsize=8, loc='left')
cursor = ax.axvline(x0, color='k', lw=1.2)
states = np.interp(np.linspace(x0, x1, 400), t_mis, health).round().astype(int)
band.imshow(np.array([[matplotlib.colors.to_rgb(STATE_COLOR[h]) for h in states]]), aspect='auto', extent=[x0, x1, 0, 1])
band.set_yticks([])
band.tick_params(labelsize=7)
band.set_xlabel('mission time [s]   (band: localization health  green RELIABLE / yellow DEGRADED / red CRITICAL)', fontsize=7)
bcur = band.axvline(x0, color='k', lw=1.5)


def draw_plot(k):
    t = offset + k / FPS
    cursor.set_xdata([t, t])
    bcur.set_xdata([t, t])
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba()).tobytes()


piped(960, 270, draw_plot, f'{D}/clips/panel_plot.mp4')
plt.close(fig)

# ---- caption bar (1920x270)
fig = plt.figure(figsize=(19.2, 2.7), dpi=100, facecolor='#14161c')
fig.text(0.012, 0.88, 'View A: Gazebo 3D simulation        View B: RViz (green = true pose, red = reliability-aware estimate, '
         'blue = baseline estimate, yellow = planned route)', color='#9aa3b2', fontsize=13, va='center')
cap = fig.text(0.012, 0.52, '', color='white', fontsize=27, va='center', fontweight='bold')
num = fig.text(0.012, 0.15, '', color='#9aa3b2', fontsize=13, va='center', family='monospace')


def draw_caption(k):
    t = offset + k / FPS
    i = int(np.clip(np.searchsorted(t_mis, t), 0, len(t_mis) - 1))
    cap.set_text(caption_at(t))
    dr = series['dead reckoning'][0][i]
    num.set_text(f't = {t:5.1f} s   error: reliability-aware {series["reliability-aware PF"][0][i]:.2f} m, baseline '
                 f'{series["baseline PF"][0][i]:.2f} m, dead reckoning {"n/a" if np.isnan(dr) else f"{dr:.2f} m"}   '
                 f'scan reliability R = {R[i]:.2f}   health: {STATE_NAME[int(health[i])]}   '
                 f'collisions: {int((hits <= t).sum())}')
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba()).tobytes()


piped(1920, 270, draw_caption, f'{D}/clips/panel_caption.mp4')
plt.close(fig)

# ---- final assembly
C = f'{D}/clips'
lbl = lambda s: (f"drawtext=fontfile={FONT}:text='{s}':x=12:y=10:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=6")  # noqa: E731
filt = (f"[0]scale=960:540,{lbl('View A: Gazebo')}[a];[1]scale=-1:540,pad=960:540:(ow-iw)/2:0:0x14161c,{lbl('View B: RViz')}[b];"
        "[2]scale=480:270[c];[3]scale=480:270[d];[4]scale=960:270[e];[5]scale=1920:270[f];"
        f'color=c=0x14161c:s=1920x1080:r={FPS}[base];[base][a]overlay=0:0[v1];[v1][b]overlay=960:0[v2];'
        '[v2][c]overlay=0:540[v3];[v3][d]overlay=480:540[v4];[v4][e]overlay=960:540[v5];[v5][f]overlay=0:810:shortest=1')
out = f'{D}/demo_final.mp4'
subprocess.run(['ffmpeg', '-nostdin', '-y', '-loglevel', 'error'] + sum(
    [['-i', f'{C}/{n}.mp4'] for n in ('oblique', 'rviz', 'chase', 'robot', 'panel_plot', 'panel_caption')], [])
    + ['-filter_complex', filt, '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '20', '-preset', 'medium', out], check=True)
print('wrote', out)
