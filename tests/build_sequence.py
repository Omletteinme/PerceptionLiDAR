#!/usr/bin/env python3
"""Resample recorded camera frames to a constant frame rate by SIM time, as symlinks ffmpeg can read.

Each camera delivers frames at its own (lower, uneven) rate because software rendering runs slower than the
simulation clock. For every output tick the latest frame at or before that sim time is held, so all clips play
in real time and stay in sync with each other.

Usage: build_sequence.py <frames_dir> <out_dir> <fps> <skip_seconds> <cam> [<cam> ...]
"""
import csv
import os
import sys

frames_dir, out_dir, fps, skip = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
cams = sys.argv[5:]
times = {c: [(int(i), float(t)) for i, t in csv.reader(open(f'{frames_dir}/{c}/times.csv'))] for c in cams}
t_start = max(rows[0][1] for rows in times.values()) + skip
t_end = min(rows[-1][1] for rows in times.values())
n = int((t_end - t_start) * fps)
for c, rows in times.items():
    os.makedirs(f'{out_dir}/{c}', exist_ok=True)
    j = 0
    for k in range(n):
        T = t_start + k / fps
        while j + 1 < len(rows) and rows[j + 1][1] <= T:
            j += 1
        os.symlink(f'{frames_dir}/{c}/{rows[j][0]:05d}.jpg', f'{out_dir}/{c}/{k:05d}.jpg')
    gaps = [b[1] - a[1] for a, b in zip(rows, rows[1:])]
    print(f'{c}: {len(rows)} source frames, mean {1 / (sum(gaps) / len(gaps)):.1f} Hz of sim time, max gap {max(gaps):.2f} s')
print(f'{n} output frames at {fps:g} fps = {n / fps:.1f} s of sim time (sim t = {t_start:.1f} .. {t_end:.1f})')
