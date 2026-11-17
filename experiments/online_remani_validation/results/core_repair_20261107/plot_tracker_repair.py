#!/usr/bin/python3
"""Plot recorded observations; does not run or configure any controller."""
import csv
import json
import os
from pathlib import Path

os.environ.setdefault('MPLCONFIGDIR', '/tmp/remani_repair_mpl')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent
fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
for folder, label, color in [
        ('blind_enter_mppi_s11', 'Before observation filter fix', '#c44e52'),
        ('blind_enter_tracker_fixed_s11', 'After observation filter fix', '#4c72b0')]:
    run = ROOT / folder
    with (run / 'sensor_visibility.csv').open() as stream:
        rows = [r for r in csv.DictReader(stream) if r['t'] and r['actor_present'] == 'True']
    axes[0].plot([float(r['t']) for r in rows],
                 [int(r['actor_visible_points']) for r in rows], color=color, label=label)
    records = [json.loads(line) for line in (run / 'snapshot_headers.jsonl').read_text().splitlines()]
    records = [r for r in records if r['elapsed_s'] is not None]
    times = [r['elapsed_s'] for r in records]
    axes[1].step(times, [r['header'][13] for r in records], where='post', color=color)
    axes[2].step(times, [r['header'][19] for r in records], where='post', color=color)
axes[0].set_ylabel('Raw visible actor points')
axes[1].set_ylabel('Exported tracked points')
axes[2].set_ylabel('Prediction valid flag')
axes[2].set_yticks([0, 1])
axes[2].set_xlabel('Time after navigation goal (s)')
axes[2].set_xlim(0, 8)
axes[0].legend(loc='upper right')
for axis in axes:
    axis.grid(alpha=.25)
fig.suptitle('Same simulated LiDAR scene, seed 11: observed target retention')
fig.text(.5, .01, 'Validity is not a visibility/clear-space certificate. Both trials still collided later.',
         ha='center', fontsize=9)
fig.tight_layout(rect=(0, .035, 1, .96))
fig.savefig(ROOT / 'tracker_repair_comparison.png', dpi=150)
