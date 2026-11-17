#!/usr/bin/python3
"""Replot saved numerical trials; no ROS, simulation, or controller execution."""
import json
import os
import sys
from pathlib import Path

for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
os.environ.setdefault('MPLCONFIGDIR', '/tmp/remani-runtime-report-mpl')
ROOT = Path(__file__).resolve().parents[5]
BASE = Path(__file__).resolve().parents[1]
OUT = BASE / 'report'
PREVIOUS = BASE.parent / 'standby_candidate_followup_20261106'
sys.path.insert(0, str(ROOT / 'src/mm_remani/scripts'))
from plot_standby_sim import Run, plot_metrics, plot_animation
import numpy as np
import matplotlib.pyplot as plt


def comparison(paths, labels, directory, animate=False):
    directory.mkdir(parents=True, exist_ok=True)
    runs = [Run(path, i) for i, path in enumerate(paths)]
    for run, label in zip(runs, labels):
        run.name = label
    plot_metrics(runs, directory)
    if animate:
        plot_animation(runs, directory, 5, 43)


comparison(
    [PREVIOUS / 'corrected_servo_zero', BASE / 'fast_zero_s11', BASE / 'fast_zero_s23'],
    ['Before speedup, seed 11', 'After speedup, seed 11', 'After speedup, seed 23'],
    OUT / 'diagnostic_zero_uncertainty', True)
comparison(
    [PREVIOUS / 'corrected_servo_current', BASE / 'fast_current_s11'],
    ['Before speedup, seed 11', 'After speedup, seed 11'],
    OUT / 'current_profile')

witness_dir = BASE / 'arm_response_witness'
witness = json.loads((witness_dir / 'result.json').read_text())
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
colors = ['#208653', '#dc8616', '#2878b5']
for key, label, color in zip(
        ['first_dynamic_received', 'first_posture_delivery'],
        ['Prescribed retract from 2.365 s', 'Prescribed retract from 7.040 s'], colors):
    case = witness['cases'][key]
    data = np.genfromtxt(witness_dir / case['trajectory_csv'], delimiter=',', names=True)
    joints = np.column_stack([data['q%d' % j] for j in range(1, 7)])
    values = [1000 * data['external_gap_m'],
              np.max(np.abs(np.rad2deg(joints - joints[0])), axis=1),
              1000 * np.hypot(data['x'] - data['x'][0], data['y'] - data['y'][0])]
    for ax, value in zip(axes, values):
        ax.plot(data['t'], value, color=color, label=label, linewidth=1.7)
        ax.axvline(case['first_command_elapsed_s'], color=color, alpha=.35, linestyle=':')
        if not case['completed']:
            ax.plot(data['t'][-1], value[-1], 'x', color=color, markersize=8)

run = Run(BASE / 'fast_zero_s11', 2)
values = [1000 * run.state['external_gap_m'],
          np.max(np.abs(np.rad2deg(run.q - run.q[0])), axis=1),
          1000 * np.linalg.norm(run.base[:, :2] - run.base[0, :2], axis=1)]
for ax, value in zip(axes, values):
    ax.plot(run.t, value, color=colors[2], label='Actual MPPI, scale=0, seed 11', linewidth=1.3)
    ax.plot(run.t[-1], value[-1], 'x', color=colors[2], markersize=8)
for ax, title, unit in zip(axes,
        ['Minimum external clearance (truth)', 'Maximum absolute joint change', 'Base displacement'],
        ['mm', 'deg', 'mm']):
    ax.set(title=title, ylabel=unit)
    ax.grid(alpha=.22)
axes[0].axhline(0, color='#bc3434', linestyle='--', linewidth=1)
axes[0].annotate('Early retract: minimum 55.25 mm', xy=(9.2, 55.2465),
                 xytext=(11, 240), arrowprops={'arrowstyle': '->', 'color': colors[0]},
                 fontsize=9, color=colors[0])
axes[-1].set_xlabel('Elapsed simulation time (s); each trace stops at its recorded end')
axes[0].legend(fontsize=8, loc='upper right')
fig.suptitle('Timing feasibility diagnosis | prescribed traces are NOT MPPI results\n'
             'Independent 5 ms plant / 20 ms feedback delay / same geometry; crosses mark collisions',
             fontsize=11)
fig.tight_layout(rect=(0, 0, 1, .93))
fig.savefig(OUT / 'early_vs_late_retract.png', dpi=140)
plt.close(fig)
print(json.dumps({'output': str(OUT), 'plots': 5, 'gifs': 1}))
