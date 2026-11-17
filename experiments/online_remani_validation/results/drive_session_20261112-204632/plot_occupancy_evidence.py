"""Offline plots from the recorded diagnostic and registered point cloud audit."""
import datetime
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
import numpy as np

p = Path(__file__).resolve().parent
audit = json.loads((p/'goal_occupancy_audit.json').read_text())
pose = audit['goal_pose_xyyaw']
params = json.loads((p/'runtime_parameters.json').read_text())['/remani_planner_node']
fixed = np.asarray(params['mm']['fixed_collision_spheres'])
c, s = math.cos(pose[2]), math.sin(pose[2])
xy = fixed[:, :2] @ np.array([[c, s], [-s, c]]) + pose[:2]
retained = np.asarray([v['center'] for v in audit['blocking_voxels']
                       if v['occupancy_history'][-1]['code'] == 3])
fig, axes = plt.subplots(1, 3, figsize=(11.5, 5.6), sharex=True, sharey=True)
for ax, name, title in zip(axes, ['diag_02.npz', 'diag_05.npz', 'diag_12.npz'],
        ['Before walking', 'Walking recorded', 'After leaving + goal']):
    with np.load(p/name) as data:
        h, a = data['header'], data['array']
        n = int(h[38]); occ = a[40+n:].reshape(h[5:8].astype(int)).astype(np.uint8)
    points = h[8:11] + (np.argwhere((occ & 1) > 0)+.5)*h[11]
    points = points[(points[:, 2] >= .65) & (points[:, 2] <= 1.2)]
    ax.scatter(points[:, 0], points[:, 1], s=6, color='#b4bcc8', marker='s', label='Occupied, z=0.65-1.20 m')
    for i in np.flatnonzero((fixed[:, 2] >= .55) & (fixed[:, 2] <= .8)):
        ax.add_patch(Circle(xy[i], fixed[i, 3]+.05, fill=False, lw=.6, color='#007c91', alpha=.65))
    idx = np.floor((retained-h[8:11])/h[11]+1e-7).astype(int)
    present = np.asarray([bool(occ[tuple(k)] & 1) for k in idx])
    ax.scatter(retained[present, 0], retained[present, 1], s=27, c='#c93838', marker='s', label='Retained blocking cells')
    ax.plot(pose[0], pose[1], '+', color='#007c91', ms=12, mew=2)
    ax.arrow(pose[0], pose[1], .18*c, .18*s, width=.005, color='#007c91', length_includes_head=True)
    ax.set(title=title+'\n'+datetime.datetime.fromtimestamp(h[3]).strftime('%H:%M:%S'),
           xlim=(-.4, 1.5), ylim=(-2.15, .2), xlabel='world x (m)', aspect='equal')
    ax.grid(alpha=.15)
axes[0].set_ylabel('world y (m)')
axes[-1].legend(loc='upper left', fontsize=7)
fig.suptitle('Same requested goal: occupancy before walking and after leaving', y=.97, fontsize=12)
fig.text(.5, .91, 'Cyan: fixed-body goal checks; red: retained blockers; voxel size 5 cm', ha='center', fontsize=10)
fig.subplots_adjust(top=.79, bottom=.12, wspace=.06, left=.065, right=.985)
fig.savefig(p/'occupancy_before_walking_after_goal.png', dpi=160)
plt.close(fig)

raw = json.loads((p/'upstream_clear_ray_sample_audit.json').read_text())
example = raw['height_removed_examples'][0]
origin = np.asarray(example['sensor_origin'])
endpoint = np.asarray(example['raw_world_endpoint'])
cell = np.asarray(example['target'])
distance = np.linalg.norm(endpoint[:2]-origin[:2])
cell_distance = np.linalg.norm(cell[:2]-origin[:2])
fig, axes = plt.subplots(1, 2, figsize=(10, 4), sharex=True, sharey=True)
for i, ax in enumerate(axes):
    ax.plot(0, origin[2], 'ko', ms=6)
    ax.add_patch(Rectangle((cell_distance-.035, cell[2]-.025), .07, .05,
                          color='#c93838', zorder=4))
    ax.plot(distance, endpoint[2], 'x', color='#c93838', ms=11, mew=2)
    if i == 0:
        ax.plot([0, distance], [origin[2], endpoint[2]], color='#007c91', lw=2)
    ax.annotate('Old occupied voxel', (cell_distance, cell[2]),
                xytext=(.3, 1.3), arrowprops={'arrowstyle': '->'}, fontsize=9)
    ax.annotate('Measured high return\nsensor z=%.2f m' % example['raw_sensor_endpoint'][2],
                (distance, endpoint[2]), xytext=(2., 2.35),
                arrowprops={'arrowstyle': '->'}, fontsize=9)
    ax.set(xlim=(-.2, 4.2), ylim=(0, 2.8), xlabel='Horizontal distance from LiDAR (m)',
           title='Registered cloud: clearing ray present' if i == 0
                 else 'After 1.55 m height filter: ray absent')
    ax.grid(alpha=.2)
axes[0].set_ylabel('World height (m)')
fig.tight_layout()
fig.savefig(p/'height_filter_lost_clear_ray.png', dpi=160)
plt.close(fig)
