#!/usr/bin/python3
"""Offline plots of recorded standby trials; never extrapolate a stopped trial.

Example: plot_standby_sim.py --input /path/to/results --output /path/to/figures
Each run supplies state.csv, obstacles.csv, result.json, planner_params.yaml,
and optionally mppi_status.jsonl ({elapsed_s, status: {...}} per line).
The animation uses the current planner's recorded sphere geometry and DH chain,
not a mesh rendering or a claim of hardware validation. No ROS imports are used.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path

for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image, GifImagePlugin

from mppi.active_arm_model import PiperArmModel


COLORS = ('#2878b5', '#dc8616', '#208653')
JOINTS = ['joint%d' % i for i in range(1, 7)]


def read_table(path, required):
    """Bounded numeric CSV loading; absent observations remain absent."""
    if not path.is_file():
        return {name: np.empty(0) for name in required}
    rows = []
    with path.open() as stream:
        reader = csv.DictReader(stream)
        if not set(required).issubset(reader.fieldnames or []):
            raise ValueError('Missing CSV columns: '+str(path))
        for row in reader:
            if len(rows) >= 20000:
                raise ValueError('CSV exceeds the 20000-row plotting budget: '+str(path))
            rows.append([float(row[name]) for name in required])
    array = np.asarray(rows, dtype=float).reshape((-1, len(required)))
    if not np.isfinite(array).all():
        raise ValueError('Nonfinite recorded state: '+str(path))
    if len(array) > 1 and np.any(np.diff(array[:, 0]) < 0):
        raise ValueError('Recorded times are not monotonic: '+str(path))
    return dict(zip(required, array.T))


def planner_parameters(directory):
    path = directory/'planner_params.yaml'
    if not path.is_file():
        raise ValueError('Missing recorded planner_params.yaml: '+str(directory))
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError('Invalid planner parameter mapping: '+str(path))
    data = data.get('/remani_planner_node', data.get('remani_planner_node', data))
    # Reject fallback geometry: these two fields must come from the actual run.
    for key in ('fixed_collision_spheres', 'base_mani_fixed_joint_xyz_ypr'):
        if ('/remani_planner_node/mm/'+key not in data and
                key not in data.get('mm', {})):
            raise ValueError('Recorded planner geometry missing '+key+': '+str(path))
    return data


class Run:
    def __init__(self, directory, number):
        self.directory = directory
        self.name = directory.name.replace('_', ' ')
        if not self.name.isascii():
            self.name = 'Run %d' % (number+1)
        self.color = COLORS[number % len(COLORS)]
        self.result = json.loads((directory/'result.json').read_text())
        experiment = json.loads((directory/'experiment.json').read_text())
        self.navigation_goal = experiment.get('navigation_goal')
        scale = experiment.get('uncertainty_scale')
        self.uncertainty_scale = float(scale) if scale is not None else None
        self.deadline_ms = 1000.*float(experiment.get('controller_profile', {}).get('deadline_s', .095))
        self.completed = self.result.get('completed') is True
        self.reason = str(self.result.get('reason', 'unknown'))
        self.state = read_table(directory/'state.csv',
            ['t', 'x', 'y', 'yaw']+JOINTS+['external_gap_m'])
        self.obstacles = read_table(directory/'obstacles.csv',
            ['t', 'x', 'y', 'r', 'z_min', 'z_max'])
        self.model = PiperArmModel.from_parameters(planner_parameters(directory))
        self.model.lower[5] = max(self.model.lower[5], -.35)
        self.model.upper[5] = min(self.model.upper[5], .35)
        if self.result.get('physical_base_locked') is not False:
            raise ValueError('Run does not confirm an unlocked physical base: '+str(directory))
        self.t = self.state['t']
        self.end = float(self.t[-1]) if len(self.t) else 0.
        violation = self.result.get('first_violation') or {}
        self.stop = float(violation.get('t', self.end))
        if self.stop < self.end-1e-6:
            keep = self.t <= self.stop+1e-9
            self.state = {key: value[keep] for key, value in self.state.items()}
            self.t = self.state['t']
            self.end = float(self.t[-1]) if len(self.t) else 0.
        self.q = np.column_stack([self.state[name] for name in JOINTS])
        self.base = np.column_stack([self.state[name] for name in ('x', 'y', 'yaw')])
        self.timing = self.read_timing()

    def read_timing(self):
        path = self.directory/'mppi_status.jsonl'
        samples = []
        if not path.is_file():
            return np.empty((0, 3))
        elapsed = self.result.get('elapsed_s')
        start_ros = (1000.+float(self.result['simulated_s'])-float(elapsed)
                     if elapsed is not None and 'simulated_s' in self.result else None)
        with path.open() as stream:
            for number, line in enumerate(stream):
                if number >= 30000:
                    raise ValueError('JSONL exceeds plotting budget: '+str(path))
                item = json.loads(line)
                status = item.get('status', item.get('data', item))
                if isinstance(status, str):
                    status = json.loads(status)
                stamp = item.get('elapsed_s')
                if start_ros is not None and 'ros_time' in item:
                    stamp = float(item['ros_time'])-start_ros
                if stamp is None or not isinstance(status, dict):
                    continue
                stamp = float(stamp)
                cycle = float(status.get('cycle_ms', float('nan')))
                solve = (float(status.get('solve_ms', float('nan')))
                         if status.get('state') == 'command_delivered' else float('nan'))
                if math.isfinite(stamp) and 0. <= stamp <= self.end+1e-6:
                    samples.append((stamp, cycle, solve))
        return np.asarray(samples, dtype=float).reshape((-1, 3))

    def sample(self, stamp):
        index = np.searchsorted(self.t, min(stamp, self.end), side='right')-1
        if index < 0:
            return None
        return index, self.model.body_spheres(self.q[index], self.base[index])

    def actor(self, stamp):
        times = self.obstacles['t']
        index = np.searchsorted(times, min(stamp, self.end), side='right')-1
        if index < 0:
            return None
        return np.array([self.obstacles[name][index]
                         for name in ('x', 'y', 'r', 'z_min', 'z_max')])


def uncertainty_label(runs):
    scales = {run.uncertainty_scale for run in runs}
    if scales == {0.}:
        return 'scale=0: DIAGNOSTIC / zero prediction uncertainty'
    if scales == {1.}:
        return 'scale=1: Current uncertainty profile'
    if scales == {None}:
        return 'Uncertainty scale not recorded'
    if len(scales) == 1:
        return 'scale=%g: MODIFIED uncertainty profile' % next(iter(scales))
    return 'MIXED uncertainty: '+', '.join(run.name+' scale='+str(run.uncertainty_scale)
                                         for run in runs)


def plot_metrics(runs, output):
    fig, axes = plt.subplots(2, 3, figsize=(12, 7), sharex=True)
    titles = ('Minimum external clearance (truth)', 'Base displacement',
              'Base yaw change', 'Maximum absolute joint change',
              'MPPI cycle time (all recorded outcomes)',
              'MPPI solve time (delivered samples only)')
    units = ('mm', 'mm', 'deg', 'deg', 'ms', 'ms')
    for ax, title, unit in zip(axes.flat, titles, units):
        ax.set(title=title, xlabel='Elapsed simulation time (s)', ylabel=unit)
        ax.grid(alpha=.22)
    axes.flat[0].axhline(0., color='#bc3434', linewidth=1, linestyle='--')
    for run in runs:
        if not len(run.t):
            continue
        label = run.name+' ('+('duration complete' if run.completed else run.reason)+')'
        delta = np.rad2deg(run.q-run.q[0])
        values = [1000.*run.state['external_gap_m'],
                  1000.*np.linalg.norm(run.base[:, :2]-run.base[0, :2], axis=1),
                  np.rad2deg(np.unwrap(run.base[:, 2])-run.base[0, 2]),
                  np.max(np.abs(delta), axis=1)]
        for ax, value in zip(axes.flat, values):
            ax.plot(run.t, value, color=run.color, label=label, linewidth=1.5)
            if not run.completed:
                ax.plot(run.t[-1], value[-1], marker='x', color=run.color, markersize=7)
        timing = run.timing
        if len(timing) and np.any(np.isfinite(timing[:, 1])):
            axes.flat[4].plot(timing[:, 0], timing[:, 1], color=run.color)
        if len(timing) and np.any(np.isfinite(timing[:, 2])):
            axes.flat[5].plot(timing[:, 0], timing[:, 2], color=run.color,
                              marker='.', markersize=2.5)
    for ax in list(axes.flat)[4:]:
        if not ax.lines:
            ax.text(.5, .5, 'No timing records', ha='center', va='center', transform=ax.transAxes)
    for deadline in sorted({run.deadline_ms for run in runs}):
        axes.flat[4].axhline(deadline, color='#bc3434', linewidth=1, linestyle='--',
                            label='%g ms deadline' % deadline)
    axes.flat[4].legend(loc='upper right', fontsize=8)
    handles, labels = axes.flat[1].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc='lower center', ncol=min(3, len(handles)), fontsize=8)
    task = 'Navigation' if any(run.navigation_goal is not None for run in runs) else 'Standby avoidance'
    fig.suptitle(task+' | physical base unlocked; controller limits apply | crosses mark stops\n'+
                 uncertainty_label(runs))
    fig.tight_layout(rect=(0, .06, 1, .92))
    fig.savefig(output/'comparison.png', dpi=135)
    plt.close(fig)

    fig, axes = plt.subplots(len(runs), 1, figsize=(10, max(3, 2.4*len(runs))),
                             sharex=True, squeeze=False)
    for ax, run in zip(axes[:, 0], runs):
        if len(run.t):
            ax.plot(run.t, np.rad2deg(run.q-run.q[0]), linewidth=1.2)
        ax.set(title=run.name+' | '+run.reason, ylabel='Joint change (deg)')
        ax.grid(alpha=.22)
    axes[0, 0].legend(['Joint %d' % i for i in range(1, 7)], ncol=6, fontsize=8)
    axes[-1, 0].set_xlabel('Elapsed simulation time (s)')
    fig.suptitle(uncertainty_label(runs))
    fig.tight_layout(rect=(0, 0, 1, .94))
    fig.savefig(output/'joint_changes.png', dpi=135); plt.close(fig)


def sphere_faces(spheres):
    """Small icosahedral display mesh at each recorded collision sphere."""
    p = (1.+math.sqrt(5.))/2.
    vertices = np.array([[-1,p,0],[1,p,0],[-1,-p,0],[1,-p,0],
        [0,-1,p],[0,1,p],[0,-1,-p],[0,1,-p],[p,0,-1],[p,0,1],[-p,0,-1],[-p,0,1]])
    vertices /= np.linalg.norm(vertices[0])
    faces = np.array([[0,11,5],[0,5,1],[0,1,7],[0,7,10],[0,10,11],
        [1,5,9],[5,11,4],[11,10,2],[10,7,6],[7,1,8],[3,9,4],
        [3,4,2],[3,2,6],[3,6,8],[3,8,9],[4,9,5],[2,4,11],
        [6,2,10],[8,6,7],[9,8,1]])
    mesh = spheres[:, None, :3]+spheres[:, None, 3:4]*vertices[None]
    return mesh[:, faces].reshape((-1, 3, 3))


def cylinder_faces(actor):
    x, y, radius, low, high = actor
    angles = np.linspace(0., 2.*np.pi, 17)
    xy = np.column_stack((x+radius*np.cos(angles), y+radius*np.sin(angles)))
    bottom = np.column_stack((xy, np.full(len(xy), low)))
    top = np.column_stack((xy, np.full(len(xy), high)))
    faces = [[bottom[i], bottom[i+1], top[i+1], top[i]] for i in range(len(xy)-1)]
    return faces+[bottom[:-1], top[:-1]]


def draw_panel(ax, run, stamp, limits):
    ax.clear()
    stopped = stamp > run.end+1e-6
    failed = not run.completed and stamp >= run.stop-1e-6
    color = '#888888' if stopped or failed else run.color
    ax.set(xlim=limits[0], ylim=limits[1], zlim=limits[2],
           xlabel='x (m)', ylabel='y (m)', zlabel='z (m)')
    ax.set_xlabel('x (m)', fontsize=8, labelpad=1)
    ax.set_ylabel('y (m)', fontsize=8, labelpad=1)
    ax.set_zlabel('z (m)', fontsize=8, labelpad=1)
    ax.view_init(elev=22, azim=-65)
    if hasattr(ax, 'set_box_aspect'):
        ax.set_box_aspect([b-a for a, b in limits])
    ax.tick_params(labelsize=6, pad=0)
    sample = run.sample(stamp)
    if sample is None:
        ax.set_title(run.name, fontsize=10, color=color)
        ax.text2D(.5, .5, 'NO RECORDED STATE', ha='center', transform=ax.transAxes)
        return
    index, spheres = sample
    count = len(run.model.fixed_spheres)
    for subset, face_color, alpha in ((spheres[:count], '#83909b', .25),
                                       (spheres[count:], color, .58)):
        ax.add_collection3d(Poly3DCollection(sphere_faces(subset),
            facecolors=face_color, edgecolors='none', alpha=alpha))
    frames = run.model.native_transforms(run.q[index])
    chain = np.vstack((run.model.mount[:3, 3], frames[:, :3, 3]))
    x, y, yaw = run.base[index]
    c, s = math.cos(yaw), math.sin(yaw)
    chain[:, :2] = chain[:, :2] @ np.array([[c, s], [-s, c]])+[x, y]
    ax.plot(*chain.T, color=color, linewidth=3, marker='o', markersize=3)
    ax.plot(run.base[:index+1, 0], run.base[:index+1, 1], 0., color=color, linewidth=1.4)
    ax.quiver(x, y, .04, .18*c, .18*s, 0., color=color, arrow_length_ratio=.3)
    actor = run.actor(stamp)
    if actor is not None:
        ax.add_collection3d(Poly3DCollection(cylinder_faces(actor),
            facecolors='#888888' if stopped or failed else '#ed5c35', edgecolors='none', alpha=.68))
        ax.plot([actor[0], actor[0]], [actor[1], actor[1]], [0., actor[3]],
                color='#ed5c35', linestyle=':', linewidth=.7)
    displacement = 1000.*np.linalg.norm(run.base[index, :2]-run.base[0, :2])
    change = np.rad2deg(np.max(np.abs(run.q[index]-run.q[0])))
    label = ('STOPPED: '+run.reason if failed else
             'DURATION COMPLETE' if stamp >= run.end-1e-6 and run.completed else
             'RECORD ENDED' if stopped else 'RUNNING')
    ax.set_title(run.name+'\n'+label, fontsize=9, color=color)
    ax.text2D(.01, -.15, 'record %.2f s | gap %.1f mm\nbase %.1f mm | arm %.1f deg' %
        (run.t[index], 1000.*run.state['external_gap_m'][index], displacement, change),
        transform=ax.transAxes, fontsize=8, color=color)


def plot_animation(runs, output, fps, max_frames):
    task = 'Navigation' if any(run.navigation_goal is not None for run in runs) else 'Standby avoidance'
    end = max((run.end for run in runs), default=0.)
    times = np.linspace(0., end, min(max_frames, max(2, int(math.ceil(end*fps))+1)))
    lower, upper = np.array([-.45, -.55, 0.]), np.array([.55, .65, 1.45])
    # A common camera contains all actual recorded poses; no predicted motion.
    for run in runs:
        for stamp in times:
            sample = run.sample(stamp)
            if sample is not None:
                spheres = sample[1]
                lower = np.minimum(lower, np.min(spheres[:, :3]-spheres[:, 3:4], axis=0))
                upper = np.maximum(upper, np.max(spheres[:, :3]+spheres[:, 3:4], axis=0))
            actor = run.actor(stamp)
            if actor is not None:
                lower[:2] = np.minimum(lower[:2], actor[:2]-actor[2])
                upper = np.maximum(upper, [actor[0]+actor[2], actor[1]+actor[2], actor[4]])
    limits = list(zip(lower-.06, upper+.06)); limits[2] = (0., upper[2]+.08)
    fig = plt.figure(figsize=(10, 4.5), dpi=100)
    axes = [fig.add_subplot(1, len(runs), i+1, projection='3d') for i in range(len(runs))]
    fig.subplots_adjust(left=.02, right=.98, bottom=.16, top=.82, wspace=.13)
    palette = Image.new('P', (1, 1))
    colors = [channel for r in range(6) for g in range(6) for b in range(6)
              for channel in (51*r, 51*g, 51*b)]
    colors += [channel for grey in np.linspace(0, 255, 40).astype(int) for channel in [grey]*3]
    palette.putpalette(colors)
    destination = output/'comparison.gif'; temporary = output/'comparison.tmp.gif'
    try:
        with temporary.open('wb') as stream:
            for number, stamp in enumerate(times):
                for ax, run in zip(axes, runs):
                    draw_panel(ax, run, stamp, limits)
                fig.suptitle(task+(' | simulation time %.2f s\n'
                    'Physical base unlocked; controller limits apply | failed trials freeze in grey\n' % stamp)+
                    uncertainty_label(runs), fontsize=10)
                fig.canvas.draw()
                frame = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3]).quantize(
                    palette=palette, dither=Image.NONE)
                if number == 0:
                    header, _ = GifImagePlugin.getheader(frame, info={'loop': 0})
                    for chunk in header: stream.write(chunk)
                duration = round(1000./fps)+(2000 if number == len(times)-1 else 0)
                for chunk in GifImagePlugin.getdata(frame, duration=duration, disposal=2):
                    stream.write(chunk)
                frame.close()
            stream.write(b';')
        temporary.replace(destination)
    finally:
        plt.close(fig); palette.close()
        if temporary.exists(): temporary.unlink()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--run', action='append', type=Path, help='Run directory; repeat for each comparison')
    group.add_argument('--input', type=Path, help='Parent containing completed run directories')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--fps', type=int, default=5)
    parser.add_argument('--max-frames', type=int, default=120)
    args = parser.parse_args()
    if not 1 <= args.fps <= 10 or not 2 <= args.max_frames <= 150:
        parser.error('Use 1..10 fps and 2..150 frames to bound memory and output size')
    directories = args.run or sorted(path for path in args.input.iterdir()
        if path.is_dir() and (path/'state.csv').is_file() and (path/'result.json').is_file())
    if not 1 <= len(directories) <= 3:
        parser.error('Select one to three runs; no unfinished or unrelated runs')
    runs = [Run(path, i) for i, path in enumerate(directories)]
    args.output.mkdir(parents=True, exist_ok=True)
    plot_metrics(runs, args.output)
    plot_animation(runs, args.output, args.fps, args.max_frames)
    print('Wrote comparison.png, joint_changes.png and comparison.gif to '+str(args.output))


if __name__ == '__main__':
    main()
