#!/usr/bin/python3
"""Offline visibility figure from recorded CSV/JSON; never starts ROS."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT/'report'
NAMES = ('moving_ideal_s11', 'moving_mid360_s11')


def read_run(name):
    folder = ROOT/name
    result = json.loads((folder/'result.json').read_text())
    runner = json.loads((folder/'runner.json').read_text())
    with (folder/'sensor_visibility.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    rows = [row for row in rows if row['t'] and float(row['t']) >= 0.]
    actor = [row for row in rows if row['actor_present'] == 'True']
    positive = [row for row in actor if int(row['actor_visible_points']) > 0]
    zero = [row for row in actor if int(row['actor_visible_points']) == 0]

    def observation(row):
        if row is None:
            return None
        keys = ('t', 'publish_t', 'actor_sampled_points', 'actor_in_fov_points',
                'actor_visible_points', 'actor_visible_fraction', 'actor_elevation_min_deg',
                'actor_elevation_max_deg', 'actor_visible_z_min_m', 'actor_visible_z_max_m')
        return {key: (None if row[key] == '' else int(row[key]) if key.endswith('_points')
                      else round(float(row[key]), 9)) for key in keys}

    plant_origin = 1000.+result['simulated_s']-result['elapsed_s']
    usable = []
    nonempty = 0
    with (folder/'snapshot_headers.jsonl').open() as stream:
        for line in stream:
            item = json.loads(line)
            if item['elapsed_s'] is None:
                continue
            time = runner['start_ros_time']+item['elapsed_s']-plant_origin
            h = item['header']
            if time < 2. or h[13] <= 0:
                continue
            nonempty += 1
            if h[4] == 1 and h[19] == 1:
                usable.append(dict(receipt_t=round(time, 9), observation_t=round(h[18]-plant_origin, 9),
                                   point_count=int(h[13])))
    dynamic_updates = 0
    with (folder/'dynamic_observations.jsonl').open() as stream:
        for line in stream:
            if json.loads(line)['points']:
                dynamic_updates += 1
    summary = dict(sensor_model=result['sensor_model'], actor_onset_truth_t=2.,
        first_actor_observation=observation(actor[0]), last_actor_observation=observation(actor[-1]),
        first_zero_observation=observation(zero[0] if zero else None),
        last_nonzero_observation=observation(positive[-1] if positive else None),
        actor_frames=len(actor), nonzero_frames=len(positive), zero_frames=len(zero),
        visible_points_range=[min(int(row['actor_visible_points']) for row in actor),
                              max(int(row['actor_visible_points']) for row in actor)],
        maximum_fov_fraction=max(int(row['actor_in_fov_points'])/int(row['actor_sampled_points']) for row in actor),
        maximum_visible_fraction=max(float(row['actor_visible_fraction']) for row in actor),
        visible_z_range_m=([min(float(row['actor_visible_z_min_m']) for row in positive),
                            max(float(row['actor_visible_z_max_m']) for row in positive)] if positive else None),
        all_frames_after_first_zero_remain_zero=(all(int(row['actor_visible_points']) == 0
            for row in actor if float(row['t']) >= float(zero[0]['t'])) if zero else None),
        nonempty_dynamic_observation_updates=dynamic_updates,
        nonempty_snapshot_messages=nonempty, usable_nonempty_snapshot_messages=len(usable),
        first_usable_dynamic_snapshot=usable[0] if usable else None,
        last_usable_dynamic_snapshot=usable[-1] if usable else None,
        stop_reason=result['reason'], stop_t=result['elapsed_s'],
        physical_collision=result['collision_physical'],
        goal_finish_received=runner['finish_after_goal_ros_time'] is not None,
        final_base_x_m=result['final_state'][0], max_arm_change_deg=result['max_arm_change_deg'],
        source_files=[str((folder/file).relative_to(ROOT)) for file in
            ('sensor_visibility.csv', 'snapshot_headers.jsonl', 'dynamic_observations.jsonl', 'result.json', 'runner.json')])
    return rows, actor, summary


datasets = {name: read_run(name) for name in NAMES}
summary = dict(
    time_basis='Plant elapsed seconds; visibility t is delayed measurement time, publish_t is publication time. '
               'Snapshot times are receipt times reconstructed from runner start, aligned to Plant origin.',
    counts='Surface sample count, not calibrated MID360 returns. FOV count is before cylinder occlusion; '
           'visible count is the actual emitted actor subset before tracker voxel filtering.',
    geometry=dict(lidar_ground_height_m=.117, base_ground_height_m=.115, relative_lidar_offset_m=[.28,0.,.002],
                  actor_z_m=[1.05,1.30], mid360_elevation_deg=[-7.,52.], near_blind_range_m=.1),
    runs={name: datasets[name][2] for name in NAMES},
    conclusion='The MID360 geometric case loses all actor samples after t=2.735 s and never produces a '
               'qualified nonempty dynamic track. The ideal case also collides despite full visibility and '
               'valid dynamic points; visibility is not the only unresolved failure.',
    limitations=['Different closed-loop base trajectories; collision-time differences are not a pure sensor effect.',
                 'One run per condition; sphere/cylinder numerical truth, no hardware validation.',
                 'Known complete static-map prior; no robot occlusion, real scan pattern, reflectivity or range noise.',
                 'Valid empty dynamic snapshots do not establish that a blind region is clear.'])
(OUT/'sensor_summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')

plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':10, 'axes.titlesize':12,
                     'axes.spines.top':False, 'axes.spines.right':False})
fig, (counts_ax, angle_ax) = plt.subplots(1, 2, figsize=(13.6,5.2))
fig.subplots_adjust(left=.065, right=.98, bottom=.25, top=.77, wspace=.22)
fig.suptitle('High obstacle visibility: ideal vs geometric MID360', y=.97, fontsize=17)
fig.text(.5,.90,'Different closed-loop base trajectories: collision-time differences are not an isolated sensor effect.',
         ha='center', fontsize=10, color='#555555')
colors = {'moving_ideal_s11':'#277DA8', 'moving_mid360_s11':'#D16235'}
labels = {'moving_ideal_s11':'Ideal: complete surface', 'moving_mid360_s11':'MID360: visible surface'}
for name in NAMES:
    rows = datasets[name][0]
    counts_ax.step([float(row['t']) for row in rows], [int(row['actor_visible_points']) for row in rows],
                   where='post', color=colors[name], label=labels[name], linewidth=2.1)
counts_ax.set(xlabel='Plant time (s)', ylabel='Emitted obstacle points per frame', xlim=(0,10), ylim=(-15,590),
              title='Point cloud delivered to perception')
counts_ax.axvline(2., color='#888888', linestyle=':', linewidth=1)
counts_ax.text(2.14,560,'Actor appears',color='#666666',fontsize=9)
counts_ax.text(7.2,544,'529 points',color=colors[NAMES[0]],fontsize=10)
counts_ax.legend(loc='upper left', bbox_to_anchor=(.01,.88), frameon=True, facecolor='white',
                 edgecolor='none', framealpha=.95, fontsize=9)
counts_ax.grid(alpha=.18)

zoom = counts_ax.inset_axes([.43,.20,.54,.44])
mid = datasets['moving_mid360_s11'][1]
t = np.array([float(row['t']) for row in mid])
visible = np.array([int(row['actor_visible_points']) for row in mid])
zoom.step(t,visible,where='post',color=colors[NAMES[1]],linewidth=2)
zoom.scatter(t[t<2.8],visible[t<2.8],color=colors[NAMES[1]],s=12)
zoom.set(xlim=(2.,2.86),ylim=(-1.5,29),xticks=[2.,2.4,2.8],yticks=[0,10,20],title='MID360 detail: 25 to 0')
zoom.tick_params(labelsize=8)
zoom.title.set_fontsize(9)
zoom.grid(alpha=.18)
zoom.annotate('0 at 2.735 s',xy=(2.735,0),xytext=(2.37,12),fontsize=8,
              arrowprops={'arrowstyle':'->','color':'#555555'},color='#555555')

minimum = np.array([float(row['actor_elevation_min_deg']) for row in mid])
maximum = np.array([float(row['actor_elevation_max_deg']) for row in mid])
angle_ax.axhspan(52.,91.,facecolor='#F4D5CF',alpha=.6)
angle_ax.fill_between(t,minimum,maximum,color='#6B7E99',alpha=.23,label='Full actor elevation span')
angle_ax.plot(t,minimum,color='#34495E',linewidth=2,label='Lowest actor elevation')
angle_ax.plot(t,maximum,color='#8191A5',linewidth=1)
angle_ax.axhline(52.,color=colors[NAMES[1]],linestyle='--',linewidth=1.7,label='MID360 upper limit: 52 deg')
angle_ax.scatter([2.735],[52.1704870363],s=25,color=colors[NAMES[1]],zorder=4)
angle_ax.annotate('All points above FOV\n52.17 deg at 2.735 s',xy=(2.735,52.170487),xytext=(3.15,43.),
                 fontsize=9,arrowprops={'arrowstyle':'->','color':'#555555'},color='#555555')
angle_ax.text(6.,87.,'Outside vertical FOV',color='#8A5148',ha='center',fontsize=10)
angle_ax.set(xlabel='Plant time (s)',ylabel='Elevation from sensor (deg)',xlim=(1.9,8.3),ylim=(39,91),
             title='MID360 run: obstacle elevation exceeds the FOV')
angle_ax.legend(loc='upper left',bbox_to_anchor=(.005,.93),frameon=False,fontsize=8)
angle_ax.grid(alpha=.18)
fig.text(.065,.105,'Sensor height: 0.117 m | Actor: 1.05-1.30 m | First visible MID360 points are only at z=1.05 m.',fontsize=10)
fig.text(.065,.057,'Curves end at recorded sensor frames. Geometric FOV + cylinder occlusion only; static map is a known prior.',
         fontsize=9,color='#666666')
fig.savefig(OUT/'sensor_visibility.png',dpi=150,facecolor='white')
plt.close(fig)
print(OUT/'sensor_visibility.png')
print(OUT/'sensor_summary.json')
