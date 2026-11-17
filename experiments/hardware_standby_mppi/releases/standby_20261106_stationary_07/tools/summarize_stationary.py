#!/usr/bin/env python3
"""Summarize recorded stationary observations; no ROS or hardware access."""
import json
import gzip
from collections import Counter
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RELEASE = Path(__file__).resolve().parents[1]
REPORTS = RELEASE/'reports'
RUN = REPORTS/'actual_recording_13'


def read_rows(directory):
    path = directory/'shadow_status.json'
    if path.exists():
        return json.loads(path.read_text())
    with gzip.open(str(path)+'.gz', 'rt') as stream:
        return json.load(stream)


def stats(values):
    x = np.asarray(values)
    return dict(min=float(x.min()), median=float(np.median(x)),
                p95=float(np.percentile(x, 95)), max=float(x.max()))


report = json.loads((RUN/'report.json').read_text())
rows = read_rows(RUN)
planned = [r for r in rows if r['kernel_diagnostics']]
accepted = [r for r in planned if r['proposed_controls'] is not None]
t = np.array([r['ros_stamp']-planned[0]['ros_stamp'] for r in planned])
decision = np.array([r['decision_ms'] for r in planned])
state = np.array([r['state'] for r in planned])
xy_offset = np.linalg.norm(state[:, :2]-state[0, :2], axis=1)
joint_change = np.abs(state[:, 5:11]-state[0, 5:11])
cloud_age = np.array([[r['input_age_s'][key] for key in
    ('raw_cloud', 'filtered_cloud', 'dynamic_cloud')] for r in planned])
attempts = []
for path in sorted(REPORTS.glob('actual_*/report.json')):
    result = json.loads(path.read_text())
    samples = read_rows(path.parent)
    timing = [r['solve_ms'] for r in samples if r['kernel_diagnostics']]
    attempts.append(dict(run=path.parent.name, reason=result['reason'],
        accepted=result['status_counts'].get('shadow_candidate_only_not_validated_for_execution', 0),
        planning_attempts=len(timing), total_loop_ms=stats(timing) if timing else None,
        recorded_packet_count=max((r.get('command_gateway', {}).get('sent', 0)
                                   for r in samples), default=0),
        can_tx_delta=result['can_tx_after']-result['can_tx_before']))
gateways = [r['command_gateway'] for r in rows]
recorded = max(g['sent'] for g in gateways)
validation = [g['validation_ms'] for g in gateways
              if g['reason']=='command_sent' and g.get('validation_ms') is not None]
all_validation = [g['validation_ms'] for g in gateways if g.get('validation_ms') is not None]
transitions = []
previous = None
for row in rows:
    gateway = row['command_gateway']
    current = (gateway['enabled'], gateway['reason'])
    if current != previous:
        transitions.append(dict(stamp=row['ros_stamp'], planner_status=row['status'], **gateway))
        previous = current
summary = dict(scope='stationary real-input integration; command packets go to memory only',
    final_run=RUN.name, planned_count=len(planned), accepted_count=len(accepted),
    startup_wait_status_count=len(rows)-len(planned),
    accepted_window_s=float(t[-1]), accepted_interval_ms=stats(np.diff(t)*1000.),
    decision_ms=stats(decision), total_loop_ms=stats([r['solve_ms'] for r in planned]),
    cloud_age_s=stats(cloud_age), deadline_ms=95., maximum_input_age_s=.25,
    accepted_cloud_age_s=stats([r['input_age_s'][key] for r in accepted
        for key in ('raw_cloud', 'filtered_cloud', 'dynamic_cloud')]) if accepted else None,
    continuous_gateway_timing_passed=False,
    recorded_packet_count=recorded, gateway_reason_counts=dict(Counter(g['reason'] for g in gateways)),
    accepted_command_validation_ms=stats(validation) if validation else None,
    observed_validation_ms_including_repeated_statuses=stats(all_validation) if all_validation else None,
    gateway_transitions=transitions, execution_horizon_s=.14,
    explicit_horizon_validation_budget_ms=30., legacy_validation_budget_ms=20.,
    stationary_odometry_max_offset_m=float(xy_offset.max()),
    stationary_odometry_note='Observed pose variation while no motion was commanded; not measured physical base travel.',
    maximum_joint_change_rad=float(joint_change.max()),
    dynamic_track_point_counts=sorted(set(r['clouds']['dynamic_cloud']['points'] for r in planned)),
    memory=dict(owned_tree_peak_mib=report['maximum_tree_rss_mib'],
                system_available_min_mib=report['minimum_available_mib'],
                owned_tree_limit_mib=report['owned_tree_limit_mib'], system_available_floor_mib=report['memory_floor_mib']),
    can_tx_delta=report['can_tx_after']-report['can_tx_before'],
    actuator_publishers=report['actuator_publishers'],
    owned_processes_remaining=report['owned_processes_remaining'],
    physical_motion_tested=False, obstacle_coverage_certified=False,
    continuous_operation_certified=False, attempts=attempts,
    limitations=['Short stationary run only; dynamic frames contained no tracked objects.',
                 'Original PassThrough/CropBox near-field coverage is not certified.',
                 'No driver response, physical stopping, arm unfolding or moving-base localization test.',
                 'Timing assumes no marker subscriber; RViz display load was not tested.'])
(REPORTS/'stationary_summary.json').write_text(json.dumps(summary, indent=2)+'\n')

fig, axes = plt.subplots(2, 2, figsize=(10, 6.8))
ax = axes[0, 0]
ax.plot(t, decision, color='#176B87', linewidth=1, label='Decision incl. freshness gate')
rejected = np.array([r['proposed_controls'] is None for r in planned])
ax.scatter(t[rejected], decision[rejected], color='#C14343', s=12, label='Rejected proposal')
ax.axhline(95., color='#C14343', linestyle='--', label='Unchanged 95 ms deadline')
ax.set(ylabel='Milliseconds', xlabel='Seconds after first planned frame', ylim=(0, max(110, decision.max()+10)),
       title='%d/%d proposals accepted; %d packets recorded' % (len(accepted),len(planned),recorded))
ax.legend(fontsize=8)
ax = axes[0, 1]
for i, name in enumerate(('Raw', 'Filtered', 'Dynamic (empty tracks)')):
    ax.plot(t, cloud_age[:, i]*1000., linewidth=.8, label=name)
ax.axhline(250., color='#C14343', linestyle='--')
ax.set(ylabel='Input age at proposal (ms)', xlabel='Seconds', ylim=(0, 280), title='Freshness check retained')
ax.legend(fontsize=8)
ax = axes[1, 0]
ax.plot(t, xy_offset*1000., color='#9575A8')
ax.set(ylabel='Odometry variation (mm)', xlabel='Seconds', title='Stationary pose noise: NOT base travel')
ax = axes[1, 1]
ax.axis('off')
ax.text(.02, .97,
    'ACTUAL INPUTS / SHADOW OUTPUT ONLY\n\n'
    'CAN TX delta: 0\nActuator publishers: none\n'
    'Packets passing final check: %d\n'
    'Continuous gateway timing: NOT passed\n'
    'Measured joint change: %.6f rad\n\n'
    'Owned process peak: %.1f MiB (limit 650)\n'
    'Minimum available RAM: %.2f GiB\n\n'
    'Original filter coverage remains unvalidated.\n'
    'First physical response test is still pending.' %
    (recorded, joint_change.max(), report['maximum_tree_rss_mib'], report['minimum_available_mib']/1024.),
    va='top', fontsize=10, linespacing=1.5)
fig.suptitle('Stationary preparation 07 | real robot inputs | 2026-11-06', fontsize=13)
fig.tight_layout(rect=(0, 0, 1, .95))
fig.savefig(REPORTS/'stationary_preparation.png', dpi=150)
plt.close(fig)
print(json.dumps({k:summary[k] for k in ('planned_count','accepted_count','recorded_packet_count','decision_ms','accepted_command_validation_ms',
      'cloud_age_s','stationary_odometry_max_offset_m','maximum_joint_change_rad','memory')}))
