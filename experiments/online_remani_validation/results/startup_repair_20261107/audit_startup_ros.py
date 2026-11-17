#!/usr/bin/python3
"""Read one completed isolated startup trial; never starts or controls ROS."""
from collections import Counter, defaultdict
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import psutil

OUT = Path(__file__).resolve().parent/'ros_startup_integration_s11'


def read(name):
    return json.loads((OUT/name).read_text())


def lines(name):
    return [json.loads(line) for line in (OUT/name).read_text().splitlines() if line]


def distribution(values):
    return dict(zip(('p50', 'p95', 'max'), map(float, np.percentile(values, [50, 95, 100])))) if len(values) else None


def main():
    global OUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=OUT)
    OUT = parser.parse_args().out.resolve()
    result, runner = read('result.json'), read('runner.json')
    statuses, commands, snapshots = lines('mppi_status.jsonl'), lines('arm_command_times.jsonl'), lines('snapshot_headers.jsonl')
    ready = next((row for row in statuses if row['status'].get('startup_ready')), None)
    normal = [row for row in statuses if ready and row['ros_time'] > ready['ros_time'] and
              row['status'].get('state') == 'command_delivered' and row['status'].get('mode') != 'startup_to_standby']
    settled_since = last_startup_time = None
    for row in statuses:
        value = row['status']; state = value.get('candidate_diagnostics', {}).get('measured_state', [])
        if value.get('mode') != 'startup_to_standby':
            continue
        last_startup_time = row['ros_time']
        settled = (value.get('state') == 'command_delivered' and value.get('startup_error_rad', 1.) < .015
                   and len(state) == 17 and max(abs(v) for v in state[11:17]) < .02)
        settled_since = (settled_since if settled_since is not None else row['ros_time']) if settled else None
    received = np.array([row['ros_time'] for row in commands])
    source = np.array([row['source_stamp'] for row in commands])
    wall = np.array([row['wall_monotonic'] for row in commands])
    previous_source_age = received[1:]-source[:-1]
    previous_lease_wall_age = np.diff(wall)+(received-source)[:-1]
    def source_lease(values):
        return dict(age_s=distribution(values), at_or_over_0_30_s_count=int(np.sum(values >= .30-1e-9)))
    state_rows = list(csv.DictReader((OUT/'state.csv').open()))
    boundary = float('inf') if ready is None else ready['elapsed_s']
    stages = {}
    for name, before in (('startup', True), ('normal', False)):
        rows = [row for row in state_rows if (float(row['t']) <= boundary) == before]
        cycles = [row['status'] for row in statuses if
                  row['status'].get('mode', row['status'].get('candidate_diagnostics', {}).get('mode')) ==
                  ('startup_to_standby' if before else 'standby_extension')]
        stages[name] = dict(samples=len(rows), status_counts=dict(Counter(row['state'] for row in cycles)),
            cycle_ms=distribution([row['cycle_ms'] for row in cycles]),
            max_base_displacement_m=max((float(np.hypot(float(row['x']), float(row['y']))) for row in rows), default=None),
            max_base_yaw_rad=max((abs(float(row['yaw'])) for row in rows), default=None),
            min_recorded_configuration_gap_m=min((float(row['configuration_gap_m']) for row in rows), default=None),
            max_command_v_m_s=max((abs(float(row['command_v'])) for row in rows), default=None),
            max_command_w_rad_s=max((abs(float(row['command_w'])) for row in rows), default=None))
    def gaps(times):
        delta = np.diff(times)
        return dict(interval_s=distribution(delta), over_0_30_s_count=int(np.sum(delta > .30+1e-9)),
                    negative_count=int(np.sum(delta < 0.)))
    ages = [row['status']['candidate_diagnostics']['plan_time']-row['status']['candidate_diagnostics']['snapshot_header'][3]
            for row in statuses if len(row['status'].get('candidate_diagnostics', {}).get('snapshot_header', [])) == 32]
    generations = defaultdict(set)
    for row in snapshots:
        generations[row['header'][14]].add(row['header'][3])
    interval_gaps = np.diff(received)
    dangerous = [dict(previous_ros_time=float(received[index]), next_ros_time=float(received[index+1]),
                     gap_s=float(gap)) for index, gap in enumerate(interval_gaps) if gap > .30+1e-9]
    port_hex = format(11466, '04X')
    port_open = any(line.split()[1].split(':')[-1] == port_hex and line.split()[3] == '0A'
                    for file in ('/proc/net/tcp', '/proc/net/tcp6')
                    for line in Path(file).read_text().splitlines()[1:])
    graph = []
    for process in psutil.process_iter(['pid', 'cmdline']):
        command = ' '.join(process.info['cmdline'] or [])
        if (str(OUT/'trial.launch') in command or str(OUT/'roslog') in command or
                ('rosmaster' in command and '11466' in command)):
            graph.append(dict(pid=process.pid, command=command))
    report = dict(scope='One real isolated ROS graph and independent 5 ms Plant, no hardware; subscriber receipt is not actual driver receipt',
        result=result, runner_reason=runner['runner_reason'], peak_graph_rss_mib=runner['peak_graph_rss_mib'],
        startup_ready=ready is not None, startup_ready_ros_time=None if ready is None else ready['ros_time'],
        startup_ready_elapsed_s=None if ready is None else ready['elapsed_s'],
        final_stable_observed_span_s=None if settled_since is None else last_startup_time-settled_since,
        normal_deliveries_after_ready=len(normal), normal_modes=dict(Counter(row['status'].get('mode') for row in normal)),
        normal_observed_span_s=0. if not normal else normal[-1]['ros_time']-normal[0]['ros_time'],
        phases=stages,
        status_counts=dict(Counter(row['status']['state'] for row in statuses)),
        cycle_ms=distribution([row['status']['cycle_ms'] for row in statuses if 'cycle_ms' in row['status']]),
        delivered_cycle_ms=distribution([row['status']['cycle_ms'] for row in statuses if row['status'].get('state') == 'command_delivered']),
        configured_deadline_ms=95., arm_received_count=len(commands),
        command_intervals=dict(ros_receipt=gaps(received), source_stamp=gaps(source), wall_receipt=gaps(wall)),
        previous_command_lease=dict(next_ros_receipt_minus_previous_source=source_lease(previous_source_age),
            wall_interval_plus_previous_receipt_age=source_lease(previous_lease_wall_age),
            message_source_age_at_receipt_s=distribution(received-source)),
        receipt_gaps_over_watchdog=dangerous,
        snapshot_count=len(snapshots), snapshot_h4_valid_fraction=sum(row['header'][4] == 1. for row in snapshots)/len(snapshots),
        snapshot_h3_age_at_plan_s=distribution(ages),
        exported_generations=len(generations), same_generation_changed_fusion_stamp_count=sum(len(v) > 1 for v in generations.values()),
        shutdown=dict(port_11466_open=port_open, remaining_trial_or_master_processes=graph,
                      available_memory_mib=psutil.virtual_memory().available/1024**2),
        limitations=['Plant .30 s watchdog brakes without the real driver stop latch; any longer gap prevents claiming hardware-equivalent continuous execution.',
                     'Snapshot fusion-stamp equality within a generation is observed consistency, not a stale-source fault injection.',
                     'Empty MID360 geometric sensing and numerical Plant do not calibrate real tracking, contact, sensing or emergency stop.'])
    summary_name = 'ros_startup_summary.json' if OUT.name == 'ros_startup_integration_s11' else OUT.name+'_summary.json'
    (OUT.parent/summary_name).write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps({key: report[key] for key in ('startup_ready', 'startup_ready_elapsed_s',
        'normal_deliveries_after_ready', 'normal_observed_span_s', 'status_counts', 'cycle_ms',
        'command_intervals', 'snapshot_h4_valid_fraction', 'snapshot_h3_age_at_plan_s',
        'same_generation_changed_fusion_stamp_count', 'peak_graph_rss_mib', 'shutdown')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
