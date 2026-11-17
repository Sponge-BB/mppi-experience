#!/usr/bin/env python3
"""Read closed evidence only; no ROS node and no runtime edits."""
import collections
import json
import pathlib
import re
import statistics

import rosbag

ROOT = pathlib.Path(__file__).resolve().parent
GOAL = 1794456459.9303157
STOP = 1794456497.2009706


def phase(t):
    return 'before_goal' if t < GOAL else 'active_navigation' if t <= STOP else 'after_final_stop'


def stats(values):
    if not values:
        return None
    values = sorted(values)
    return dict(n=len(values), median=statistics.median(values),
                p95=values[int(.95 * (len(values)-1))], maximum=values[-1])


def main():
    snapshots = collections.defaultdict(list)
    for line in (ROOT / 'snapshot_headers.jsonl').open():
        row = json.loads(line)
        snapshots[phase(row['receive_ros'])].append(row)
    header_result = {}
    for key, rows in snapshots.items():
        ages = [(r['receive_ros']-r['header'][3])*1000 for r in rows]
        versions = len(set(r['header'][14] for r in rows))
        duration = rows[-1]['receive_ros']-rows[0]['receive_ros']
        header_result[key] = dict(
            packets=len(rows), duration_s=duration, packet_hz=(len(rows)-1)/duration,
            field_versions=versions, versions_per_s=(versions-1)/duration,
            fusion_age_at_recorder_receipt_ms=stats(ages),
            received_older_than_map_ttl=sum(r['receive_ros']-r['header'][3] > r['header'][15] for r in rows),
            invalid_header=sum(r['header'][4] != 1 for r in rows),
            esdf_copy_age_at_receipt_ms=stats([(r['receive_ros']-r['header'][17])*1000 for r in rows]),
            cached_joint_age_at_receipt_ms=stats([(r['receive_ros']-r['header'][24])*1000 for r in rows]),
            dynamic_age_at_receipt_ms=stats([(r['receive_ros']-r['header'][18])*1000 for r in rows]))
    clouds = {t: {} for t in ('/velodyne_points', '/velodyne_points_mapping')}
    logs = []
    status_counts = collections.defaultdict(collections.Counter)
    good_cycles = collections.defaultdict(list)
    expired_geometry = []
    with rosbag.Bag(str(ROOT/'capture.bag'), 'r') as bag:
        for topic, msg, receipt in bag.read_messages(topics=list(clouds)+['/rosout_agg', '/mppi/status']):
            stamp = receipt.to_sec()
            if topic in clouds:
                clouds[topic][(msg.header.stamp.to_nsec(), msg.header.frame_id)] = dict(
                    points=msg.width*msg.height, bytes=len(msg.data), receipt=stamp,
                    point_step=msg.point_step)
            elif topic == '/rosout_agg':
                if any(s in msg.msg for s in ('[GridMap][pipeline]', '[GridMap][LocalESDF]')):
                    logs.append(dict(source=msg.header.stamp.to_sec(), receipt=stamp, message=msg.msg))
            else:
                data = json.loads(msg.data)
                key = phase(stamp)
                status_counts[key][data.get('state')] += 1
                if data.get('state') == 'command_delivered':
                    good_cycles[key].append(data.get('cycle_ms', 0))
                if data.get('state') == 'no_feasible_candidate':
                    context = data.get('candidate_diagnostics', {})
                    expired_geometry.append(dict(receipt=stamp, phase=key,
                        cycle_ms=data.get('cycle_ms'), plan_time=context.get('plan_time'),
                        seq=(context.get('snapshot_header') or [0, 0, None])[2]))
    common = set(clouds['/velodyne_points']) & set(clouds['/velodyne_points_mapping'])
    full_points = sum(clouds['/velodyne_points'][k]['points'] for k in common)
    map_points = sum(clouds['/velodyne_points_mapping'][k]['points'] for k in common)
    full_bytes = sum(clouds['/velodyne_points'][k]['bytes'] for k in common)
    map_bytes = sum(clouds['/velodyne_points_mapping'][k]['bytes'] for k in common)
    cloud_result = dict(exact_stamp_and_frame_pairs=len(common),
        full_recorded=len(clouds['/velodyne_points']), mapping_recorded=len(clouds['/velodyne_points_mapping']),
        mean_full_points=full_points/len(common), mean_mapping_points=map_points/len(common),
        point_reduction_fraction=1-map_points/full_points,
        payload_reduction_fraction=1-map_bytes/full_bytes,
        full_point_steps=sorted(set(v['point_step'] for v in clouds['/velodyne_points'].values())),
        mapping_point_steps=sorted(set(v['point_step'] for v in clouds['/velodyne_points_mapping'].values())))
    workers = []
    pattern = re.compile(r'committed regions=(\d+) compute=(\d+)/(\d+).*worker=([\d.]+)ms commit=([\d.]+)ms')
    for row in logs:
        match = pattern.search(row['message'])
        if match:
            r, compute, bound, worker, commit = map(float, match.groups())
            workers.append(dict(source=row['source'], phase=phase(row['source']), regions=int(r),
                computed_voxels=int(compute), input_bound_voxels=int(bound), worker_ms=worker,
                commit_ms=commit, voxel_fraction=compute/bound if bound else None))
    output = dict(time_windows=dict(goal_receipt=GOAL, final_stop_command_receipt=STOP),
        snapshot_timing=header_result, matched_clouds=cloud_result,
        mppi_status_counts={k:dict(v) for k,v in status_counts.items()},
        command_delivered_cycle_ms={k:stats(v) for k,v in good_cycles.items()},
        no_feasible_cycles=expired_geometry, sparse_worker_log_samples=workers,
        sparse_worker_ms=stats([v['worker_ms'] for v in workers]),
        pipeline_log_samples=logs,
        limitations=['snapshot age is at recorder receipt, which may lag decision time',
          'cloud counts compare exact matching source stamp and frame only',
          'data-volume reduction does not measure end-to-end latency improvement',
          'performance log samples are throttled and not every worker cycle',
          'cycle_ms is measured after failure instrumentation; it is not exact solve_ms',
          'a cached unchanged field may reuse its original ESDF-copy timestamp'])
    (ROOT/'map_timing_and_cloud_audit.json').write_text(json.dumps(output,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:v for k,v in output.items() if k not in ('pipeline_log_samples', 'sparse_worker_log_samples')},ensure_ascii=False,indent=2))


if __name__ == '__main__':
    main()
