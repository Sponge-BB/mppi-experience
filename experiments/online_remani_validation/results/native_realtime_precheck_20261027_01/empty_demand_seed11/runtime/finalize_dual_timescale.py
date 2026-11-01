#!/usr/bin/env python3
"""Audit fixed-reference diagnostics and render the recorded experiment set."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DEFAULT = HERE / 'results/dual_timescale_20261018'


def read(path):
    return json.loads(path.read_text())


def write(path, obj):
    path.write_text(json.dumps(obj, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def csv_rows(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def audit_local(base):
    from run_mppi_rejoin_diagnosis import DEFAULT_CASES
    expected = sha(HERE / 'builds/native_v12_07/build/libonline_mppi.so')
    rows = []
    for source in ('source_01', 'source_02'):
        protocol = read(base / 'local_rejoin_01' / source / 'protocol.json')
        assert protocol['kernel_identical'] and protocol['actual_kernel_sha256'] == expected
        for case in DEFAULT_CASES:
            path = base / 'local_rejoin_01' / source / case
            result, metadata = read(path / 'result.json'), read(path / 'metadata.json')
            state = csv_rows(path / 'state.csv')
            assert len(state) == result['steps']
            assert result['library_sha256'] == expected
            assert sha(Path(metadata['library'])) == expected
            assert max(float(r['command_bound_ratio']) for r in state) <= 1 + 1e-8
            for row in state:
                if row['reason'] != 'ok':
                    assert abs(float(row['cmd_v'])) + abs(float(row['cmd_w'])) < 1e-12
            row = dict(source=source, case=case, passed=result['passed'],
                collision=result['collision'], sustained_rejoin=result['sustained_rejoin'],
                endpoint_reached=result['endpoint_reached'],
                elapsed_sim_s=result['simulated_duration_s'],
                first_rejoin_s=result['first_sustained_rejoin_s'],
                minimum_physical_gap_m=result['min_physical_gap_m'],
                active_zero_s=.05 * result['reason_counts'].get('active_zero', 0),
                deadline_misses=result['deadline_misses'],
                callback_p95_ms=result['callback_p95_ms'],
                final_distance_to_reference_m=result['final_geometric_reference_error_m'],
                max_command_bound_ratio=result['max_command_bound_ratio'],
                library_sha256=expected, path=str(path))
            rows.append(row)
    assert len(rows) == 18
    with (base / 'local_rejoin_summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    failed = base / 'local_rejoin_01/source_02/dynamic_crossing'
    repeated = base / 'local_rejoin_01/confirm_dynamic_endpoint_01/dynamic_crossing'
    original, repeat = csv_rows(failed / 'state.csv'), csv_rows(repeated / 'state.csv')
    columns = ('t', 'x', 'y', 'yaw', 'v', 'w', 'cmd_v', 'cmd_w', 'reason', 'active_zero')
    identical = len(original) == len(repeat) and all(
        all(a[k] == b[k] for k in columns) for a, b in zip(original, repeat))
    assert identical, 'The same-seed confirmation is not an exact state/control reproduction'
    result = read(failed / 'result.json')
    ref = np.genfromtxt(failed / 'reference.csv', delimiter=',', names=True)
    bundle = read(failed / 'reference_bundle.json')
    cusp_time = sum(piece['duration'] for piece in bundle[0]['polynomials'])
    cusp_index = int(np.argmin(abs(ref['t'] - cusp_time)))
    pose = np.asarray(result['final_state'])
    zeros = [abs(float(r['cmd_v'])) + abs(float(r['cmd_w'])) < 1e-12 for r in original]
    last_nonzero = max((i for i, z in enumerate(zeros) if not z), default=-1)
    traces = [json.loads(line) for line in (failed / 'search_trace.jsonl').read_text().splitlines() if line]
    last_trace = traces[-1]
    samples = np.asarray(last_trace['rows'])
    proposals = []
    for ident in np.unique(samples[:, 0]):
        sequence = samples[samples[:, 0] == ident]
        proposals.append(dict(candidate=int(ident), cost=float(sequence[0, 4]),
            predicted_gap_m=float(sequence[0, 5]), feasible=bool(sequence[0, 6]),
            selected=bool(sequence[0, 7]),
            net_displacement_m=float(np.linalg.norm(sequence[-1, 2:4] - sequence[0, 2:4]))))
    stall = dict(scope='Read-only analysis of retained failed/confirmation runs; no controller modification',
        same_seed_state_and_controls_identical=identical,
        confirmation_label_note='Directory name endpoint was a provisional hypothesis; measured failure is near an intermediate gear change, not the final goal.',
        final_state=result['final_state'], reference_error_m=result['final_geometric_reference_error_m'],
        distance_to_final_goal_m=float(np.linalg.norm(pose[:2] - [ref['x'][-1], ref['y'][-1]])),
        first_gear_end_time_s=cusp_time,
        final_projected_reference_time_s=result['final_progress_time_s'],
        distance_to_first_gear_endpoint_m=float(np.linalg.norm(pose[:2] - [ref['x'][cusp_index], ref['y'][cusp_index]])),
        gear_signs=[message['singul'] for message in bundle],
        terminal_zero_command_start_sim_s=float(original[last_nonzero + 1]['t']),
        terminal_zero_command_duration_s=.05 * (len(original) - last_nonzero - 1),
        deadline_misses=result['deadline_misses'], validator_codes=result['validator_code_counts'],
        last_search_sim_s=last_trace['t'], last_search_proposals=proposals,
        interpretation='Safe moving proposals were present, but a stationary proposal had the lowest recorded merit. This is not a Tracker clock hold or a REMANI solve wait. The specific merit/gear-transition repair still needs a controlled algorithm ablation.')
    write(base / 'mppi_stall_diagnosis.json', stall)
    summary = dict(trials=len(rows), passed=sum(r['passed'] for r in rows),
        collisions=sum(r['collision'] for r in rows),
        deadline_misses=sum(r['deadline_misses'] for r in rows),
        all_commands_within_bounds=all(r['max_command_bound_ratio'] <= 1+1e-8 for r in rows),
        rows=rows, confirmation=stall,
        scope='18 controlled fixed-native-reference fixtures, plus one confirmation; NOT 19 independent navigation trials, not Gazebo or real vehicle.',
        success='collision-free, sustained safe rejoin, settled final pose, commands within limits')
    write(base / 'local_rejoin_summary.json', summary)
    return summary


def plot_summaries(base, local):
    from report import plt
    summaries = read(base / 'online_analysis_02/batch_summary.json')['rows']
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.7))
    for ax, scene, title in zip(axes, ('static', 'dynamic_head_on_center', 'dynamic_four'),
                               ('静态', '中心迎面', '四动态')):
        selected = [r for r in summaries if r['scene'] == scene]
        for i, row in enumerate(selected):
            total = row['native_window_wall_s']
            if total is None:
                ax.text(.03, i, '未交付新参考：21次刷新均失败', color='#b65349', va='center', fontsize=9)
                continue
            request, wait, other = (row[k] for k in ('requests_union_wall_s', 'explicit_wait_union_wall_s', 'unattributed_wall_s'))
            left = 0.
            for value, color, label in zip((request, wait, other), ('#638ca5', '#d8ad58', '#c7cbd0'),
                                            ('规划请求', '显式等待', '其他间隙')):
                ax.barh(i, value, left=left, height=.52, color=color, label=label if i==0 else None)
                if value>.5:ax.text(left+value/2, i, '%.2f' % value, ha='center', va='center', fontsize=8)
                left += value
            ax.text(total+.04, i, '%.2fs' % total, va='center', fontsize=8)
        ax.set_title(title); ax.set_yticks([0, 1]); ax.set_yticklabels(['重复1', '重复2'])
        ax.set_ylim(1.6, -.6); ax.set_xlim(0, max([r['native_window_wall_s'] or 0 for r in selected]+[1])*1.22)
        ax.set_xlabel('确认参考受阻 → 发布新参考 / 墙钟秒'); ax.grid(axis='x', alpha=.15)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', ncol=3, frameon=False)
    fig.tight_layout(rect=[0, 0, 1, .90]); fig.savefig(base/'01_replan_time_breakdown.png', dpi=160); plt.close(fig)
    labels = ['原轨迹', '+10cm', '-10cm', '+20cm', '-20cm', '+30cm', '-30cm', '静态绕障', '动态横穿']
    fig, axes = plt.subplots(1, 2, figsize=(10, 5), sharex=True, sharey=True)
    for ax, source in zip(axes, ('source_01', 'source_02')):
        selected = [r for r in local['rows'] if r['source'] == source]
        for i, row in enumerate(selected):
            ax.barh(i, row['elapsed_sim_s'], color='#5b997f' if row['passed'] else '#bd685c', height=.62)
            ax.text(row['elapsed_sim_s']+.4, i,
                    '%.1fs%s' % (row['elapsed_sim_s'], '' if row['passed'] else ' 未到终点'), va='center', fontsize=8)
        ax.set_title('固定原生参考 '+source[-2:]); ax.set_yticks(range(9)); ax.set_yticklabels(labels)
        ax.set_xlim(0, 68); ax.set_ylim(8.7, -.7); ax.set_xlabel('运行的仿真时间 / s'); ax.grid(axis='x', alpha=.15)
    fig.suptitle('现有 MPPI 独立回线：绿色通过，红色未满足完整判据（不是算法改良对照）', fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, .93]); fig.savefig(base/'02_mppi_rejoin_results.png', dpi=160); plt.close(fig)


def render_all(base, refresh=False):
    from render_dual_timescale import render, plot_timeline, Trial
    jobs = []
    scene_names = {'empty':'空旷', 'static':'静态障碍', 'dynamic_head_on_center':'中心迎面', 'dynamic_four':'四动态'}
    for trial in sorted((base/'online_validation_01').iterdir()):
        if not (trial/'result.json').exists():continue
        scene = read(trial/'result.json')['scene']
        jobs.append(([trial], base/'figures/online'/trial.name, [scene_names[scene]+' | REMANI＋MPPI']))
    for trial in sorted((base/'delivery_screen_01').iterdir()):
        if not (trial/'result.json').exists():continue
        delay = read(trial/'manifest.json')['reference_delay_s']
        jobs.append(([trial], base/'figures/delivery'/trial.name, ['参考额外迟到 %.1fs' % delay]))
    case_names = {'native_turn':'原生转弯参考', 'static_rejoin':'静态绕障后回线', 'dynamic_crossing':'动态横穿后回线'}
    for source in ('source_01', 'source_02'):
        for trial in sorted((base/'local_rejoin_01'/source).iterdir()):
            if not (trial/'result.json').exists():continue
            label = case_names.get(trial.name, trial.name.replace('offset_p', '初始侧移 +').replace('offset_n','初始侧移 -')+'cm')
            if trial.name.startswith('offset_'):
                label = '初始侧移 %s%dcm' % ('+' if trial.name[7]=='p' else '-', int(trial.name[8:]))
            jobs.append(([trial], base/'figures/local_rejoin'/source/trial.name, [source[-2:]+' | '+label]))
    first, second = base/'local_rejoin_01/source_01', base/'local_rejoin_01/source_02'
    jobs.append(([base/'local_rejoin_01/confirm_dynamic_endpoint_01/dynamic_crossing'],
        base/'figures/local_rejoin/confirmation_dynamic_stall', ['相同输入复核：换向附近停滞']))
    jobs.extend([
        ([first/'native_turn', first/'offset_p030'], base/'figures/comparisons/initial_offset', ['正常起步', '初始侧移 +30cm']),
        ([first/'static_rejoin', first/'dynamic_crossing'], base/'figures/comparisons/static_dynamic', ['静态绕障：固定参考', '动态横穿：固定参考']),
        ([first/'dynamic_crossing', second/'dynamic_crossing'], base/'figures/comparisons/dynamic_success_stall', ['参考01：到达终点', '参考02：换向附近停滞']),
    ])
    index = ['# GIF 索引', '', '全部为数值仿真、默认2倍播放。两个并排样本使用不同参考时，不是算法A/B对照。', '']
    for sources, dest, titles in jobs:
        if dest.exists() and not refresh:
            assert (dest/'overview.gif').exists() and (dest/'visualization_audit.json').exists(), 'Incomplete existing visualization '+str(dest)
            plot_timeline([Trial(p, t) for p, t in zip(sources, titles)], dest)
        else:
            render(sources, dest, titles, playback=2., step=.2, refresh=refresh)
        index.append('- [%s](%s)' % (' / '.join(titles), (dest/'overview.gif').relative_to(base)))
        print('GIF_COMPLETE', dest.relative_to(base), flush=True)
    (base/'GIF_INDEX.md').write_text('\n'.join(index)+'\n')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', type=Path, default=DEFAULT)
    p.add_argument('--render', action='store_true')
    p.add_argument('--refresh-visuals', action='store_true', help='Regenerate only this experiment set\'s derived pictures; no controller execution')
    a = p.parse_args(); base = a.out.resolve()
    local = audit_local(base)
    plot_summaries(base, local)
    if a.render or a.refresh_visuals:render_all(base, a.refresh_visuals)
    print('FINALIZE_COMPLETE', base, 'local passed', local['passed'], '/', local['trials'], flush=True)


if __name__ == '__main__':main()
