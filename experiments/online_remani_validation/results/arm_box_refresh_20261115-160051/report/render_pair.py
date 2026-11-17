#!/usr/bin/env python3
"""Reproduce this saved pair using the existing arm-box report module only.

Run: PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 \
     /usr/bin/python3 render_pair.py
No ROS, compilation, optimizer replay, source edits, or motion commands.
The parent process stops only its own worker if RSS exceeds 200 MiB.
"""
import gc
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

OUT = Path(__file__).resolve().parent
CASE = OUT.parent
HERE = Path(__file__).resolve().parents[3]
PAIR = [('legacy_seed11', '旧版刷新 · seed 11', '#356d9b'),
        ('fixed_seed11', '新鲜 prefix / 统一 seed · seed 11', '#dc833b')]
NOTE = '同配置、同障碍出现规则；初始轨迹并非完全相同。此图是配对示例，未据此宣称整体避障改善。'


def digest(path):
    value = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            value.update(block)
    return value.hexdigest()


def worker():
    sys.path.insert(0, str(HERE))
    import report_arm_box_navigation as report
    from matplotlib.figure import Figure
    inputs = []
    trials = []
    report.LABELS = {name: label for name, label, _ in PAIR}
    report.COLORS = {name: color for name, _, color in PAIR}
    for name, label, _ in PAIR:
        source = CASE / name / 'mppi'
        status = report.read_json(source / 'run_status.json')
        if not status.get('result_written') or not status.get('owned_process_group_stopped'):
            raise RuntimeError('Pair source must be complete and stopped: ' + str(source))
        trial = report.load_trial(source)
        trial['metrics'].update(group=name, label=label, actual_trial_directory=str(source))
        inputs.append(dict(source=source, sha256=dict(trial['metrics']['source_sha256'])))
        # Existing plot functions choose a palette key using path.name. Keep
        # the real data/source path explicit above; this is a display key only.
        trial['path'] = CASE / name
        trials.append(trial)
    original_suptitle = Figure.suptitle
    def annotated_title(self, text, *args, **kwargs):
        return original_suptitle(self, text + '\n' + NOTE, *args, **kwargs)
    Figure.suptitle = annotated_title
    report.plot_comparison(trials, OUT)
    gc.collect()
    animation = report.make_gif(trials, OUT, max_frames=55)
    gc.collect()
    Figure.suptitle = original_suptitle
    header = ('<header><h1>机械臂前方突现长方体：旧版与新鲜输入刷新</h1>'
              '<p>两组均为 REMANI＋MPPI，按各轮实际起步时刻对齐。' + NOTE +
              '</p><p>修复范围：可选慢刷新使用新鲜 prefix，并统一该次请求的 seed。'
              '三维姿态、底盘轨迹和结果均来自这两轮已保存记录。</p></header>')
    report.HTML = re.sub(r'<header>.*?</header>', lambda _: header, report.HTML, count=1, flags=re.S)
    report.HTML = report.HTML.replace('grid-template-columns:repeat(3,minmax(0,1fr))',
                                     'grid-template-columns:repeat(2,minmax(0,1fr))')
    report.make_html(trials, OUT)
    for entry in inputs:
        for name, expected in entry['sha256'].items():
            if digest(entry['source'] / name) != expected:
                raise RuntimeError('Trial source changed: ' + str(entry['source'] / name))
    summary = dict(schema='arm_box_refresh_pair_report_v1', note=NOTE,
        comparison_scope='same configuration and appearance rule; not identical initial native trajectories',
        claimed_scope='fresh-input optional-refresh repair; example pair does not establish overall avoidance superiority',
        trials=[trial['metrics'] for trial in trials], gif=animation,
        report_module=dict(path=str(HERE / 'report_arm_box_navigation.py'), sha256=digest(HERE / 'report_arm_box_navigation.py')),
        render_script_sha256=digest(__file__), hardware_validation=False, input_hashes_verified_after_render=True)
    (OUT / 'summary.json').write_text(json.dumps(report.safe(summary), ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(output=str(OUT), files=['comparison.png', 'comparison.gif', 'comparison.html', 'summary.json']), ensure_ascii=False), flush=True)


def monitor():
    env = os.environ.copy()
    env.update(PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1', OPENBLAS_NUM_THREADS='1',
               OMP_NUM_THREADS='1', MALLOC_ARENA_MAX='2', MPLCONFIGDIR='/tmp/arm_box_refresh_report_mpl')
    started = time.monotonic()
    peak = 0
    aborted = None
    with (OUT / 'render_stdout.log').open('w') as log:
        proc = subprocess.Popen([sys.executable, __file__, '--worker'], env=env, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        try:
            while proc.poll() is None:
                try:
                    status = Path('/proc/%d/status' % proc.pid).read_text()
                    rss = next(int(line.split()[1]) * 1024 for line in status.splitlines() if line.startswith('VmRSS:'))
                    peak = max(peak, rss)
                    if rss > 200 * 1024**2:
                        aborted = 'render_worker_RSS_above_200MiB'
                        os.killpg(proc.pid, signal.SIGTERM)
                        break
                except (OSError, StopIteration):
                    pass
                time.sleep(.05)
            proc.wait(timeout=5)
        except BaseException:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
            raise
    stats = dict(peak_worker_rss_mib=peak / 1024**2, wall_s=time.monotonic() - started,
                 return_code=proc.returncode, abort_reason=aborted, limit_mib=200)
    (OUT / 'render_resources.json').write_text(json.dumps(stats, indent=2) + '\n')
    print(json.dumps(stats), flush=True)
    if aborted or proc.returncode:
        raise SystemExit('Render stopped; inspect render_stdout.log, preserving trial inputs.')


if __name__ == '__main__':
    (worker if '--worker' in sys.argv else monitor)()
