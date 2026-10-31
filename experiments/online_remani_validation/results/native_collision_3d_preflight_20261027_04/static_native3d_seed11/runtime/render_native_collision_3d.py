#!/usr/bin/env python3
"""Read-only OFF/ON native-3D comparison GIFs and independent 3-D truth audit.

Run only after every timed online trial has stopped. No controller replay,
invented search traces or 2-D-projection-as-3-D collision assertions.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from dynamic_scenarios import actors_at
from native_collision_runtime import sampled_truth_clearance
from native_visuals import SearchReplay
from render_coupled_objective import SourceReader, execution_row
from render_dual_timescale import Trial as BaseTrial, FONT, latest


class Trial(BaseTrial):
    def __init__(self, source, group):
        self.source = Path(source).resolve()
        self.reader = read = SourceReader(self.source)
        self.manifest = read.json('manifest.json', required=True)
        self.parameters = read.json('coupling_parameters.json', required=True)
        self.result = read.json('result.json', required=True)
        self.metadata = {}
        self.status = read.json('run_status.json', required=True)
        self.audit = read.json('trial_audit.json', required=True)
        self.state = read.csv('state.csv', required=True)
        if len(self.state) < 2 or not {'t', 'x', 'y', 'yaw'}.issubset(self.state.dtype.names or ()):
            raise ValueError('missing recorded measured poses')
        if not np.isfinite(np.column_stack([self.state[k] for k in ('t', 'x', 'y', 'yaw')])).all() or np.any(np.diff(self.state['t']) < 0):
            raise ValueError('invalid recorded measured poses')
        self.body_record = read.json('native_collision_3d_body.json', required=True)
        self.body3d = np.asarray(self.body_record['body'], dtype=float)
        self.resolution = float(self.body_record['header'][11])
        sampled_truth_clearance(np.zeros(5), self.body3d, self.resolution, [])
        self.geometry = read.json('geometry.json', required=True)
        self.refs = sorted(read.jsonl('consumed_references.jsonl'), key=lambda row: row['t'])
        self.ref_events = sorted(read.jsonl('reference_events.jsonl'), key=lambda row: row['t'])
        self.execution = sorted(read.jsonl('execution_rollouts.jsonl', execution_row), key=lambda row: row['t'])
        self.diagnostics = read.json('mppi_diagnostics.json', [])
        self.selected, self.control_decisions = [], []
        self.fixed_ref, self.offline = None, False
        self.search = SearchReplay(self.source)
        for _, path in self.search.entries:
            read.locate(str(path.relative_to(self.source)), required=True)
        self.search_json = []
        self.obs = read.csv('obstacles.csv')
        self.obs_json = []
        self.anchor = float(self.result.get('motion_anchor') or self.result.get('goal_sent') or self.state['t'][0])
        self.end = float(min(self.state['t'][-1], self.result.get('elapsed', self.state['t'][-1])))
        if self.end <= self.anchor:
            raise ValueError('no recorded interval after alignment anchor')
        self.group = group
        self.title = 'OFF：旧二维附加门' if group == 'control' else 'ON：原生三维碰撞'
        enabled = self.manifest.get('native_collision_3d', {}).get('enabled')
        if enabled != (group == 'native3d'):
            raise ValueError('recorded mode differs from requested panel')

    def verify_unchanged(self):
        self.reader.verify()


def truth_actors(trial, t):
    anchor = trial.result.get('motion_anchor') if trial.manifest.get('scenario_start') == 'motion' else trial.result.get('goal_sent')
    return actors_at(trial.manifest['scene'], t-float(anchor)) if anchor is not None else []


def truth_report(trial):
    gaps = [sampled_truth_clearance(np.array([row['x'], row['y'], row['yaw']]), trial.body3d,
                trial.resolution, truth_actors(trial, float(row['t']))) for row in trial.state]
    min_gap = float(min(gaps))
    monitor = trial.result.get('native3d_truth_monitor', {})
    return dict(trial=str(trial.source), group=trial.group, termination=trial.result['reason'],
        reached_goal=trial.result['reason'] == 'goal', motion_duration_s=trial.end-trial.anchor,
        native3d_independent_sample_collision=min_gap < 0,
        native3d_physical_sample_collision=bool(min_gap < 0 or monitor.get('sample_collision')),
        native3d_monitor_ready=bool(monitor.get('geometry_available') and monitor.get('checks', 0) > 0 and not monitor.get('error')),
        native3d_independent_min_physical_sample_gap_m=min_gap, native3d_state_samples=len(gaps),
        native3d_online_substep_monitor=monitor,
        projection_2d_overlap=trial.result.get('collision_conservative_projection'),
        projection_2d_min_gap_m=trial.result.get('min_clearance'),
        projection_is_not_native3d_contact=True,
        interface_audit_valid=trial.audit.get('valid_experiment'),
        scope='analytic native physical sphere samples vs finite cylinder/wall truth; discrete state sample audit, not mesh contact or continuous certificate')


def draw_panel(image, trial, t, rect):
    left, top, right, bottom = rect
    layer = Image.new('RGB', image.size, 'white')
    draw = ImageDraw.Draw(layer)
    scale = min((right-left)/6.6, (bottom-top)/4.6)

    def xy(x, y):
        return ((left+right)/2+(x-1.5)*scale, (top+bottom)/2-y*scale)

    draw.rectangle([xy(-1.5, 2.), xy(4.5, -2.)], outline='#c9c9c9', width=2)
    for ox, oy, radius, *_ in truth_actors(trial, t):
        px, py = xy(ox, oy)
        r = radius*scale
        draw.ellipse((px-r, py-r, px+r, py+r), fill='#999999', outline='#666666')
    reference = trial.reference_at(t)
    if reference is not None and len(reference) > 1:
        draw.line([xy(*point[:2]) for point in reference], fill='#d78224', width=3)
    row, index = trial.state_at(t)
    if index:
        draw.line([xy(x, y) for x, y in zip(trial.state['x'][:index+1], trial.state['y'][:index+1])], fill='#2874a6', width=3)
    c, s = math.cos(row['yaw']), math.sin(row['yaw'])
    for bx, by, bz, radius, _, _, kind in trial.body3d:
        px, py = xy(row['x']+c*bx-s*by, row['y']+s*bx+c*by)
        r = (radius-(trial.resolution if kind == 0 else 0))*scale
        draw.ellipse((px-r, py-r, px+r, py+r), fill='#e5edf4', outline='#9db0c0', width=1)
    length, width = trial.geometry['base_length']/2, trial.geometry['base_width']/2
    corners = [(-length, -width), (length, -width), (length, width), (-length, width), (-length, -width)]
    draw.line([xy(row['x']+c*x-s*y, row['y']+s*x+c*y) for x, y in corners], fill='#2874a6', width=2)
    draw.line([xy(row['x'], row['y']), xy(row['x']+.27*c, row['y']+.27*s)], fill='#164a73', width=3)
    # Real search subset only. Gray traces never imply execution approval.
    search = trial.search_at(t)
    if search is not None:
        rows = np.asarray(search['rows'])
        if rows.ndim == 2 and len(rows):
            for ident in np.unique(rows[:, 0]):
                path = rows[rows[:, 0] == ident]
                if len(path) > 1:
                    draw.line([xy(x, y) for x, y in path[:, 2:4]], fill='#b5b5b5', width=1)
    selected = trial.selected_at(t)
    if selected is not None:
        draw.line([xy(*point[:2]) for point in selected], fill='#258446', width=3)
    image.paste(layer.crop(rect), (left, top))
    ImageDraw.Draw(image).rectangle(rect, outline='#dddddd')
    return row


def render_pair(control, treatment, out, max_frames=160):
    trials = [Trial(control, 'control'), Trial(treatment, 'native3d')]
    if len({t.manifest['scene'] for t in trials}) != 1 or len({t.parameters['seed'] for t in trials}) != 1:
        raise ValueError('comparison requires same scene and seed')
    if not np.array_equal(trials[0].body3d, trials[1].body3d) or trials[0].resolution != trials[1].resolution:
        raise ValueError('native physical sample geometry differs between groups')
    out = Path(out).resolve()
    if any(out == trial.source or trial.source in out.parents for trial in trials):
        raise ValueError('renderer output must not modify source trials')
    out.mkdir(parents=True, exist_ok=False)
    duration = max(t.end-t.anchor for t in trials)
    ages = np.linspace(0., duration, min(max_frames, int(math.ceil(duration/.3))+1))
    step = float(ages[1]-ages[0])
    frame_ms = max(20, 10*round(1000*step/2./10))
    font = ImageFont.truetype(FONT, 18)
    small = ImageFont.truetype(FONT, 14)
    frames = []
    for age in ages:
        image = Image.new('RGB', (1160, 482), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((16, 8), '在线实测  %.1f s　|　同运动起点对齐' % age, font=font, fill='#333333')
        draw.text((16, 34), '橙：REMANI参考　蓝：实际底盘/固定臂投影　灰细线：真实MPPI候选子集　绿：获准展开', font=small, fill='#555555')
        for index, trial in enumerate(trials):
            t = min(trial.anchor+age, trial.end)
            draw.text((index*580+16, 64), trial.title, font=font, fill='#333333')
            row = draw_panel(image, trial, t, (index*580+16, 98, (index+1)*580-16, 404))
            draw = ImageDraw.Draw(image)
            draw.text((index*580+16, 414), '%s　|　v=%+.2f m/s' % (trial.status_at(t), row['v']), font=small, fill='#555555')
        draw.text((16, 452), '固定臂差速数值仿真，非实车。画图投影不是三维碰撞判据；结束后不续画未来轨迹。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    frames[0].save(out/'comparison.gif', save_all=True, append_images=frames[1:],
        duration=[frame_ms]*(len(frames)-1)+[max(500, frame_ms)], loop=0, disposal=2, optimize=True)
    frames[-1].convert('RGB').save(out/'paths.png')
    reports = [truth_report(trial) for trial in trials]
    for trial in trials:
        trial.verify_unchanged()
    value = dict(scene=trials[0].manifest['scene'], seed=trials[0].parameters['seed'], trials=reports,
        alignment='recorded motion_anchor, fallback goal_sent; panels stop at actual termination',
        source_sha256={trial.group: trial.reader.hashes for trial in trials},
        frames=len(frames), no_controller_reexecution=True, real_search_subset_only=True,
        statistical_or_causal_superiority_claim=False)
    (out/'report.json').write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    text = ['# 原生三维碰撞 OFF/ON 对照', '', '仅单次同种子数值闭环，不是统计结论或实车验收。', '',
        '| 组 | 结束原因 | 运动时长/s | 独立3D采样净距/m | 旧2D投影最小净距/m |', '|---|---|---:|---:|---:|']
    for report in reports:
        text.append('| %s | %s | %.2f | %.4f | %.4f |' % (report['group'], report['termination'], report['motion_duration_s'],
            report['native3d_independent_min_physical_sample_gap_m'], report['projection_2d_min_gap_m']))
    text += ['', '3D复核使用原生MM实际固定臂/底盘球采样与有限高真值圆柱、墙面；底盘半径扣除原生地图分辨率保守量。它不是完整网格接触或连续安全证明。旧2D投影相交不等于3D物理接触。失败原样保留。']
    (out/'README.md').write_text('\n'.join(text)+'\n')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    batch = args.batch.resolve()
    if not (batch/'complete.json').is_file():
        raise ValueError('timed batch must finish before rendering')
    plan = json.loads((batch/'plan.json').read_text())
    args.out.mkdir(parents=True, exist_ok=False)
    results = []
    for scene in plan['scenes']:
        name = '%s_%%s_seed%d' % (scene, plan['seed'])
        results.append(render_pair(batch/(name % 'control'), batch/(name % 'native3d'), args.out/scene))
    (args.out/'report.json').write_text(json.dumps(results, ensure_ascii=False, indent=2)+'\n')


if __name__ == '__main__':
    main()
