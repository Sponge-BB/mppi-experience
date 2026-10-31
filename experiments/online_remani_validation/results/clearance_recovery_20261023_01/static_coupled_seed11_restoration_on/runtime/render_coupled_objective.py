#!/usr/bin/env python3
"""Read-only two-panel replay of native / coupled online numerical trials.

Reuses render_dual_timescale drawing and recorded-reference/command lease
semantics. JSON/JSONL archives are streamed directly; no extracted files, ROS,
controller replay, search reconstruction or replacement robot geometry.
Run AFTER timed trials, never concurrently with measured planning.
"""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import render_dual_timescale as upstream


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


class SourceReader:
    """Hashes the ACTUAL compressed/plain source file consumed by each read."""
    def __init__(self, source):
        self.source = Path(source).resolve()
        self.hashes = {}

    def locate(self, name, required=False):
        relative = Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('source reads must remain within trial directory')
        candidates = [self.source / name, self.source / (name + '.gz')]
        paths = [p for p in candidates if p.is_file()]
        if len(paths) > 1:
            raise ValueError('ambiguous plain/archive duplicates: ' + name)
        if not paths:
            if required:
                raise ValueError('missing recorded source: ' + name)
            return None
        path = paths[0]
        if path.is_symlink():
            raise ValueError('symlinked source not accepted: ' + name)
        self.hashes[path.relative_to(self.source).as_posix()] = sha(path)
        return path

    def open(self, path):
        return gzip.open(path, 'rt', encoding='utf8') if path.suffix == '.gz' else path.open(encoding='utf8')

    def json(self, name, default=None, required=False):
        path = self.locate(name, required)
        if path is None:
            return default
        with self.open(path) as stream:
            return json.load(stream)

    def jsonl(self, name, transform=None):
        path = self.locate(name)
        if path is None:
            return []
        rows = []
        with self.open(path) as stream:
            for line in stream:
                if line.strip():
                    value = json.loads(line)
                    rows.append(transform(value) if transform else value)
        return rows

    def csv(self, name, required=False, matrix=False):
        path = self.locate(name, required)
        if path is None:
            return np.empty(0, dtype=[('t', float)])
        with self.open(path) as stream:
            if matrix:
                return np.atleast_2d(np.loadtxt(stream, delimiter=','))
            return np.atleast_1d(np.genfromtxt(stream, names=True, delimiter=',', encoding='utf8'))

    def verify(self):
        for relative, digest in self.hashes.items():
            if sha(self.source / relative) != digest:
                raise RuntimeError('source changed during visualization: ' + relative)


def keep(row, keys):
    return {key: row[key] for key in keys if key in row}


def thin_path(path, stride=5, columns=None):
    points = np.asarray(path, dtype=float)
    if points.ndim != 2 or not len(points):
        return np.empty((0, 4 if columns is None else columns))
    indices = np.unique(np.r_[np.arange(0, len(points), stride), len(points)-1])
    return np.ascontiguousarray(points[indices, :columns]) if columns else np.ascontiguousarray(points[indices])


def execution_row(row):
    out = keep(row, ('t', 'reason', 'lease_s', 'guide_revision'))
    # Only display samples of the actual approved rollout. No new optimization,
    # interpolation, trajectory extension or inferred search candidates.
    out['rollout'] = thin_path(row.get('rollout', []), columns=4)
    return out


def diagnostic_row(row):
    return keep(row, ('t', 'reason', 'observed_obstacle_age', 'observed_obstacles'))


class Trial(upstream.Trial):
    """Lean archive-aware storage; inherited display rules remain unchanged."""
    def __init__(self, source, group):
        self.source = Path(source).resolve()
        self.reader = SourceReader(self.source)
        read = self.reader
        self.parameters = read.json('coupling_parameters.json', required=True)
        if self.parameters.get('group') != group:
            raise ValueError('declared panel group differs from recorded treatment')
        self.manifest = read.json('manifest.json', required=True)
        self.status = read.json('run_status.json', required=True)
        self.audit = read.json('trial_audit.json', required=True)
        if not self.status.get('result_written'):
            raise ValueError('trial has no final result; finish timing before visualization')
        self.state = read.csv('state.csv', required=True)
        if len(self.state) < 2 or not {'t', 'x', 'y', 'yaw'}.issubset(self.state.dtype.names or ()):
            raise ValueError('at least two measured robot poses are required')
        if not np.isfinite(np.column_stack([self.state[k] for k in ('t', 'x', 'y', 'yaw')])).all() or np.any(np.diff(self.state['t']) < 0):
            raise ValueError('invalid measured state timestamps/coordinates')
        self.result = read.json('result.json', required=True)
        self.metadata = read.json('metadata.json', {})
        self.offline = False
        self.fixed_ref = None
        self.title = '原串联 REMANI＋MPPI' if group == 'native' else '两项耦合（含工程底座）'
        anchor = self.result.get('motion_anchor')
        if anchor is None:
            anchor = self.result.get('goal_sent', self.state['t'][0])
        self.anchor = float(anchor)
        self.end = float(min(self.state['t'][-1], self.result.get('elapsed', self.state['t'][-1])))
        if not math.isfinite(self.anchor) or self.end <= self.anchor:
            raise ValueError('no recorded post-anchor motion interval')
        self.body = read.csv('geometry.csv', required=True, matrix=True)
        self.geometry = read.json('geometry.json', required=True)
        if self.body.shape[1] != 3 or not len(self.body) or not np.isfinite(self.body).all() or np.any(self.body[:, 2] <= 0):
            raise ValueError('complete fixed-arm body geometry is required; point fallback forbidden')
        if not np.allclose(self.body, self.geometry.get('body_disks_xy_r', []), rtol=0., atol=1e-12):
            raise ValueError('geometry CSV/JSON mismatch')
        self.refs = sorted(read.jsonl('consumed_references.jsonl',
            lambda row: keep(row, ('t', 'revision', 'reference', 'epoch'))), key=lambda row: row['t'])
        self.ref_events = sorted(read.jsonl('reference_events.jsonl',
            lambda row: keep(row, ('t', 'event', 'revision'))), key=lambda row: row['t'])
        self.diagnostics = [diagnostic_row(row) for row in read.json('mppi_diagnostics.json', [])]
        self.diagnostics += read.jsonl('diagnostics.jsonl', diagnostic_row)
        self.diagnostics.sort(key=lambda row: row['t'])
        self.execution = sorted(read.jsonl('execution_rollouts.jsonl', execution_row), key=lambda row: row['t'])
        self.selected = []  # This online runner records publication-approved execution_rollouts.
        self.control_decisions = []
        self.search_json = []
        self.search = None
        if any((self.source/name).exists() for name in ('search_trace.jsonl','search_trace.jsonl.gz')) or next((self.source/'search').glob('*.npz'), None) is not None:
            raise ValueError('this wrapper explicitly expects trials WITHOUT search recording')
        self.obs = read.csv('obstacles.csv')
        self.obs_json = sorted(read.jsonl('obstacles.jsonl'), key=lambda row: row['t'])
        self.hashes = read.hashes
        self.group = group

    def search_at(self, t):
        return None  # This experiment did not enable search recording.

    def verify_unchanged(self):
        self.reader.verify()


def render(sources, out, step=.3, playback=2., max_frames=160, max_mib=10.):
    if len(sources) != 2 or not .05 <= step <= 2. or not 0. < playback <= 20.:
        raise ValueError('need two sources and bounded step/playback')
    if not 2 <= max_frames <= 240 or not .5 <= max_mib <= 30.:
        raise ValueError('invalid frame/output bound')
    trials = [Trial(sources[0], 'native'), Trial(sources[1], 'coupled')]
    scenes = {trial.result.get('scene', trial.manifest.get('scene')) for trial in trials}
    seeds = {trial.parameters.get('seed') for trial in trials}
    if len(scenes) != 1 or None in scenes or len(seeds) != 1 or None in seeds:
        raise ValueError('panels must use the same declared scene and seed')
    if trials[0].body.shape != trials[1].body.shape or not np.array_equal(trials[0].body, trials[1].body):
        raise ValueError('panel robot collision projections differ')
    out = Path(out).resolve()
    if any(out == t.source or t.source in out.parents for t in trials):
        raise ValueError('output must not modify a source trial directory')
    out.mkdir(parents=True, exist_ok=False)
    duration = max(t.end-t.anchor for t in trials)
    count = min(max_frames, int(math.ceil(duration/step))+1)
    ages = np.linspace(0., duration, count)
    effective_step = float(ages[1]-ages[0])
    frame_ms = max(20, 10*round(1000*effective_step/playback/10))
    limits = upstream.bounds(trials)
    panel_width, height = 540, 454
    font = ImageFont.truetype(upstream.FONT, 18)
    small = ImageFont.truetype(upstream.FONT, 14)
    frames, final = [], None
    for age in ages:
        image = Image.new('RGB', (2*panel_width, height), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((16, 7), '实测仿真记录  %.1f s  |  %.2g倍播放' % (age, 1000*effective_step/frame_ms), font=font, fill='#333333')
        draw.text((16, 33), '橙：已消费REMANI参考   蓝：实际路线与完整投影   绿：获准MPPI轨迹', font=small, fill='#444444')
        draw.text((16, 54), '搜索候选未记录；不补画。灰箭头仅示意1秒估计运动。', font=small, fill='#666666')
        for index, trial in enumerate(trials):
            t = min(trial.anchor+float(age), trial.end)
            draw.text((index*panel_width+16, 79), trial.title, font=font, fill='#333333')
            row, _ = upstream.draw_panel(image, trial, t,
                (index*panel_width+16, 108, (index+1)*panel_width-16, 363), limits, show_search=False)
            draw = ImageDraw.Draw(image)
            speed = float(row['v']) if 'v' in row.dtype.names else float('nan')
            draw.text((index*panel_width+16, 374), '实际速度 %+.2f m/s' % speed, font=small, fill='#444444')
            draw.text((index*panel_width+16, 396), trial.status_at(t)[:49], font=small, fill='#555555')
        draw.text((16, 428), '固定机械臂 / 在线数值仿真，非实车；结束后不延长轨迹，不补画过期指令。', font=small, fill='#777777')
        final = image
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    durations = [frame_ms]*len(frames)
    durations[-1] = max(frame_ms, 500)
    target = out/'overview.gif'
    frames[0].save(target, save_all=True, append_images=frames[1:], duration=durations, loop=0, disposal=2, optimize=True)
    final.save(out/'paths.png', optimize=True)
    for trial in trials:
        trial.verify_unchanged()
    size = target.stat().st_size
    audit = dict(scene=next(iter(scenes)), seed=next(iter(seeds)),
        visualization_only=True, source_unchanged=True, no_controller_reexecution=True,
        no_future_reference=True, no_missing_rollout_fabricated=True,
        no_search_recorded=True, no_search_candidates_drawn=True,
        approved_rollout_display_stride=5, source_archives_read_without_extraction=True,
        scope='fixed-arm native ROS numerical SIL, not Gazebo or real vehicle',
        comparison_note='coupled includes engineering geometry baseline; this pair alone cannot isolate coupling-item effects',
        alignment='each recorded motion_anchor, else goal_sent; panels stop at actual recorded termination',
        arrow_definition='one-second measured CV motion illustration, not complete prediction horizon',
        frame_count=len(frames), requested_sample_sim_s=step, effective_sample_sim_s=effective_step,
        regular_frame_ms=frame_ms, terminal_hold_ms=durations[-1],
        gif_bytes=size, max_gif_bytes=int(max_mib*1024**2), output_size_within_bound=size<=max_mib*1024**2,
        output_sha256={name:sha(out/name) for name in ('overview.gif', 'paths.png')},
        code_sha256={str(Path(__file__).resolve()):sha(Path(__file__)),
                     str(Path(upstream.__file__).resolve()):sha(Path(upstream.__file__)),
                     str(Path(upstream.__file__).with_name('native_visuals.py').resolve()):sha(Path(upstream.__file__).with_name('native_visuals.py'))},
        sources=[dict(path=str(t.source), group=t.group, input_sha256=t.hashes,
                      body_disks_xy_r=t.body.tolist(), result=t.result,
                      valid_experiment=t.audit.get('valid_experiment')) for t in trials])
    (out/'visualization_audit.json').write_text(json.dumps(audit, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    if not audit['output_size_within_bound']:
        raise RuntimeError('GIF exceeds declared output bound; retained with audit, rerun to NEW directory using fewer frames')
    return audit


def self_test():
    # Small input-only test; deliberately produces no GIF or experiment output.
    import tempfile
    with tempfile.TemporaryDirectory(prefix='coupling_render_reader_') as directory:
        root = Path(directory)
        with gzip.open(root/'events.jsonl.gz', 'wt', encoding='utf8') as stream:
            stream.write(json.dumps(dict(t=1., reason='ok', rollout=[[0.,0.,0.,0.],[.1,1.,0.,0.]]))+'\n')
        with gzip.open(root/'result.json.gz', 'wt', encoding='utf8') as stream:
            json.dump(dict(reason='goal'), stream)
        reader = SourceReader(root)
        assert reader.json('result.json')['reason'] == 'goal'
        rows = reader.jsonl('events.jsonl', execution_row)
        assert rows[0]['rollout'].shape == (2,4)
        assert set(reader.hashes) == {'result.json.gz','events.jsonl.gz'}
        reader.verify()
        (root/'result.json').write_text('{}')
        try:
            reader.json('result.json')
        except ValueError:
            pass
        else:
            raise AssertionError('ambiguous plain/archive sources were silently accepted')
        # Exercise full inherited reference-adoption and command-lease rules
        # against actual compressed records, still without drawing a GIF.
        trial_root=root/'trial';trial_root.mkdir()
        documents={
            'coupling_parameters.json':dict(group='native',seed=1),
            'manifest.json':dict(scene='static'),
            'run_status.json':dict(result_written=True),
            'trial_audit.json':dict(valid_experiment=True),
            'result.json':dict(reason='goal',motion_anchor=1.,elapsed=2.,scene='static'),
            'geometry.json':dict(body_disks_xy_r=[[0.,0.,.2]])}
        for name,value in documents.items():
            with gzip.open(trial_root/(name+'.gz'),'wt',encoding='utf8') as stream:
                json.dump(value,stream)
        (trial_root/'state.csv').write_text('t,x,y,yaw,v\n1,0,0,0,0\n2,1,0,0,0\n')
        (trial_root/'geometry.csv').write_text('0,0,.2\n')
        records={
            'consumed_references.jsonl':[dict(t=1.1,revision=1,reference=[[0.,0.],[1.,0.]])],
            'reference_events.jsonl':[dict(t=1.,revision=1,event='replace'),dict(t=1.3,revision=2,event='cancel')],
            'execution_rollouts.jsonl':[dict(t=1.15,reason='ok',lease_s=.075,
                rollout=[[0.,0.,0.,0.],[.1,1.,0.,0.]])]}
        for name,values in records.items():
            with gzip.open(trial_root/(name+'.gz'),'wt',encoding='utf8') as stream:
                for value in values:
                    stream.write(json.dumps(value)+'\n')
        trial=Trial(trial_root,'native')
        assert trial.reference_at(1.05) is None
        assert trial.reference_at(1.15) is not None
        assert trial.reference_at(1.31) is None
        assert trial.selected_at(1.16).shape==(2,3)
        assert trial.selected_at(1.226) is None
        assert 'execution_rollouts.jsonl.gz' in trial.hashes
        assert 'execution_rollouts.jsonl' not in trial.hashes
        trial.verify_unchanged()
    print('COUPLING_RENDER_READER_TEST_PASSED; no GIF generated')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native', type=Path)
    parser.add_argument('--coupled', type=Path)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--step', type=float, default=.3)
    parser.add_argument('--playback', type=float, default=2.)
    parser.add_argument('--max-frames', type=int, default=160)
    parser.add_argument('--max-mib', type=float, default=10.)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.native or not args.coupled or not args.out:
        parser.error('--native, --coupled and a NEW --out directory are required')
    audit = render([args.native,args.coupled], args.out, args.step, args.playback, args.max_frames, args.max_mib)
    print(json.dumps({key:audit[key] for key in ('scene','seed','frame_count','gif_bytes','output_size_within_bound')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
