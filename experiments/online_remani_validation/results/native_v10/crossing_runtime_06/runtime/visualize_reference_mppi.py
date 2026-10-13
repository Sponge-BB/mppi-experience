#!/usr/bin/env python3
"""Read-only experiment replay: fixed reference vs recorded selected MPPI vs executed path.

This experiment has no REMANI online path. Do not fabricate or relabel one.
Outputs are new derived artifacts; no controller or ROS publisher is used.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np
from PIL import Image, ImageDraw, ImageFont

WIDTH, HEIGHT = 640, 500
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
REFERENCE, PREDICTION, EXECUTED = '#c17c18', '#21964b', '#226baf'
CASES = {'05_head_on_only': '单迎面', '04_four_dense': '四障碍'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def selected_at(record, t):
    """No old-plan holding, time interpolation or actual-future substitution."""
    tick = round(t/.05)
    snap = record['selected'].get(tick)
    if snap is None or abs(snap['t']-t) > 1e-8:
        return None
    return snap['points']


def load_record(path):
    r = json.loads(path.read_text())
    selected = {}; seen = 0; available = 0
    for snap in r.pop('searches'):
        cs = [c for c in snap['candidates'] if c['selected']]
        assert len(cs) <= 1
        points = None
        if cs:
            c = cs[0]; assert c['feasible'] and c['gap'] >= 0
            points = np.array(c['points'], dtype=float)
            assert points.shape[1] == 3 and np.isfinite(points).all()
            tick = round(snap['t']/.05)
            assert abs(r['trace'][tick]['t']-snap['t']) < 1e-8
            assert np.allclose(points[0, 1:], r['trace'][tick]['state'][:2], atol=1e-10, rtol=0)
            assert np.all(np.diff(points[:, 0]) > 0)
            if snap['t'] >= 2: assert points[-1, 0]+.04 <= r['config']['validity_s']+1e-8
            available += 1
        selected[round(snap['t']/.05)] = dict(t=snap['t'], points=points)
        seen += 1
    assert seen > 0, 'No sampled prediction records; do not invent MPPI curves'
    r['selected'] = selected
    r['overlay_audit'] = dict(snapshots=seen, selected_snapshots=available,
                              no_selected_snapshots=seen-available, exact_timestamp_only=True)
    return r


def frame(record, index, zoom=False):
    row = record['trace'][index]; terminal = index == len(record['trace'])-1
    state = row['next_state'] if terminal else row['state']
    t = row['t']+(.05 if terminal else 0.)
    pred = None if terminal else selected_at(record, row['t'])
    image = Image.new('RGB', (WIDTH, HEIGHT), 'white'); d = ImageDraw.Draw(image)
    title = ImageFont.truetype(FONT, 19); small = ImageFont.truetype(FONT, 16)
    name = {'base': '原速', 'fast': '提速', 'capacity': '仅提高上限'}[record['config']['profile']]
    h = .1*int(np.floor((record['config']['validity_s']-.05)/.1))
    d.text((18, 10), f'{CASES[record["case"]]} | {name} | 展开{h:.1f}s | t={t:.2f}s'+(' | 放大' if zoom else ''), font=title, fill='#333333')
    for x, label, color in ((18, '固定参考', REFERENCE), (208, 'MPPI预测', PREDICTION), (410, '实际轨迹', EXECUTED)):
        if color == REFERENCE:
            for k in range(3): d.line((x+10*k, 51, x+10*k+6, 51), fill=color, width=3)
        else: d.line((x, 51, x+27, 51), fill=color, width=3)
        d.text((x+35, 39), label, font=small, fill=color)
    d.text((18, 66), '参考线非REMANI在线规划输出；灰色为障碍物及CV预测', font=small, fill='#666666')
    layer = Image.new('RGB', image.size, 'white'); g = ImageDraw.Draw(layer)
    scale = 230 if zoom else 98
    def xy(x, y):
        return (300+(x-state[0])*scale, 275-(y-state[1])*scale) if zoom else (76+x*scale, 270-y*scale)
    for y in (-2, 2): g.line([xy(-1, y), xy(6, y)], fill='#e4e4e4')
    obs = np.array(row['obstacles'], dtype=float).reshape(-1, 6).copy()
    if terminal: obs[:, :2] += .05*obs[:, 2:4]
    for o in obs:
        cx, cy = xy(*o[:2]); end = xy(*(o[:2]+h*o[2:4])); radius = scale*o[4]
        g.line([(cx, cy), end], fill='#999999', width=2)
        angle = math.atan2(end[1]-cy, end[0]-cx)
        g.line([(end[0]-7*math.cos(angle-.5), end[1]-7*math.sin(angle-.5)), end,
                (end[0]-7*math.cos(angle+.5), end[1]-7*math.sin(angle+.5))], fill='#999999', width=2)
        g.ellipse((cx-radius, cy-radius, cx+radius, cy+radius), fill='#b7b7b7')
    for bx, by, radius in record['body']:
        x = state[0]+math.cos(state[2])*bx-math.sin(state[2])*by
        y = state[1]+math.sin(state[2])*bx+math.cos(state[2])*by
        cx, cy = xy(x, y); radius *= scale
        g.ellipse((cx-radius, cy-radius, cx+radius, cy+radius), outline='#93a8bd', width=1)
    # The frozen fixture is exactly the x=0..3, y=0 polyline; reconstruct it,
    # do not call it a stored REMANI trajectory. Dashes remain visible on blue.
    executed = [xy(*r['state'][:2]) for r in record['trace'][:index+1]]+[xy(*state[:2])]
    if len(executed) > 1: g.line(executed, fill=EXECUTED, width=3)
    for x in np.arange(0, 3, .10): g.line([xy(x, 0), xy(min(3, x+.06), 0)], fill=REFERENCE, width=2)
    if pred is not None:
        g.line([xy(p[1], p[2]) for p in pred], fill=PREDICTION, width=4)
        px, py = xy(*pred[-1, 1:]); g.ellipse((px-4, py-4, px+4, py+4), fill=PREDICTION)
    cx, cy = xy(*state[:2]); g.ellipse((cx-4, cy-4, cx+4, cy+4), fill=EXECUTED)
    g.line([(cx, cy), xy(state[0]+.15*math.cos(state[2]), state[1]+.15*math.sin(state[2]))], fill=EXECUTED, width=2)
    gx, gy = xy(3, 0); g.ellipse((gx-4, gy-4, gx+4, gy+4), fill='#333333')
    image.paste(layer.crop((16, 96, WIDTH-16, 421)), (16, 96)); d = ImageDraw.Draw(image)
    d.rectangle((16, 96, WIDTH-16, 421), outline='#dddddd')
    note = ('结束：'+record['metrics']['termination']+'；不显示旧预测') if terminal else (
        f'本帧已记录MPPI预测：{pred[-1,0]:.1f}s' if pred is not None else '本帧没有已记录的选中预测（不补画）')
    d.text((18, 427), note, font=small, fill='#444444')
    d.text((18, 452), f'v={state[3]:.2f}m/s | 净距={row["gap"]:.3f}m | seed 1 | 2倍速', font=small, fill='#555555')
    d.text((18, 476), '仅重绘原记录；预测记录5Hz；没有重跑控制器', font=small, fill='#777777')
    return image


def animation(path, records, zoom=False):
    count = max(len(r['trace']) for r in records)
    indices = list(range(0, count, 4))
    if indices[-1] != count-1: indices.append(count-1)
    frames = []
    for k in indices:
        canvas = Image.new('RGB', (WIDTH*len(records), HEIGHT), 'white')
        for n, r in enumerate(records): canvas.paste(frame(r, min(k, len(r['trace'])-1), zoom), (WIDTH*n, 0))
        frames.append(canvas.convert('P', palette=Image.ADAPTIVE, colors=128))
    frames[0].save(path, save_all=True, append_images=frames[1:], duration=100, loop=0, disposal=2)
    with Image.open(path) as im:
        assert im.n_frames > 1; im.seek(im.n_frames-1); im.load()


def main():
    p = argparse.ArgumentParser(); p.add_argument('--experiment', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True); a = p.parse_args()
    root = a.experiment.resolve(); out = a.out.resolve()
    manifest = json.loads((root/'manifest.json').read_text())
    original_audit = json.loads((root/'report_audit.json').read_text())
    source = root/'source_snapshot/suite_multi_dynamic.py'
    assert sha(source) == manifest['snapshots'][source.name]
    assert 'x = np.linspace(0, 3, 301)' in source.read_text()
    assert 'np.column_stack((x, x*0, x*0, np.minimum(reference_speed, .8*(3-x))))' in source.read_text()
    before = {name: sha(root/name) for name in original_audit['record_sha256']}
    assert before == original_audit['record_sha256']
    out.mkdir(parents=True, exist_ok=False)
    shutil.copy2(Path(__file__), out/Path(__file__).name)
    # Preserve explicit provenance rather than pretending this was published by REMANI.
    np.savetxt(out/'fixed_reference.csv', np.column_stack((np.linspace(0, 3, 301), np.zeros(301))),
               delimiter=',', header='x_m,y_m', comments='')
    checks = {}; lines = ['# MPPI预测 / 参考路径 / 实际轨迹', '',
        '橙色虚线：固定参考路径；绿色：本帧记录的选中MPPI预测；蓝色：已执行轨迹。灰色：障碍物及CV预测。', '',
        '**上一轮没有运行REMANI在线规划器，所以不存在可回放的REMANI在线路径。橙线是脚本生成的固定参考，不能作为REMANI重规划结果。**', '',
        '只重绘原实验seed 1；不重跑算法，不更改参数、成绩、原始JSON或旧GIF。',
        '预测记录为5Hz，显示时严格匹配记录时间；缺失、无选中候选及终止帧不补画，不用实际未来运动冒充MPPI预测。', '',
        '## 时域对照', '']
    for case in CASES:
        records = {}
        for key, cfg in manifest['configs'].items():
            if case not in cfg['cases']: continue
            r = load_record(root/key/case/'data/seed1.json'); records[key] = r
            assert r['seed'] == 1 and r['config'] == cfg
            dest = out/key/case; dest.mkdir(parents=True)
            animation(dest/'paths.gif', [r])
            if key == 'base_h3': animation(dest/'paths_zoom.gif', [r], zoom=True)
            index = min(200, len(r['trace'])-1) # Fixed t=10s rule, or terminal if already stopped.
            frame(r, index).save(dest/'paths_snapshot.png')
            if key == 'base_h3': frame(r, index, zoom=True).save(dest/'paths_zoom.png')
            checks[f'{key}/{case}'] = r['overlay_audit']
            print('DRAW', key, case, flush=True)
        dest = out/'comparison'/case; dest.mkdir(parents=True)
        for profile in ('base', 'fast'):
            animation(dest/f'{profile}_horizons.gif', [records[f'{profile}_h{h}'] for h in (2,3,4)])
            lines.append(f'- {CASES[case]}，{"原速" if profile=="base" else "提速"}：[三时域对照](comparison/{case}/{profile}_horizons.gif)')
    lines += ['', '## 各配置单独查看', '']
    for key, cfg in manifest['configs'].items():
        for case in cfg['cases']:
            line = f'- {key} / {CASES[case]}：[轨迹GIF]({key}/{case}/paths.gif) · [t=10秒快照或终止帧]({key}/{case}/paths_snapshot.png)'
            if key == 'base_h3': line += f' · [局部放大GIF]({key}/{case}/paths_zoom.gif)'
            lines.append(line)
    lines += ['', '本次无法补出不存在的REMANI路径。要同时查看真正的REMANI在线重规划与MPPI，需要另跑完整在线实验并记录规划轨迹。', '']
    (out/'README.md').write_text('\n'.join(lines)+'\n')
    for name, h in before.items(): assert sha(root/name) == h
    gifs = list(out.rglob('*.gif'))
    (out/'audit.json').write_text(json.dumps(dict(
        source_experiment=str(root), records_unchanged=len(before), record_sha256=before,
        reference_kind='synthetic fixed reference, NOT REMANI online output', reference_source_sha256=sha(source),
        visualization_only=True, reran_controller=False, selected_prediction_checks=checks,
        readable_gifs=len(gifs), renderer_sha256=sha(Path(__file__))), indent=2)+'\n')
    print('COMPLETE', out, 'GIFs', len(gifs), flush=True)


if __name__ == '__main__':
    main()
