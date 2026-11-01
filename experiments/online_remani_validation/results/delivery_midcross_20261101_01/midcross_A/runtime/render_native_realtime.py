#!/usr/bin/env python3
"""Small recorded original/demand GIFs; both panels are native 3-D models."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from render_dual_timescale import FONT
from render_native_collision_3d import draw_panel
from report_native_realtime import Trial, summarize_trial


def render_pair(original, demand, out, max_frames=160):
    trials = [Trial(original, 'original'), Trial(demand, 'demand')]
    if len({trial.manifest['scene'] for trial in trials}) != 1:
        raise ValueError('panels must use same declared scene')
    if not np.array_equal(trials[0].body3d, trials[1].body3d):
        raise ValueError('native3D body geometry differs')
    out = Path(out).resolve()
    if any(out == trial.source or trial.source in out.parents for trial in trials):
        raise ValueError('cannot alter measured source trials')
    out.mkdir(parents=True, exist_ok=False)
    # A stationary baseline has no motion anchor. In that case BOTH panels
    # align to actual goal publication, clearly labelled; no virtual obstacle.
    by_motion = all(trial.result.get('motion_anchor') is not None for trial in trials)
    if not by_motion:
        for trial in trials:
            trial.anchor = float(trial.result.get('goal_sent') or trial.state['t'][0])
    duration = max(trial.end-trial.anchor for trial in trials)
    ages = np.linspace(0., duration, min(max_frames, int(math.ceil(duration/.3))+1))
    step = float(ages[1]-ages[0])
    frame_ms = max(20, 10*round(1000*step/2./10))
    font, small = ImageFont.truetype(FONT, 18), ImageFont.truetype(FONT, 14)
    frames = []
    for age in ages:
        image = Image.new('RGB', (1160, 482), 'white')
        draw = ImageDraw.Draw(image)
        draw.text((16, 8), '记录时间 %.1f s　|　%s' % (age, '各自实测运动起点对齐' if by_motion else '双方目标发布时间对齐（原组无运动起点）'), font=font, fill='#333333')
        draw.text((16, 34), '橙：真实REMANI参考　蓝：实际底盘/固定臂投影　灰细线：真实候选子集　绿：获准展开', font=small, fill='#555555')
        for index, trial in enumerate(trials):
            t = min(trial.anchor+age, trial.end)
            draw.text((index*580+16, 64), trial.title, font=font, fill='#333333')
            row = draw_panel(image, trial, t, (index*580+16, 98, (index+1)*580-16, 404))
            draw = ImageDraw.Draw(image)
            scene = trial.result.get('native_realtime_evidence', {})
            appeared = scene.get('first_truth_actor_t')
            label = '障碍未触发' if appeared is None or t < appeared else '障碍已真实出现'
            draw.text((index*580+16, 414), '%s　|　%s　v=%+.2f' % (trial.status_at(t), label, row['v']), font=small, fill='#555555')
        draw.text((16, 452), '两组均原生三维碰撞。按真实运动触发障碍；早停/未触发不算避障成功。数值仿真，非实车。', font=small, fill='#777777')
        frames.append(image.convert('P', palette=Image.ADAPTIVE, colors=96))
    frames[0].save(out/'comparison.gif', save_all=True, append_images=frames[1:],
        duration=[frame_ms]*(len(frames)-1)+[max(500, frame_ms)], loop=0, disposal=2, optimize=True)
    frames[-1].convert('RGB').save(out/'paths.png')
    for trial in trials:
        trial.verify_unchanged()
    record = dict(scene=trials[0].manifest['scene'], both_native_collision_3d=True,
        alignment='motion_anchor' if by_motion else 'both goal_sent because a group never triggered motion',
        no_future_actor_fabrication=True, no_search_reconstruction=True, actual_recorded_subset_only=True,
        source_sha256={trial.group: trial.reader.hashes for trial in trials}, frames=len(frames),
        trials=[summarize_trial(original, 'original'), summarize_trial(demand, 'demand')])
    (out/'report.json').write_text(json.dumps(record, ensure_ascii=False, indent=2)+'\n')
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    batch = args.batch.resolve()
    if not (batch/'complete.json').is_file():
        raise ValueError('complete timed trials before rendering')
    plan = json.loads((batch/'plan.json').read_text())
    args.out.mkdir(parents=True, exist_ok=False)
    for scene in plan['scenes']:
        name = '%s_%%s_seed%d' % (scene, plan['seed'])
        render_pair(batch/(name % 'original'), batch/(name % 'demand'), args.out/scene)


if __name__ == '__main__':
    main()
