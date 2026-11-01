#!/usr/bin/env python3
"""Geometry/time observation on the recorded first adopted replan, not a solve."""
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT=Path(__file__).resolve().parent.parent


def check(trial):
    metadata_path=trial/'native3d_event_input.json'
    metadata=json.loads(metadata_path.read_text())
    path=trial/'native3d_event_input.npz'
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    if digest!=metadata['npz_sha256']:
        raise ValueError('Recorded event changed: '+str(path))
    if metadata['event']!='first_replan_reference_adoption':
        raise ValueError('This is not the declared adoption event: '+str(trial))
    with np.load(path,allow_pickle=False) as data:
        state=data['state'].copy();reference=data['reference'].copy()
        times=data['reference_times'].copy()
    if len(times)!=len(reference) or np.any(np.diff(times)<0):
        raise ValueError('Invalid recorded reference times')
    phase=metadata['cusp_debug'];begin,end=int(phase[3]),int(phase[4])
    best=None
    # Matches BaseMPPI::project without a hint: current measured gear-phase
    # bounds, point-to-segment distance, earliest index wins an exact tie.
    for index in range(begin,end):
        a,b=reference[index,:2],reference[index+1,:2]
        d=b-a;length2=float(d@d)
        if length2<1e-18:
            continue
        fraction=float(np.clip((state[:2]-a)@d/length2,0.,1.))
        point=a+fraction*d;distance2=float((state[:2]-point)@(state[:2]-point))
        if best is None or distance2<best[0]:
            best=(distance2,index,fraction,point)
    if best is None:
        raise ValueError('No nonzero segment in recorded active phase')
    distance2,index,fraction,point=best
    projected_time=float(times[index]+fraction*(times[index+1]-times[index]))
    wall_age=float(metadata['solve_ros_time']-metadata['guide_start'])
    clock_t=float(np.clip(wall_age,times[0],times[-1]))
    clock_xy=np.array([np.interp(clock_t,times,reference[:,axis]) for axis in (0,1)])
    return dict(trial=trial.name,event=metadata['event'],npz_sha256=digest,
        metadata_sha256=hashlib.sha256(metadata_path.read_bytes()).hexdigest(),
        solve_ros_time=metadata['solve_ros_time'],guide_start=metadata['guide_start'],
        original_reference_clock_age_s=wall_age,actual_solve_state=state.tolist(),
        phase_begin=begin,phase_end=end,nearest_reference_segment=index,
        nearest_reference_fraction=fraction,nearest_reference_xy=point.tolist(),
        measured_to_reference_start_m=float(np.linalg.norm(state[:2]-reference[0,:2])),
        measured_to_nearest_active_phase_m=float(np.sqrt(distance2)),
        nearest_reference_polynomial_time_s=projected_time,
        geometric_progress_minus_original_clock_s=projected_time-wall_age,
        original_clock_reference_xy=clock_xy.tolist(),
        measured_to_original_clock_reference_m=float(np.linalg.norm(state[:2]-clock_xy)),
        semantics='Recorded geometric association and piecewise-linear time interpolation only; '
                  'no MPPI invocation, no alternative controller execution, no collision certificate. '
                  'Difference between clocks alone does not prove an erroneous acceptance rejection.')


def main():
    plan=json.loads((ROOT/'plan.json').read_text())
    result=dict(method='Same recorded measured state, original full reference and measured gear-phase; '
                       'point-to-segment association matches native no-hint projection.',
                scope='First adopted replan only; rejected old q2 has no corresponding complete execution event.',
                trials=[check(ROOT/job['trial']) for job in plan['jobs']])
    (ROOT/'report/adoption_projection.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    for row in result['trials']:
        print(row['trial'],'start_gap_m',row['measured_to_reference_start_m'],
              'projection_gap_m',row['measured_to_nearest_active_phase_m'],
              'clock_age_s',row['original_reference_clock_age_s'],
              'projection_time_s',row['nearest_reference_polynomial_time_s'])


if __name__=='__main__':
    main()
