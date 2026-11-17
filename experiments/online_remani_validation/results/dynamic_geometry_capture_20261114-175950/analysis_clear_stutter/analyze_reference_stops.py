"""Offline native polynomial sampling; PYTHONNOUSERSITE=1, no large bag reads."""
import bisect
import json
import math
from pathlib import Path
import numpy as np
import rosbag
from scipy.ndimage import map_coordinates
from scipy.spatial import cKDTree
from analyze_occupancy import decode_array, local
from analyze_body_clearance import rows

HERE=Path(__file__).resolve().parent
CASE=HERE.parent


def evaluate(traj,phase):
    duration=sum(p['duration'] for p in traj['pieces'])
    t=max(0.,min(duration,float(phase)))
    for i,p in enumerate(traj['pieces']):
        if t<=p['duration'] or i==len(traj['pieces'])-1:
            break
        t-=p['duration']
    coef=np.asarray(p['data']).reshape((p['num_dim'],p['num_order']+1),order='F')
    pos=np.array([np.polyval(c,t) for c in coef])
    vel=np.array([np.polyval(np.polyder(c),t) for c in coef[:2]])
    yaw=math.atan2(traj['gear']*vel[1],traj['gear']*vel[0])
    return pos,vel,yaw


def main():
    trajs=rows('trajectories.jsonl')
    progress=rows('progress.jsonl')
    odom=sorted(rows('odom_full_pose.jsonl'),key=lambda x:x['source'])
    ostamps=[d['source'] for d in odom]
    stops=[d for d in rows('logs.jsonl') if 'stop before replanning' in d.get('msg','')]
    snapshots=rows('snapshot_prefix.jsonl')
    metadata=json.loads((HERE/'occupancy_metadata.json').read_text())['frames']
    reports=[]
    for stop in stops:
        now=stop['source']
        received=[t for t in trajs if t['bag']<=now]
        traj=next((t for t in reversed(received) if t['action']==1),None)
        assert traj is not None
        prog=max((p for p in progress if p['bag']<=now and p['source']<=now),key=lambda p:p['source'])
        age=now-prog['source']
        matches=abs(prog['start']-traj['source'])<1e-3 and prog['start']>0
        duration=sum(p['duration'] for p in traj['pieces'])
        phase=min(duration,prog['phase']+age*prog['scale']) if matches and age<=.5 else min(duration,now-traj['source'])
        k=bisect.bisect_right(ostamps,now)
        before,after=odom[k-1:k+1]
        alpha=(now-before['source'])/(after['source']-before['source'])
        x=before['x']+alpha*(after['x']-before['x'])
        y=before['y']+alpha*(after['y']-before['y'])
        yaw=before['yaw']+alpha*((after['yaw']-before['yaw']+math.pi)%(2*math.pi)-math.pi)
        pos,vel,reference_yaw=evaluate(traj,phase)
        near=min(metadata,key=lambda f:abs(f['acquisition']-now))
        near_delta=near['acquisition']-now
        out=dict(stop_source=now,stop_source_local=local(now),stop_receipt=stop['bag'],
            published_trajectory_start=traj['source'],trajectory_receipt=traj['bag'],gear=traj['gear'],
            progress=prog,progress_age_ms=age*1000,progress_matches=matches,
            inferred_fsm_phase=phase,phase_method='fresh matched progress' if matches and age<=.5 else 'wall-time fallback',
            wall_time_fallback_phase=min(duration,now-traj['source']),
            actual_pose_xyyaw=[x,y,yaw],reference_pose_xyyaw=[float(pos[0]),float(pos[1]),reference_yaw],
            actual_to_reference_xy_distance=float(np.linalg.norm(pos[:2]-[x,y])),
            reference_minus_actual_yaw=float((reference_yaw-yaw+math.pi)%(2*math.pi)-math.pi),
            reference_raw_speed=float(np.linalg.norm(vel)),
            nearest_diagnostic_index=near['index'],diagnostic_minus_stop_s=near_delta,
            geometric_replay_available=abs(near_delta)<.55,
            samples=[])
        previous=pos[:2]
        path=0.
        for step in range(11):
            p,v,a=evaluate(traj,phase+step*.05)
            path+=float(np.linalg.norm(p[:2]-previous))
            previous=p[:2]
            out['samples'].append(dict(ahead_phase_s=step*.05,phase=phase+step*.05,
                path_distance=path,xyyaw=[float(p[0]),float(p[1]),a],raw_speed=float(np.linalg.norm(v))))
        reports.append(out)
    needed={r['nearest_diagnostic_index'] for r in reports if r['geometric_replay_available']}
    with rosbag.Bag(str(CASE/'occupancy_evidence.bag')) as bag:
        for i,(_,raw,receipt) in enumerate(bag.read_messages(raw=True)):
            if i not in needed:continue
            a=decode_array(raw);h=a[:40];shape=h[5:8].astype(int);n=int(h[38])
            esdf=a[40:40+n].reshape(shape);codes=a[40+n:].reshape(shape).astype(np.uint8)
            occupied=h[8:11]+(np.argwhere(codes&1)+.5)*h[11]
            tree=cKDTree(occupied)
            matches=[s for s in snapshots if s['header'][14]==h[14]]
            body=np.asarray(min(matches,key=lambda s:abs(s['header'][24]-h[24]))['body'])
            body=body[body[:,6]==0]
            for report in reports:
                if not report['geometric_replay_available'] or report['nearest_diagnostic_index']!=i:continue
                report['map_field_version']=int(h[14]);report['map_fusion']=float(h[3])
                report['map_age_at_stop_ms']=1000*(report['stop_source']-h[3])
                for sample in report['samples']:
                    x,y,yaw=sample['xyyaw'];c,s=math.cos(yaw),math.sin(yaw)
                    centers=body[:,:3].copy();centers[:,:2]=centers[:,:2]@np.array([[c,s],[-s,c]])+[x,y]
                    idx=(centers-h[8:11])/h[11]-.5
                    inside=np.all((idx>=0)&(idx<shape-1),axis=1)
                    d=map_coordinates(esdf,idx.T,order=1,prefilter=False,mode='nearest')
                    raw_distance,raw_index=tree.query(centers)
                    gaps=d-body[:,3];raw_gaps=raw_distance-body[:,3]
                    worst=int(np.argmin(raw_gaps))
                    sample['complete_map_geometry']=dict(all_spheres_inside=bool(inside.all()),
                        min_esdf_hard_gap=float(gaps.min()),min_raw_center_hard_gap=float(raw_gaps.min()),
                        esdf_collision_count=int(np.count_nonzero(gaps<0)),raw_occupied_overlap_count=int(np.count_nonzero(raw_gaps<0)),
                        worst_raw_sphere=worst,world_sphere_center=centers[worst].tolist(),
                        nearest_raw_center_world=occupied[raw_index[worst]].tolist())
    output=dict(method=[
        'Native coefficients reshape with order=F into 8x8 and evaluate descending powers; gear multiplies derivative before atan2; piece boundaries use t<=duration.',
        'Fresh matched recorded progress is advanced by source age times scale, matching FSM formula; if stale or start mismatched, wall-time fallback is used.',
        'The replay uses latest bag-received progress as an approximation. It cannot prove the exact progress callback last consumed by the planner process.',
        'Only first 0.5 phase seconds in 0.05 steps are sampled; base fixed spheres are exact exported geometry. Arm future coefficients are not tested here.',
        'Nearest Complete diagnostic within 0.55 s is used. Map fusion and source differences are reported; it is not asserted to equal the FSM live occupancy generation.',
        'Raw nearest occupied checks include static cells, without static-match removal; absence of any overlap therefore also excludes overlap with a subset on this recorded Complete map.',
        'A collision-free replay on this map does not disprove a newer live-map collision or a different planner internal trajectory.'],stops=reports)
    (HERE/'reference_stop_replay.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps([dict(time=r['stop_source_local'],phase=r['inferred_fsm_phase'],xy_error=r['actual_to_reference_xy_distance'],
        yaw_error=r['reference_minus_actual_yaw'],speed=r['reference_raw_speed'],map_available=r['geometric_replay_available'],
        min_esdf_gap=min((s.get('complete_map_geometry',{}).get('min_esdf_hard_gap',float('inf')) for s in r['samples'])),
        min_raw_gap=min((s.get('complete_map_geometry',{}).get('min_raw_center_hard_gap',float('inf')) for s in r['samples']))) for r in reports],indent=2))


if __name__=='__main__':main()
