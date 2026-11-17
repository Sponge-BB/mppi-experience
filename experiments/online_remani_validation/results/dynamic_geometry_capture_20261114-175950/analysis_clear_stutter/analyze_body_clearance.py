"""Join small pose/body records to diagnostics; run with PYTHONNOUSERSITE=1."""
import bisect
import json
import math
from pathlib import Path
import numpy as np
import rosbag
from scipy.ndimage import map_coordinates
from scipy.spatial import cKDTree
from analyze_occupancy import decode_array, local

HERE = Path(__file__).resolve().parent
CASE = HERE.parent


def rows(name):
    with (HERE/name).open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def main():
    snapshots = rows('snapshot_prefix.jsonl')
    odom = sorted(rows('odom_pose.jsonl'), key=lambda x:x['source'])
    stamps = [p['source'] for p in odom]
    reports = []
    with rosbag.Bag(str(CASE/'occupancy_evidence.bag')) as bag:
        for i, (_, raw, receipt) in enumerate(bag.read_messages(raw=True)):
            a = decode_array(raw)
            h = a[:40]
            candidates = [s for s in snapshots if s['header'][14] == h[14]]
            assert candidates, 'No matching field body prefix'
            snap = min(candidates,key=lambda s:abs(s['header'][24]-h[24]))
            body = np.asarray(snap['body'])
            j = bisect.bisect_right(stamps,h[37])
            assert 0 < j < len(odom), 'Acquisition not bracketed by odometry'
            before, after = odom[j-1], odom[j]
            fraction = (h[37]-before['source'])/(after['source']-before['source'])
            x = before['x'] + fraction*(after['x']-before['x'])
            y = before['y'] + fraction*(after['y']-before['y'])
            dyaw = (after['yaw']-before['yaw']+math.pi)%(2*math.pi)-math.pi
            yaw = before['yaw']+fraction*dyaw
            c, s = math.cos(yaw),math.sin(yaw)
            rotation = np.array([[c,-s],[s,c]])
            centers = body[:,:3].copy()
            centers[:,:2] = centers[:,:2]@rotation.T + [x,y]
            shape = h[5:8].astype(int)
            n = int(h[38])
            esdf = a[40:40+n].reshape(shape)
            codes = a[40+n:].reshape(shape).astype(np.uint8)
            index = (centers-h[8:11])/h[11]-.5
            inside = np.all((index>=0)&(index<shape-1),axis=1)
            distance = map_coordinates(esdf,index.T,order=1,prefilter=False,mode='nearest')
            hard_gap = distance-body[:,3]
            soft_gap = distance-body[:,4]
            occupied = h[8:11]+(np.argwhere(codes&2)+.5)*h[11]
            tree = cKDTree(occupied)
            nearest_distance, nearest_index = tree.query(centers)
            nearest = occupied[nearest_index]
            occupied_raw = h[8:11]+(np.argwhere(codes&1)+.5)*h[11]
            raw_distance, raw_index = cKDTree(occupied_raw).query(centers)

            def sphere_record(k):
                cell_index = np.floor((nearest[k]-h[8:11])/h[11]+1e-7).astype(int)
                relative = nearest[k].copy()
                relative[:2] = (relative[:2]-[x,y])@rotation
                return dict(sphere_index=int(k),kind='base_fixed' if body[k,6]==0 else 'arm',
                    center_world=centers[k].tolist(),center_body=body[k,:3].tolist(),
                    exported_hard_radius=float(body[k,3]),exported_soft_radius=float(body[k,4]),
                    esdf_distance=float(distance[k]),hard_gap=float(hard_gap[k]),soft_gap=float(soft_gap[k]),
                    nearest_inflated_center_world=nearest[k].tolist(),nearest_inflated_center_body=relative.tolist(),
                    nearest_inflated_center_distance=float(nearest_distance[k]),nearest_occupancy_code=int(codes[tuple(cell_index)]),
                    nearest_raw_center_world=occupied_raw[raw_index[k]].tolist(),nearest_raw_center_distance=float(raw_distance[k]))

            groups = {}
            for name,flag in [('base_fixed',0),('arm',1)]:
                which = np.flatnonzero(body[:,6]==flag)
                worst = which[np.argsort(hard_gap[which])[:5]]
                groups[name] = dict(count=int(which.size),min_esdf_hard_gap=float(hard_gap[which].min()),
                    min_esdf_soft_gap=float(soft_gap[which].min()),hard_overlap_count=int(np.count_nonzero(hard_gap[which]<0)),
                    min_raw_center_minus_hard_radius=float((raw_distance[which]-body[which,3]).min()),
                    worst_spheres=[sphere_record(k) for k in worst])
            reports.append(dict(index=i,acquisition=float(h[37]),acquisition_local=local(h[37]),field_version=int(h[14]),
                map_fusion=float(h[3]),map_age_at_acquisition_ms=1000*(h[37]-h[3]),
                pose_xyyaw=[x,y,yaw],odom_bracket_source=[before['source'],after['source']],
                odom_bracket_ms=1000*(after['source']-before['source']),
                odom_measured_v_w=[before['v'],before['w']],snapshot_body_joint_stamp=float(snap['header'][24]),
                diagnostic_joint_stamp=float(h[24]),body_joint_stamp_difference_ms=1000*(snap['header'][24]-h[24]),
                same_occupancy_sequence=bool(snap['header'][29]==h[29]),all_spheres_inside_interpolation_domain=bool(inside.all()),
                groups=groups))
    summary = dict(frame_count=len(reports),time_window_local=[reports[0]['acquisition_local'],reports[-1]['acquisition_local']],
        exact_body_joint_stamp_match_count=int(sum(x['body_joint_stamp_difference_ms']==0 for x in reports)),
        same_occupancy_sequence_count=sum(x['same_occupancy_sequence'] for x in reports),
        all_spheres_inside_domain=all(x['all_spheres_inside_interpolation_domain'] for x in reports),
        odom_bracket_max_ms=max(x['odom_bracket_ms'] for x in reports),
        minimum_hard_gap_by_group={g:min(x['groups'][g]['min_esdf_hard_gap'] for x in reports) for g in ['base_fixed','arm']},
        hard_overlap_sphere_samples_by_group={g:sum(x['groups'][g]['hard_overlap_count'] for x in reports) for g in ['base_fixed','arm']})
    stops = [r for r in rows('logs.jsonl') if 'stop before replanning' in r.get('msg','')]
    stop_alignment = []
    for stop in stops:
        nearest = min(reports,key=lambda f:abs(f['acquisition']-stop['source']))
        delta = nearest['acquisition']-stop['source']
        if abs(delta)>.55:
            continue
        stop_alignment.append(dict(stop_source=stop['source'],stop_source_local=local(stop['source']),
            stop_receipt=stop['bag'],diagnostic_index=nearest['index'],diagnostic_minus_stop_ms=delta*1000,
            min_base_hard_gap=nearest['groups']['base_fixed']['min_esdf_hard_gap'],
            min_arm_hard_gap=nearest['groups']['arm']['min_esdf_hard_gap'],
            complete_map_age_ms=nearest['map_age_at_acquisition_ms']))
    output = dict(summary=summary,stop_source_alignment=stop_alignment,method=[
        'Only occupancy_evidence.bag and root-extracted snapshot_prefix/odom_pose JSONL are read; the large bag is not opened.',
        'Same exported field is required; body with nearest diagnostic joint stamp is selected, and time offsets are reported.',
        'Odometry x/y and shortest-angle yaw are linearly interpolated at diagnostic acquisition between bracketing source stamps.',
        'As in runtime planar sphere model, odometry z is not added; exported fixed/arm sphere heights already define collision height.',
        'ESDF query uses voxel-center trilinear interpolation. Exported hard radii already include the fixed-base one-resolution margin; no second margin is added.',
        'Occupied nearest centers are merged raw/inflated map voxels; these diagnostics do not label static PCD versus live sensor provenance.',
        'Reported distances are collision-model gaps on the exported Complete map, not physical tape-measure distances. The ESDF saturates at 0.6 m, so saturated gaps are conservative lower bounds.',
        'FSM live-occupancy checks may use a newer map layer than the coherent exported Complete map. The bag does not establish the exact live map used by each stop decision.',
        'These are current actual-pose checks, not predicted trajectory checks; no assertion about future reference collision is made.'],frames=reports)
    (HERE/'actual_body_clearance.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
