"""Read only extracted stop point clouds and recorded TF; no ROS node/master."""
import bisect,json,math
from pathlib import Path
import numpy as np
import rospy,tf2_py
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

BASE=Path('/home/l/remani_ws/experiments/online_remani_validation/results/dynamic_geometry_capture_20261114-175950/analysis_clear_stutter')
DIR=BASE/'stop_clouds'

def rows(path):
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]

transforms=rows(DIR/'transforms.jsonl')
tf=tf2_py.BufferCore(rospy.Duration(120))
for r in sorted(transforms,key=lambda v:v['stamp']):
    m=TransformStamped();m.header.stamp=rospy.Time.from_sec(r['stamp'])
    m.header.frame_id=r['parent'];m.child_frame_id=r['child']
    m.transform.translation.x,m.transform.translation.y,m.transform.translation.z=r['translation']
    m.transform.rotation.x,m.transform.rotation.y,m.transform.rotation.z,m.transform.rotation.w=r['rotation']
    if r['topic']=='/tf_static':tf.set_transform_static(m,'recorded_offline')
    else:tf.set_transform(m,'recorded_offline')

odom=sorted(rows(BASE/'odom_full_pose.jsonl'),key=lambda v:v['source'])
stamps=[v['source'] for v in odom]
snapshot=rows(BASE/'snapshot_prefix.jsonl')[0]
body=np.asarray(snapshot['body']);body=body[body[:,6]==0]
resolution=float(snapshot['header'][11])
stop_times=[1794650930.353382,1794650933.8426497,1794650944.1902032]
frames=[]
for row in json.loads((DIR/'index.json').read_text()):
    report=dict(row);time=row['source']
    report['relative_to_stop_s']=time-stop_times[row['event']]
    try:
        tr=tf.lookup_transform_core('world',row['frame'],rospy.Time.from_sec(time))
    except Exception as e:
        report['error']='Recorded exact-time TF unavailable: '+str(e);frames.append(report);continue
    qt=tr.transform.rotation;tt=tr.transform.translation
    rotation=Rotation.from_quat([qt.x,qt.y,qt.z,qt.w]).as_dcm()
    translation=np.array([tt.x,tt.y,tt.z])
    with np.load(DIR/row['file']) as data:
        points=data['xyz'].astype(float)
    finite=np.isfinite(points).all(axis=1)
    world=points@rotation.T+translation
    j=bisect.bisect_right(stamps,time);a,b=odom[j-1:j+1]
    alpha=(time-a['source'])/(b['source']-a['source'])
    x=a['x']+alpha*(b['x']-a['x']);y=a['y']+alpha*(b['y']-a['y'])
    yaw=a['yaw']+alpha*((b['yaw']-a['yaw']+math.pi)%(2*math.pi)-math.pi)
    c,s=math.cos(yaw),math.sin(yaw)
    R=np.array([[c,-s],[s,c]])
    centers=body[:,:3].copy();centers[:,:2]=centers[:,:2]@R.T+[x,y]
    center_distance=np.sqrt(((world[:,None,:]-centers[None,:,:])**2).sum(axis=2))
    hard_gap=center_distance-body[None,:,3]
    physical_gap=center_distance-(body[None,:,3]-resolution)
    min_gap=hard_gap.min(axis=1);min_physical=physical_gap.min(axis=1)
    # These are the source-configured point acceptance filters, not proof that
    # the input callback ran or that its pending frame was eventually fused.
    eligible=finite&(np.abs(points)<[8.,8.,3.]).all(axis=1)&(world[:,2]>=.15)
    before_ground=finite&(np.abs(points)<[8.,8.,3.]).all(axis=1)
    def describe(mask):
        ids=np.flatnonzero(mask)
        if not ids.size:return dict(points=0)
        worst=ids[np.argsort(min_gap[ids])[:4]]
        return dict(points=int(ids.size),min_hard_gap=float(min_gap[ids].min()),
            min_physical_sphere_gap=float(min_physical[ids].min()),
            hard_overlap_points=int(np.count_nonzero(min_gap[ids]<0)),
            physical_sphere_overlap_points=int(np.count_nonzero(min_physical[ids]<0)),
            within_5cm_of_hard_spheres=int(np.count_nonzero(min_gap[ids]<.05)),
            closest=[dict(point_index=int(k),point_sensor=points[k].tolist(),point_world=world[k].tolist(),
                world_height=float(world[k,2]),sphere_index=int(np.argmin(hard_gap[k])),
                hard_gap=float(min_gap[k]),physical_sphere_gap=float(min_physical[k])) for k in worst])
    report.update(dict(tf_world_sensor_translation=translation.tolist(),
        tf_world_sensor_quaternion=[qt.x,qt.y,qt.z,qt.w],actual_base_xyyaw=[x,y,yaw],
        odom_bracket_ms=1000*(b['source']-a['source']),all_finite_input=describe(finite),
        source_filter_eligible=describe(eligible),removed_by_ground_filter=describe(before_ground&~eligible),
        ground_below_15cm_count=int(np.count_nonzero(finite&(world[:,2]<.15)))))
    frames.append(report)

summary=[]
for e,t in enumerate(stop_times):
    selected=[r for r in frames if r['event']==e and r['relative_to_stop_s']<=0 and 'error' not in r]
    for group in ['all_finite_input','source_filter_eligible']:
        nonempty=[r[group] for r in selected if r[group]['points']]
        summary.append(dict(event=e,subset='before_stop',point_group=group,frame_count=len(selected),
            point_samples=sum(d['points'] for d in nonempty),
            min_hard_gap=min(d['min_hard_gap'] for d in nonempty),
            min_physical_sphere_gap=min(d['min_physical_sphere_gap'] for d in nonempty),
            hard_overlap_point_samples=sum(d['hard_overlap_points'] for d in nonempty),
            physical_overlap_point_samples=sum(d['physical_sphere_overlap_points'] for d in nonempty)))
output=dict(method=[
    'Actual live mapping callback is GridMap::cloudOdomCallback for pose_type=2. It performs lookupTransform(world, cloud.frame_id, cloud.header.stamp); no odometry/extrinsic fallback is used in this implementation.',
    'Recorded TF is replayed through tf2 BufferCore, with interpolation at exact scan source stamp. Chain world-map-camera_init-body-livox_frame uses recorded static body-to-LiDAR calibration, not launch lidar_offset in its place.',
    'Cloud geometry is compared to the 47 fixed collision spheres at odometry source-time interpolated x/y/yaw. Configured world sphere heights are preserved; odometry z is not added.',
    'Hard radius is exported radius, including one 0.05m voxel margin. Physical-sphere radius removes that margin; it remains a model radius, not a measured physical surface.',
    'Source configured range abs(sensor xyz)<(8,8,3) and world z>=0.15 are reported as an eligible subset. Launch disables downstream box self filter. No claim is made that this frame was accepted, fused, or caused a queried voxel.',
    'Only extracted small NPZ/JSONL files are opened. No robot node, ROS publisher, service or large bag is used.'],
    frame_count=len(frames),tf_failure_count=sum('error'in r for r in frames),before_stop_summary=summary,frames=frames)
(BASE/'stop_cloud_geometry.json').write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps(dict(frame_count=len(frames),tf_failure_count=output['tf_failure_count'],summary=summary),indent=2))
