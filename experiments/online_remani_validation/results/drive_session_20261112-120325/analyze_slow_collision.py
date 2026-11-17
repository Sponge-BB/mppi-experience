"""Small offline cloud/TF audit; does not reconstruct the missing event map."""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import rosbag
import rospy
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64MultiArray

ROOT = Path('/home/l/remani_ws')
OUT = Path(__file__).resolve().parent
HARD = 1794456473.320249
SOURCE = 1794456473.0192356
parameters = json.loads((OUT/'runtime_parameters.json').read_text())
planner = parameters['/remani_planner_node']
spheres = np.array(planner['mm']['fixed_collision_spheres'])
origin = np.array([-7.5, -5., 0.])
resolution = float(planner['grid_map']['resolution'])
cap = float(planner['grid_map']['esdf_distance_cap'])


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for data in iter(lambda: stream.read(1024*1024), b''):
            h.update(data)
    return h.hexdigest()


def pose(message):
    q = message.pose.pose.orientation
    return np.array([message.pose.pose.position.x, message.pose.pose.position.y,
                     math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))])


def centers(base):
    c = spheres[:, :3].copy()
    rotation = np.array([[math.cos(base[2]), -math.sin(base[2])],
                         [math.sin(base[2]), math.cos(base[2])]])
    c[:, :2] = c[:, :2]@rotation.T+base[:2]
    return c


def point_gaps(points, base):
    c = centers(base)
    d, idx = cKDTree(points).query(c)
    gap = d-spheres[:, 3]
    i = int(np.argmin(gap))
    return dict(gap_m=float(gap[i]), sphere_index_zero_based=i,
                sphere_center_world=c[i].tolist(), sphere_radius=float(spheres[i, 3]),
                point_world=points[idx[i]].tolist(), point_index=int(idx[i]))


def xyz(message):
    dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': ['<f4']*3,
                     'offsets': [next(f.offset for f in message.fields if f.name == n)
                                 for n in 'xyz'], 'itemsize': message.point_step})
    rows = np.ndarray((message.height, message.width), dtype=dtype,
                      buffer=message.data, strides=(message.row_step, message.point_step))
    return np.column_stack([rows[n].ravel() for n in 'xyz'])


def matrix(tr):
    q, t = tr.transform.rotation, tr.transform.translation
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_dcm(), np.array([t.x, t.y, t.z])


def single_voxel_interpolant(point, base):
    """Counterfactual exact trilinear EDT for ONE isolated occupied cell."""
    voxel = origin+(np.floor((point-origin)/resolution)+.5)*resolution
    c = centers(base)
    fraction = (c-origin)/resolution-.5
    lo = np.floor(fraction)
    fraction -= lo
    distance = np.zeros(len(spheres))
    for i in range(2):
        for j in range(2):
            for k in range(2):
                corner = origin+(lo+[i, j, k]+.5)*resolution
                weight = np.prod(np.where(np.array([i, j, k]), fraction, 1-fraction), axis=1)
                distance += weight*np.minimum(np.linalg.norm(corner-voxel, axis=1), cap)
    gap = distance-spheres[:, 3]
    rank = np.argsort(gap)
    return dict(pose=base.tolist(), voxel_center_world=voxel.tolist(),
                voxel_index=np.floor((point-origin)/resolution).astype(int).tolist(),
                minimum_gap_m=float(gap[rank[0]]),
                ranked_spheres=[dict(index=int(i), gap_m=float(gap[i])) for i in rank[:5]])


frames, static, clouds, odometry, logs, statuses = {}, {}, [], [], [], []
with rosbag.Bag(str(OUT/'capture.bag')) as bag:
    for _, m, _ in bag.read_messages(topics=['/tf_static']):
        for tr in m.transforms:
            static[(tr.header.frame_id, tr.child_frame_id)] = tr
    topics = ['/tf', '/odom', '/velodyne_points_mapping', '/velodyne_points',
              '/cloud_registered', '/rosout_agg', '/mppi/status']
    for topic, m, bt in bag.read_messages(topics=topics,
            start_time=rospy.Time.from_sec(HARD-2.4), end_time=rospy.Time.from_sec(HARD+2.2)):
        if topic == '/tf':
            for tr in m.transforms:
                if tr.header.frame_id == 'camera_init' and tr.child_frame_id == 'body':
                    frames[tr.header.stamp.to_sec()] = tr
        elif topic == '/odom':
            odometry.append((m.header.stamp.to_sec(), pose(m), bt.to_sec(), m.header.frame_id))
        elif topic == '/rosout_agg':
            logs.append(dict(receipt=bt.to_sec(), source=m.header.stamp.to_sec(), node=m.name, message=m.msg))
        elif topic == '/mppi/status':
            data = json.loads(m.data)
            statuses.append(dict(receipt=bt.to_sec(), **{k: data[k] for k in
                ('state', 'external_min_gap', 'base_v', 'base_w', 'snapshot', 'candidate_diagnostics') if k in data}))
        else:
            clouds.append((topic, m, bt.to_sec()))

# Establish world/camera_init/odom coincidence from recorded static transforms,
# rather than comparing different frame names by assumption.
chain = [('world', 'map'), ('map', 'camera_init'), ('camera_init', 'odom')]
identity_checks = []
for edge in chain:
    rotation, translation = matrix(static[edge])
    identity_checks.append(dict(parent=edge[0], child=edge[1],
        rotation_error=float(np.max(np.abs(rotation-np.eye(3)))),
        translation=translation.tolist()))
    assert np.allclose(rotation, np.eye(3), atol=1e-12) and np.linalg.norm(translation) < 1e-12
extrinsic_R, extrinsic_t = matrix(static[('body', 'livox_frame')])
event_odom = min(odometry, key=lambda row: abs(row[0]-HARD))
previous_odom = min(odometry, key=lambda row: abs(row[0]-1794456473.2191284))
cloud_evidence, samples = [], {}
for topic, m, receipt in clouds:
    stamp = m.header.stamp.to_sec()
    points = xyz(m)
    if m.header.frame_id == 'livox_frame':
        assert stamp in frames, 'This audit requires the exact saved scan TF'
        rotation, translation = matrix(frames[stamp])
        world = (points@extrinsic_R.T+extrinsic_t)@rotation.T+translation
    else:
        assert m.header.frame_id == 'camera_init'
        world = points
    finite = np.isfinite(world).all(axis=1)
    selected = finite & (world[:, 2] >= .15)
    filtered = world[selected]
    odom = min(odometry, key=lambda row: abs(row[0]-stamp))
    assert abs(odom[0]-stamp) < 1e-9
    row = dict(topic=topic, source=stamp, receipt=receipt, frame=m.header.frame_id,
               tf_exact_stamp=True, pose=odom[1].tolist(), after_world_z_015_points=len(filtered),
               nearest_at_scan_pose=point_gaps(filtered, odom[1]),
               nearest_at_neighbor_event_pose=point_gaps(filtered, event_odom[1]))
    cloud_evidence.append(row)
    if abs(stamp-SOURCE) < 1e-9:
        samples[topic] = (m, world, row)

point = np.array(samples['/velodyne_points_mapping'][2]['nearest_at_scan_pose']['point_world'])
own_pose = np.array(samples['/velodyne_points_mapping'][2]['pose'])
raw_m, raw, _ = samples['/cloud_registered']
raw_distance, raw_index = cKDTree(raw).query(point)
filtered_m, filtered_world, _ = samples['/velodyne_points']
filtered_distance, filtered_index = cKDTree(filtered_world).query(point)
q = np.array([[math.cos(own_pose[2]), -math.sin(own_pose[2])],
              [math.sin(own_pose[2]), math.cos(own_pose[2])]])
local_point = np.r_[(point[:2]-own_pose[:2])@q, point[2]]
candidate = dict(source=SOURCE, point_world=point.tolist(),
    point_in_planar_base_ground_coordinates=local_point.tolist(),
    raw_cloud_index=int(raw_index), raw_correspondence_error_m=float(raw_distance),
    filtered_cloud_index=int(filtered_index), filtered_correspondence_error_m=float(filtered_distance),
    physical_sphere_gap_m=samples['/velodyne_points_mapping'][2]['nearest_at_scan_pose']['gap_m'],
    self_filter_padding_m=.01,
    evidence_boundary='This is a recorded near-body return, not proof that its physical source was the robot. Missing full-rate clouds and event ESDF prevent exact occupancy attribution.')
counterfactuals = {
    'same_scan_pose': single_voxel_interpolant(point, own_pose),
    'recorded_odom_73_219128': single_voxel_interpolant(point, previous_odom[1]),
    'nearest_event_odom_73_318959': single_voxel_interpolant(point, event_odom[1])}

# PCD is in map-input coordinates. The running publisher still applies the
# launch transform (+.075m Z); then GridMap excludes points below world .15m.
with (ROOT/'src/mm_remani/maps/exp0_planner.pcd').open('rb') as stream:
    while not stream.readline().startswith(b'DATA binary'):
        pass
    pcd = np.frombuffer(stream.read(), dtype='<f4').reshape(-1, 3).astype(float)
pcd += [0., 0., .075]
pcd = pcd[pcd[:, 2] >= .15]
static_query = point_gaps(pcd, event_odom[1])
static_query['caveat'] = 'Reconstructed launch PCD transform, not the event occupancy/ESDF; no PCD correction topic was recorded. Runtime auto_icp_adjust=false.'

# Recorded publisher body rows verify runtime native fixed spheres, without
# assuming that source files alone prove what the linked library produced.
geometry_checks = []
for path in sorted((OUT/'map_events').glob('*_full.msg')):
    msg = Float64MultiArray().deserialize(path.read_bytes())
    a = np.asarray(msg.data)
    body = a[32:32+7*int(a[12])].reshape(-1, 7)[:47]
    geometry_checks.append(dict(file=str(path.relative_to(OUT)), version=int(a[14]),
        center_max_error=float(np.max(np.abs(body[:, :3]-spheres[:, :3]))),
        physical_radius_max_error=float(np.max(np.abs(body[:, 3]-a[11]-spheres[:, 3])))))
headers = []
for line in (OUT/'snapshot_headers.jsonl').open():
    row = json.loads(line)
    if HARD-.7 <= row['receive_ros'] <= HARD+.4:
        h = row['header']
        headers.append(dict(receipt=row['receive_ros'], sequence=int(h[2]), fusion=h[3], valid=h[4],
            field_version=int(h[14]), acquisition=h[17], complete_observation=int(h[29]), full=bool(h[20])))
code_paths = ['src/mm_remani_planner/plan_env/src/grid_map.cpp',
 'src/mm_remani_planner/plan_env/include/plan_env/grid_map.h',
 'src/mm_remani_planner/mm_config/src/mm_config.cpp',
 'src/mm_remani_planner/plan_manage/src/remani_replan_fsm.cpp',
 'src/mm_remani_planner/plan_manage/include/plan_manage/mppi_collision_export.hpp',
 'src/mm_remani/scripts/mppi/active_arm_mppi.py', 'src/mm_remani/scripts/mppi/active_arm_model.py',
 'src/mm_remani/scripts/velodyne_transformer.py', 'devel/lib/libmm_config.so',
 'devel/lib/libplan_env.so', 'devel/lib/remani_planner/remani_planner_node']
code = {path:sha(ROOT/path) for path in code_paths}
capture_hashes = json.loads((OUT/'code_and_inputs.json').read_text())
report = dict(
 scope='Offline first slow-layer stop audit. No runtime edits, ROS nodes, compilation or hardware commands. Small neighboring clouds; event ESDF was NOT saved.',
 first_latch_source=1794456473.207416, hard_collision_source=HARD,
 hard_collision_logged=dict(type=0, check_distance_m=.004, base_surface_gap_m=.004, hard_stop_margin_m=.01),
 code_sha256=code,
 captured_planner_binary_matches=(code['devel/lib/remani_planner/remani_planner_node']==capture_hashes['devel/lib/remani_planner/remani_planner_node']['sha256']),
 deployed_library_provenance='Current libplan_env SHA70d8f688 matches complete restore-pre020 and inherited versions; libmm_config SHA0d85aa62 matches all complete baselines. Libraries were not separately hashed by this recorder at capture start; cannot claim loaded memory attestation.',
 frame_identity_checks=identity_checks,
 geometry=dict(fixed_sphere_count=47, native_vs_python='Both use runtime mm/fixed_collision_spheres and planar world x/y/yaw. Z already includes ground offset, not base_link Z again.',
    runtime_publisher_rows=geometry_checks,
    slow_formula='min_i trilinear(native ESDF, sphere_center_i) - physical_radius_i; emergency if <.01m',
    mppi_formula='min_i trilinear(exported Complete ESDF crop, sphere_center_i) - physical_radius_i -.05m for fixed spheres, plus dynamic points when present',
    same_field_implication='If same base pose and ESDF give slow gap .004m, MPPI fixed-sphere gap must be at most -.046m. Geometry/radius policy cannot explain positive .43166m for the same inputs.',
    mppi_cap_gap_m=cap-float(spheres[:,3].max())-resolution),
 retained_near_body_return=candidate, neighbor_clouds=cloud_evidence,
 counterfactual_single_voxel=counterfactuals,
 counterfactual_boundary='One occupied voxel hypothesis uses the C++ trilinear formula; .004327m at recorded prior odom rounds to .004 log. It is not replay of the missing generation3356 ESDF, and exact FSM callback odom was not recorded.',
 static_pcd_check=static_query, snapshot_headers=headers,
 event_logs=[r for r in logs if any(token in r['message'] for token in ['collision', 'seq=3356', 'not free', 'expired', 'self-filter'])],
 mppi_status_window=[r for r in statuses if HARD-.8<=r['receipt']<=HARD+.3],
 causal_assessment=[
  'A near-body return is present in the full filtered branch AND mapping sparse branch at the identical stamp; sparse mapping did not create this point.',
  'Its raw distance exceeds1cm self-mask by0.8066mm, but5cm voxel rounding moves the obstacle center and can put the distance field inside the robot at the same pose.',
  'Native live raycast marks a voxel occupied from any endpoint in that frame; p_hit=.65 does not imply multi-frame confirmation because occupied buffer is set directly to clamp_max.',
  'At the last delivered commands, MPPI uses older Complete generations. Raw occupancy3356 triggered slow stop before its complete field was exported. Header3356 first received73.356; prior .431660 gap belongs earlier fields. Thus the two values do not prove simultaneous inconsistent geometry.',
  'Evidence strongly supports a short-lived near-body occupancy event amplified by voxelization/map latency. Physical origin (robot echo, multipath/noise, or a genuine nearby object) remains unproven; do not erase it or lower the hard-stop threshold based on this inference alone.'],
 minimal_followup=[
  'At the slow hard-stop, retain actual base pose, minimizing sphere index/center, occupancy and ESDF sequence, and a small exact local crop from the same ReadScope. This closes the present missing-generation ambiguity.',
  'Use the saved scan and verified physical body geometry to repair specifically the self-filter/voxel boundary if the return is confirmed self echo; test both that point and real external points just outside the body. Do not globally enlarge blind filtering or remove collision protection.',
  'Keep fast/slow geometry definitions aligned and measure Complete-generation age. Optimize delivery latency separately; changing collision radii or treating historical MPPI gap as current clearance is not justified.'])
(OUT/'slow_collision_root_cause_analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
text = '''首个慢层碰撞判断：离线证据复核

结论：已找到能数值解释0.004m的近车体点云线索，暂不能称同代地图精确复现。
12:07:53.0192356保存的完整过滤点云和稀疏建图点云均含同一点：world=(0.32747656,-0.51998431,0.19448658)。其同源TF与里程计齐全，变换无插值。
该点在扫描时距47球模型最近表面10.8066mm，只比现有10mm自体过滤边界多0.8066mm。它被保留不是稀疏支路新增的点。
该点量化为5cm体素中心(0.325,-0.525,0.175)。假设该体素单独占据，按原C++八角点三线性距离计算，同一扫描姿态下底盘表面间隙为-3.7319mm；采用已录53.219128的里程计，间隙为4.3270mm，与53.320日志保留三位小数的0.004一致。最近53.318959里程计对应2.8476mm。
这是单体素反事实核算，不是缺失3356代ESDF的重放；FSM到底使用哪帧里程计没有独立记录。也不能仅凭该点的位置证明它来自机器人自身。

快慢层为什么看起来差很大：
两层47个固定球中心与物理半径一致，已用7份现场全量包body行比对，中心/半径误差为数值舍入量。慢层硬停计算真实半径表面间隙<1cm；MPPI还加5cm体素裕度，因此同一姿态同一场若慢层为0.004，MPPI应至多为-0.046，不可能为+0.43166。
记录实际不是同一时刻同一代：最后数次MPPI指令的0.431660=0.6距离截断上限-最大固定球半径-0.05；53.207慢层已经锁定新占据3356，而MPPI包仍为3354/3355。3356完整场首次记录接收为53.356，硬碰撞日志源时刻53.320。
因此不能拿故障前MPPI旧场的正间隙反驳新占据触发的慢层硬停，也没有证据归因于两套半径模型冲突。

地图放大环节：
原生raycast中任一命中端点即可使本帧体素occupied=true，并直接设clamp_max；运行p_hit=0.65不是多帧确认门限。一处偶发近体返回就可生成障碍；5cm体素取中心和机器人位姿变化足以改变间隙符号。邻近52.019和54.019保存帧的过滤点云没有这个近体点，但采集只有1Hz，不能据此证明其持续时间或真实来源。
静态PCD必须保留运行+0.075mZ变换并应用GridMap世界z>=0.15过滤；已按该规则核算，附近静态点远离车体，不支持用这一静态地图邻近点解释4mm。

最小改进方向：
1. 慢层硬停当轮保存实际位姿、最小球编号/中心、占据与ESDF代号和同一ReadScope的小裁剪场，补齐这次证据缺口。
2. 先确认该近体返回是否为自体回波/噪声，再针对对应物理部件修正自体过滤与体素边界的一致性，并保留贴近车体的真实外障反例。不建议直接删除所有近点、全面加大盲区或降低硬停门限。
3. 快慢层沿用同一物理球定义；地图提交/交付时延单独优化。保留真实碰撞停止。

证据边界：未修改运行代码、未启动ROS节点/机器人、未编译；没有3356代全场。部署程序SHA与采集一致；当前库SHA可追溯回退链，但记录器没有采集开始时单独hash共享库，不能称已证明运行进程载入内存完全一致。详细数值、原始索引、时间线和源码hash见同名JSON；核算脚本analyze_slow_collision.py。
'''
(OUT/'slow_collision_root_cause_analysis.txt').write_text(text)
print(json.dumps({'candidate':candidate,'counterfactuals':counterfactuals,
                  'static':static_query,'geometry_checks':geometry_checks},indent=2))
