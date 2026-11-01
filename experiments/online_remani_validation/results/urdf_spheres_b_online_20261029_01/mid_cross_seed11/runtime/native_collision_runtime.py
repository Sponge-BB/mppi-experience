"""Native-3D snapshot binding for NEW isolated trial runtime copies only.

The legacy 2-D frame remains a proposal/visualization input, never a fallback
authorization source when native 3-D mode is enabled. No ROS import at module
scope: protocol, freshness and patching can be checked without a ROS master.
"""
import ctypes as ct
import hashlib
import json
from pathlib import Path
import queue
import threading

import numpy as np

HEADER = 32
MAX_VALUES = 8_000_000
TOPIC = '/sim/native_collision_3d'
DP = ct.POINTER(ct.c_double)


def decode_snapshot(data):
    raw = bytes(data)
    if len(raw) % 8 or not HEADER * 8 <= len(raw) <= MAX_VALUES * 8:
        raise ValueError('native3d_invalid_binary_length')
    values = np.frombuffer(raw, dtype='<f8').astype(np.float64, copy=True)
    if not np.isfinite(values).all() or values[0] != 3 or values[1] != 1:
        raise ValueError('native3d_invalid_header')
    for index in (2, 5, 6, 7, 12, 13, 14):
        if values[index] != int(values[index]) or values[index] < 0:
            raise ValueError('native3d_invalid_count')
    nx, ny, nz, nb, nd = (int(values[i]) for i in (5, 6, 7, 12, 13))
    if min(nx, ny, nz, nb) < 1 or nb > 1000 or nd > 100000 or values[11] <= 0:
        raise ValueError('native3d_invalid_shape')
    if values[20] not in (0., 1.):
        raise ValueError('native3d_invalid_field_mode')
    count = HEADER + 7 * (nb + nd) + (nx * ny * nz if values[20] else 0)
    if len(values) != count:
        raise ValueError('native3d_incomplete_snapshot')
    body = values[HEADER:HEADER + 7 * nb].reshape((-1, 7))
    if np.any(body[:, 3:5] <= 0) or np.any(body[:, 5] < 0):
        raise ValueError('native3d_invalid_body')
    return np.ascontiguousarray(values)


def freshness_reason(payload, now):
    if payload is None:
        return 'native3d_missing_snapshot'
    if payload[4] != 1 or payload[19] != 1:
        return 'native3d_unhealthy'
    if payload[22] != 1 or payload[23] != 1:
        return 'native3d_fixed_body_invalid'
    # h17 is ESDF acquisition time, NOT its original observation timestamp.
    # A static ESDF may be cached; the publisher's health contract owns it.
    max_age = min(.35, float(payload[15]))
    if max_age <= 0 or not -.05 <= now - payload[3] <= max_age:
        return 'native3d_publication_stale'
    if not -.05 <= now - payload[18] <= max_age:
        return 'native3d_dynamic_stale'
    # Native FK is a cached fixed-arm shape. Actual joint freshness and its
    # deviation from fixed q are checked independently by local_execution.
    if payload[24] <= 0 or payload[24] > now + .05:
        return 'native3d_joint_invalid'
    return None


def field_key(payload):
    return tuple(payload[i] for i in (14, 5, 6, 7, 8, 9, 10, 11))


def attach_cached_field(payload, cached):
    """Prevent an early full field being lost before the first solver bind."""
    offset = HEADER + 7 * int(payload[12] + payload[13])
    if payload[20] == 1:
        return payload, (field_key(payload), payload[offset:].copy())
    if cached is None or cached[0] != field_key(payload):
        return payload, cached
    merged = np.concatenate((payload, cached[1]))
    merged[20] = 1.
    return merged, cached


class SnapshotRecorder:
    """Lossless, map-deduplicated raw ROS payload evidence off callback thread."""
    def __init__(self, out):
        self.out = Path(out)
        self.root = self.out / 'native3d_snapshots'
        (self.root / 'maps').mkdir(parents=True, exist_ok=False)
        self.queue = queue.Queue(maxsize=64)
        self.closed = False
        self.stats = {'frames': 0, 'parity': 0, 'dropped': 0, 'errors': []}
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()

    def submit(self, raw, payload=None, parity=False):
        if self.closed:
            return
        try:
            self.queue.put_nowait((raw, payload, parity))
        except queue.Full:
            self.stats['dropped'] += 1

    def _write(self):
        maps = {}
        with (self.root/'frames.bin').open('wb') as frames, (self.root/'parity.bin').open('wb') as parity, \
                (self.root/'index.jsonl').open('w', buffering=1) as index:
            while True:
                item = self.queue.get()
                try:
                    if item is None:
                        return
                    raw, payload, is_parity = item
                    if is_parity:
                        offset = parity.tell()
                        parity.write(raw)
                        index.write(json.dumps({'kind': 'parity', 'offset': offset, 'length': len(raw),
                            'sha256': hashlib.sha256(raw).hexdigest(),
                            'header': np.frombuffer(raw, dtype='<f8')[:6].tolist()})+'\n')
                        self.stats['parity'] += 1
                        continue
                    nb, nd = int(payload[12]), int(payload[13])
                    prefix_bytes = 8*(HEADER+7*(nb+nd))
                    prefix, field = raw[:prefix_bytes], raw[prefix_bytes:]
                    key = hashlib.sha256(np.asarray(field_key(payload), dtype='<f8').tobytes()).hexdigest()
                    if payload[20] == 1:
                        digest = hashlib.sha256(field).hexdigest()
                        if key in maps and maps[key] != digest:
                            raise ValueError('immutable_native3d_mapkey_changed_content')
                        if key not in maps:
                            # Different observation sequences can contain the
                            # identical raw field. Deduplicate by CONTENT while
                            # retaining every original key/header in the index.
                            map_path = self.root/'maps'/(digest+'.f64')
                            if not map_path.exists():
                                map_path.write_bytes(field)
                            maps[key] = digest
                    map_file = 'maps/'+maps.get(key, key)+'.f64'
                    offset = frames.tell()
                    frames.write(prefix)
                    index.write(json.dumps({'kind': 'snapshot', 'offset': offset, 'length': len(prefix),
                        'raw_length': len(raw), 'raw_sha256': hashlib.sha256(raw).hexdigest(),
                        'prefix_sha256': hashlib.sha256(prefix).hexdigest(), 'header': payload[:HEADER].tolist(),
                        'map_key': key, 'map_file': map_file, 'map_sha256': maps.get(key),
                        'original_full_esdf': bool(payload[20]), 'cached_map_available': key in maps})+'\n')
                    self.stats['frames'] += 1
                    body_path = self.out/'native_collision_3d_body.json'
                    if not body_path.exists():
                        body_path.write_text(json.dumps({'schema_version': 1, 'source': TOPIC,
                            'header': payload[:HEADER].tolist(),
                            'body': payload[HEADER:HEADER+7*nb].reshape((-1, 7)).tolist(),
                            'scope': 'actual native MM samples; physical base radius=hard_radius-map_resolution, arm radius=hard_radius'}, indent=2)+'\n')
                except Exception as exc:
                    self.stats['errors'].append('%s: %s' % (type(exc).__name__, exc))
                finally:
                    self.queue.task_done()

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.queue.put(None)
        self.worker.join()
        self.stats['scope'] = 'exact UInt8 payload prefixes plus once-per-map raw ESDF; lossless message reconstruction; background I/O, no control authorization'
        (self.root/'recording.json').write_text(json.dumps(self.stats, indent=2)+'\n')


def load_recorded_snapshot(root, record, self_contained=False):
    """Reconstruct exactly the raw ROS payload, or attach its cached ESDF."""
    root = Path(root)
    if record['kind'] != 'snapshot':
        raise ValueError('not a snapshot record')
    with (root/'frames.bin').open('rb') as stream:
        stream.seek(record['offset'])
        prefix = stream.read(record['length'])
    if hashlib.sha256(prefix).hexdigest() != record['prefix_sha256']:
        raise ValueError('snapshot prefix hash mismatch')
    field = b''
    if record['original_full_esdf'] or self_contained:
        field = (root/record['map_file']).read_bytes()
        if hashlib.sha256(field).hexdigest() != record['map_sha256']:
            raise ValueError('snapshot map hash mismatch')
    raw = prefix + (field if record['original_full_esdf'] else b'')
    if hashlib.sha256(raw).hexdigest() != record['raw_sha256'] or len(raw) != record['raw_length']:
        raise ValueError('raw snapshot reconstruction mismatch')
    if self_contained:
        values = np.frombuffer(prefix+field, dtype='<f8').copy()
        values[20] = 1.
        return values
    return raw


def configure(node):
    import rospy
    from std_msgs.msg import UInt8MultiArray
    node.native_collision_3d = bool(rospy.get_param('~native_collision_3d', False))
    node.native_collision_payload = None
    node.native_collision_error = 'native3d_missing_snapshot'
    node.native_collision_bindings = {}
    node.native_collision_body_written = False
    node.native_collision_cached_field = None
    node.native_proposal_field = np.full((node.ny, node.nx), 10., dtype=np.float64)
    node.native_collision_log = (node.out / 'native_collision_cycles.jsonl').open('w', buffering=1)
    rospy.on_shutdown(node.native_collision_log.close)
    if not node.native_collision_3d:
        return
    for name in ('online_set_native_collision_snapshot', 'online_set_native_collision_clock'):
        if not hasattr(node.lib, name):
            raise RuntimeError('native3d_required_abi_missing: ' + name)
    node.lib.online_set_native_collision_snapshot.argtypes = [ct.c_void_p, DP, ct.c_int]
    node.lib.online_set_native_collision_snapshot.restype = ct.c_int
    node.lib.online_set_native_collision_clock.argtypes = [ct.c_void_p, ct.c_double]
    node.lib.online_set_native_collision_clock.restype = ct.c_int

    def receive(message):
        try:
            payload = decode_snapshot(message.data)
            payload, cached_field = attach_cached_field(payload, node.native_collision_cached_field)
            error = freshness_reason(payload, rospy.get_time())
            with node.lock:
                previous = node.native_collision_payload
                if previous is not None and payload[2] <= previous[2]:
                    return
                node.native_collision_payload = payload
                node.native_collision_cached_field = cached_field
                node.native_collision_error = error
        except (ValueError, TypeError, OverflowError) as exc:
            with node.lock:
                node.native_collision_payload = None
                node.native_collision_error = str(exc)
    node.native_collision_subscription = rospy.Subscriber(TOPIC, UInt8MultiArray, receive, queue_size=1,
                                                         buff_size=64 * 1024 * 1024)


def bind_collision(node, now, phase):
    """Called at both solver entry and latest-frame publication validation.

    The caller may hold node.lock. Payload arrays are replaced atomically, not
    mutated; acquiring that non-reentrant lock here would deadlock validation.
    """
    if not node.native_collision_3d:
        payload = node.collision_payload if node.shared_collision else None
        if node.shared_collision and (payload is None or node.lib.online_set_collision_snapshot(
                node.handle, payload.ctypes.data_as(DP), len(payload))):
            return 'shared_collision_binding_failed'
        return None
    payload = node.native_collision_payload
    reason = freshness_reason(payload, now)
    entry = {'ros_time': now, 'reason': reason, 'header': payload[:HEADER].tolist() if payload is not None else None}
    node.native_collision_bindings[phase] = entry
    if reason:
        return reason
    code = node.lib.online_set_native_collision_snapshot(node.handle, payload.ctypes.data_as(DP), len(payload))
    entry['snapshot_code'] = int(code)
    if code:
        entry['reason'] = 'native3d_snapshot_binding_failed'
        return entry['reason']
    code = node.lib.online_set_native_collision_clock(node.handle, now)
    entry['clock_code'] = int(code)
    if code:
        entry['reason'] = 'native3d_clock_binding_failed'
    return entry['reason']


def publication_reason(node, now):
    return freshness_reason(node.native_collision_payload, now) if node.native_collision_3d else None


def collision_age(node, now, legacy_stamp):
    payload = node.native_collision_payload
    stamp = payload[18] if node.native_collision_3d and payload is not None else legacy_stamp
    return max(0., now-stamp)


def refresh_validation_clock(node, now):
    if not node.native_collision_3d:
        return None
    reason = publication_reason(node, now)
    if reason:
        return reason
    code = node.lib.online_set_native_collision_clock(node.handle, now)
    return 'native3d_clock_binding_failed' if code else None


def record_cycle(node, t, reason):
    """Telemetry only, AFTER command publication; cannot authorize movement."""
    node.native_collision_log.write(json.dumps({'t': t, 'enabled': node.native_collision_3d,
        'reason': reason, 'bindings': node.native_collision_bindings}) + '\n')
    node.native_collision_bindings = {}


def replace_once(text, anchor, replacement):
    if text.count(anchor) != 1:
        raise ValueError('native3d_runtime_patch_anchor_missing_or_ambiguous: ' + anchor[:90])
    return text.replace(anchor, replacement, 1)


def sampled_truth_clearance(state, body, resolution, actors):
    """Independent analytic finite-surface check of measured 3-D samples.

    Uses synthetic scene truth ONLY for evaluation, never controller inputs.
    Base physical radius removes the native voxel-resolution hard reserve;
    arm radius is the native physical thickness. This is not mesh contact.
    """
    body = np.asarray(body, dtype=float)
    c, s = np.cos(state[2]), np.sin(state[2])
    centers = body[:, :3].copy()
    centers[:, :2] = centers[:, :2] @ np.array([[c, s], [-s, c]]) + state[:2]
    radii = body[:, 3] - (body[:, 6] == 0) * resolution
    if len(body) == 0 or np.any(radii <= 0):
        raise ValueError('invalid native physical sample radii')
    x, y, z = centers.T
    dz = np.maximum(np.maximum(.15-z, z-1.41), 0.)
    dx = np.maximum(np.maximum(-1.5-x, x-7.5), 0.)
    dy = np.maximum(np.maximum(-2.-y, y-2.), 0.)
    gaps = [np.sqrt((y-side)**2 + dx**2 + dz**2)-radii for side in (-2., 2.)]
    gaps += [np.sqrt((x-side)**2 + dy**2 + dz**2)-radii for side in (-1.5, 7.5)]
    for ox, oy, radius, *_ in actors:
        radial = np.hypot(x-ox, y-oy) - radius
        vertical = np.abs(z-(.15+1.4)/2) - (1.4-.15)/2
        distance = np.hypot(np.maximum(radial, 0.), np.maximum(vertical, 0.)) + np.minimum(np.maximum(radial, vertical), 0.)
        gaps.append(distance-radii)
    return float(np.min(gaps))


def configure_truth_monitor(sim):
    import rospy
    from std_msgs.msg import UInt8MultiArray
    sim.native_truth_body = None
    sim.native_truth_resolution = None
    sim.native_truth_min_gap = float('inf')
    sim.native_truth_collision = False
    sim.native_truth_checks = 0
    sim.native_truth_error = 'waiting_native_geometry'
    sim.native_snapshot_recorder = SnapshotRecorder(sim.out)
    rospy.on_shutdown(sim.native_snapshot_recorder.close)

    def receive(message):
        try:
            raw = bytes(message.data)
            payload = decode_snapshot(raw)
            sim.native_snapshot_recorder.submit(raw, payload)
            body = payload[HEADER:HEADER+7*int(payload[12])].reshape((-1, 7)).copy()
            sampled_truth_clearance(np.zeros(5), body, payload[11], [])
            with sim.lock:
                sim.native_truth_body = body
                sim.native_truth_resolution = float(payload[11])
                sim.native_truth_error = None
        except (ValueError, TypeError, OverflowError) as exc:
            with sim.lock:
                sim.native_truth_error = str(exc)
    sim.native_truth_subscription = rospy.Subscriber(TOPIC, UInt8MultiArray, receive, queue_size=1,
                                                    buff_size=64*1024*1024)
    def receive_parity(message):
        raw = bytes(message.data)
        if len(raw) % 8 == 0 and len(raw) >= 6*8:
            sim.native_snapshot_recorder.submit(raw, parity=True)
    sim.native_truth_parity_subscription = rospy.Subscriber('/sim/native_collision_parity', UInt8MultiArray,
        receive_parity, queue_size=1, buff_size=1024*1024)


def check_truth(sim, t):
    body = sim.native_truth_body
    if body is None:
        return
    gap = sampled_truth_clearance(sim.s, body, sim.native_truth_resolution, sim.obstacles(t))
    sim.native_truth_checks += 1
    sim.native_truth_min_gap = min(sim.native_truth_min_gap, gap)
    sim.native_truth_collision |= gap < 0


def patch_trial(out, enabled, parity=False):
    """Exact edits to a freshly created trial snapshot; never production files."""
    import shutil
    import xml.etree.ElementTree as ET
    out = Path(out).resolve()
    runtime = out / 'runtime'
    text = (runtime / 'mppi_node.py').read_text()
    text = replace_once(text, '            configure(self)',
        '            configure(self)\n            from native_collision_runtime import configure as configure_native3d\n            configure_native3d(self)')
    (runtime / 'mppi_node.py').write_text(text)
    text = (runtime / 'local_execution.py').read_text()
    text = replace_once(text, 'def run(node):\n',
        'def run(node):\n    from native_collision_runtime import bind_collision, publication_reason, record_cycle, collision_age, refresh_validation_clock\n')
    old = '''                if node.shared_collision and (collision_payload is None or
                    node.lib.online_set_collision_snapshot(node.handle,ptr(collision_payload),len(collision_payload))):
                    reason='shared_collision_binding_failed'
                    code=-1'''
    new = '''                binding_error=bind_collision(node,now,'solve')
                if binding_error:
                    reason=binding_error
                    code=-1'''
    text = replace_once(text, old, new)
    # New 3-D mode must not retain legacy 2-D map/age readiness as an extra
    # environmental authorization gate. Keep it solely as a proposal input.
    for old, new in (
        ("            elif not healthy or field is None or not -.05<=now-cloud_stamp<=.35:reason='perception_unhealthy'",
         "            elif not node.native_collision_3d and (not healthy or field is None or not -.05<=now-cloud_stamp<=.35):reason='perception_unhealthy'"),
        ("            elif not -.05<=now-stamp<=.35:reason='dynamic_stale'",
         "            elif not node.native_collision_3d and not -.05<=now-stamp<=.35:reason='dynamic_stale'"),
        ("            elif not node.healthy or field is None or not -.05<=published-cloud_stamp<=.35:reason='perception_unhealthy'",
         "            elif not node.native_collision_3d and (not node.healthy or field is None or not -.05<=published-cloud_stamp<=.35):reason='perception_unhealthy'"),
        ("            elif not -.05<=published-stamp<=.35:reason='dynamic_stale'",
         "            elif not node.native_collision_3d and not -.05<=published-stamp<=.35:reason='dynamic_stale'"),
        ("            elif not -.05<=final_time-cloud_stamp<=.35:reason='perception_unhealthy'",
         "            elif not node.native_collision_3d and not -.05<=final_time-cloud_stamp<=.35:reason='perception_unhealthy'"),
        ("            elif not -.05<=final_time-stamp<=.35:reason='dynamic_stale'",
         "            elif not node.native_collision_3d and not -.05<=final_time-stamp<=.35:reason='dynamic_stale'"),
        ("                if code:reason='kernel_error'",
         "                if code:reason=reason if reason!='ok' else 'kernel_error'"),
        ('        solve_obstacles=obs.copy();solve_age=max(0.,now-stamp)',
         '        if node.native_collision_3d and field is None:field=node.native_proposal_field\n        solve_obstacles=obs.copy();solve_age=max(0.,now-stamp)'),
        ('            if session.generation!=generation:reason=\'goal_changed\'',
         '            if node.native_collision_3d and field is None:field=node.native_proposal_field\n            if session.generation!=generation:reason=\'goal_changed\''),
        ('                        max(0.,now-stamp),ptr(field),node.nx,node.ny,node.gx,node.gy,node.res,0,ptr(result))',
         '                        collision_age(node,now,stamp),ptr(field),node.nx,node.ny,node.gx,node.gy,node.res,0,ptr(result))'),
        ('                        ptr(obs),len(obs),max(0.,published-stamp),max(0.,published-odom_stamp),ptr(field),',
         '                        ptr(obs),len(obs),collision_age(node,published,stamp),max(0.,published-odom_stamp),ptr(field),'),
        ('                    published=rospy.get_time()\n                    validation_code=node.lib.online_validate_execution',
         '                    published=rospy.get_time()\n                    clock_error=refresh_validation_clock(node,published)\n                    if clock_error:reason=clock_error;break\n                    validation_code=node.lib.online_validate_execution')):
        text = replace_once(text, old, new)
    old = '''                if node.shared_collision and (collision_payload is None or
                    node.lib.online_set_collision_snapshot(node.handle,ptr(collision_payload),len(collision_payload))):
                    reason='shared_collision_binding_failed' '''.rstrip()
    new = '''                binding_error=bind_collision(node,published,'validation')
                if binding_error:reason=binding_error'''
    text = replace_once(text, old, new)
    text = replace_once(text, "            if reason!='ok':result[:2]=0;validated_count=0",
        "            native_final_error=publication_reason(node,final_time)\n            if native_final_error:reason=native_final_error\n            if reason!='ok':result[:2]=0;validated_count=0")
    text = replace_once(text, '        node.lib.online_feedback(node.handle,result[0],result[1])',
        '        record_cycle(node,published-1000,reason)\n        node.lib.online_feedback(node.handle,result[0],result[1])')
    (runtime / 'local_execution.py').write_text(text)
    # A 2-D projection overlap remains a reported metric, not a 3-D truth stop.
    # Apply identically to both groups: actors and actual integration unchanged.
    text = (runtime / 'simulator.py').read_text()
    if parity:
        # Separate diagnostic initial pose near a wall supplies both clear and
        # colliding native parity poses. Formal navigation trials stay at 0,0.
        text = replace_once(text, '        self.s=np.zeros(5);self.command=np.zeros(2);',
            '        self.s=np.array([0.,1.5,0.,0.,0.]);self.command=np.zeros(2);')
    text = replace_once(text, "            if self.scene in CASES and self.collided:reason='collision';break",
        "            if self.native_truth_collision:reason='native3d_sample_collision';break")
    anchor = "        self.offset=np.array([rospy.get_param('/remani_planner_node/grid_map/lidar_offset_'+a) for a in 'xyz'])"
    text = replace_once(text, anchor, anchor+'\n        from native_collision_runtime import configure_truth_monitor\n        configure_truth_monitor(self)')
    anchor = '        c=self.limits;h=.5*c[\'track\']'
    text = replace_once(text, anchor, '        if self.native_truth_body is None:u=np.zeros(2) # evaluation geometry unavailable: fail closed\n'+anchor)
    anchor = "            if getattr(self,'scene',None) in CASES:"
    text = replace_once(text, anchor, '            from native_collision_runtime import check_truth\n            check_truth(self,self.t-dt+(kstep+1)*step)\n'+anchor)
    anchor = "        (self.out/'result.json').write_text(json.dumps(result,indent=2)+'\\n')"
    text = replace_once(text, anchor, "        result['native3d_truth_monitor']={'sample_collision':self.native_truth_collision,'min_physical_sample_gap':self.native_truth_min_gap if self.native_truth_checks else None,'checks':self.native_truth_checks,'error':self.native_truth_error,'geometry_available':self.native_truth_body is not None,'scope':'independent analytic sphere vs finite cylinder/wall truth, <=5ms integration substeps; not full mesh or continuous proof'}\n"+anchor)
    (runtime / 'simulator.py').write_text(text)
    source = Path(__file__).resolve()
    if source != runtime / source.name:
        shutil.copy2(source, runtime / source.name)
    tree = ET.parse(out / 'experiment.launch')
    launch = tree.getroot()
    found = []
    for node in launch.findall('node'):
        if node.get('name') in ('base_mppi', 'remani_planner_node'):
            found.append(node.get('name'))
            ET.SubElement(node, 'env', name='NATIVE_COLLISION_3D', value=str(int(enabled)))
            ET.SubElement(node, 'env', name='NATIVE_COLLISION_3D_EXPORT', value='1')
            ET.SubElement(node, 'env', name='NATIVE_COLLISION_PARITY', value=str(int(parity)))
            ET.SubElement(node, 'env', name='DELIVERY_REPLAY_KIND', value='')
            ET.SubElement(node, 'env', name='FEASIBILITY_STOP_GATE', value='0')
            if node.get('name') == 'base_mppi':
                ET.SubElement(node, 'param', name='native_collision_3d', value=str(bool(enabled)).lower(), type='bool')
    if sorted(found) != ['base_mppi', 'remani_planner_node']:
        raise ValueError('native3d_expected_exact_planner_and_mppi_nodes')
    tree.write(out / 'experiment.launch', encoding='unicode')
    path = out / 'manifest.json'
    manifest = json.loads(path.read_text())
    for name in ('native_collision_runtime.py', 'mppi_node.py', 'local_execution.py', 'simulator.py'):
        manifest['experiment_sources_sha256'][name] = hashlib.sha256((runtime / name).read_bytes()).hexdigest()
    manifest['native_collision_3d'] = {'enabled': bool(enabled), 'topic': TOPIC,
        'native_environment': int(enabled), 'coupling_weights': [0., 0.], 'fallback_2d': False if enabled else None,
        'legacy_2d_frame': 'proposal actors and visualization only' if enabled else 'unchanged collision authorization',
        'binding': 'solve and latest-frame prepublication; final freshness recheck',
        'sim_projection_collision_stop': False, 'sim_projection_metric_retained': True,
        'independent_3d_truth_audit': 'online <=5ms substep sphere/cylinder/wall stop plus post-run sampled audit; not full mesh or continuous certificate',
        'source_snapshot_patched_only': True}
    manifest['native_collision_3d']['preflight_only_initial_y_m'] = 1.5 if parity else None
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')

