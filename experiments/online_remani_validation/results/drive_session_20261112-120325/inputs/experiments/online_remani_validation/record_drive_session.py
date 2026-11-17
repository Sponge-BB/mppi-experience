#!/usr/bin/env python3
"""Bounded subscriber-only evidence capture across startup and multiple goals.

Bag timestamps are receipt times. Serialized messages retain their source stamps.
No goals, actuator messages, enables or other control messages are published.
"""
import collections
import datetime
import hashlib
import json
import math
import os
import shutil
import signal
import struct
import threading
import time
from pathlib import Path

import rosbag
import rospy
from rospy import AnyMsg
from std_msgs.msg import String

MIB = 1024 ** 2
WORKSPACE = Path('/home/l/remani_ws')
OUTROOT = WORKSPACE / 'experiments/online_remani_validation/results'
SMALL = ('/move_base_simple/goal', '/mppi/status', '/rosout_agg', '/cmd_vel',
         '/odom', '/joint_states_single', '/mm/joint_states', '/joint_ctrl_single',
         '/planning/trajectory', '/planning/finish', '/planning/start',
         '/planning/safety_speed_limit', '/mm_controller_node/trajectory_progress',
         '/mm_controller_node/base_tracking_error', '/mm_controller_node/arm_tracking_error',
         '/tracking_fault', '/tf', '/tf_static', '/dynamic_points',
         '/piper_execution_state', '/arm_status', '/piper_motor_enable_states',
         '/mppi/prediction_rollouts')
CLOUDS = ('/cloud_registered', '/velodyne_points', '/velodyne_points_mapping')
SNAPSHOT = '/mppi/collision_snapshot'


def array_header(raw):
    """Read Float64MultiArray framing and 32 header doubles without array copies."""
    dims = struct.unpack_from('<I', raw)[0]
    if dims > 8:
        raise ValueError('too many dimensions')
    offset = 4
    for _ in range(dims):
        length = struct.unpack_from('<I', raw, offset)[0]
        offset += 4 + length + 8
    offset += 4
    count = struct.unpack_from('<I', raw, offset)[0]
    offset += 4
    if count < 32 or len(raw) != offset + count * 8:
        raise ValueError('invalid snapshot length')
    header = struct.unpack_from('<32d', raw, offset)
    if header[:2] != (3., 1.) or not all(math.isfinite(v) for v in header):
        raise ValueError('invalid snapshot schema/header')
    shape = [int(v) for v in header[5:8]]
    if (min(shape) < 2 or shape != list(header[5:8]) or
            math.prod(shape) > 500000 or header[20] not in (0., 1.) or
            header[11] <= 0 or header[12] < 1 or header[13] < 0 or
            int(header[12]) != header[12] or int(header[13]) != header[13]):
        raise ValueError('invalid snapshot shape')
    prefix = 32 + 7 * (int(header[12]) + int(header[13]))
    if prefix < 32 or count != prefix + (math.prod(shape) if header[20] else 0):
        raise ValueError('snapshot payload does not match header')
    return header


def field_key(header):
    return tuple(header[i] for i in (5, 6, 7, 8, 9, 10, 11, 14))


def json_text(value):
    return json.dumps(value, ensure_ascii=False, separators=(',', ':'), allow_nan=False)


class Recorder:
    def __init__(self):
        label = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
        self.out = OUTROOT / ('drive_session_' + label.strftime('%Y%m%d-%H%M%S'))
        self.out.mkdir(parents=True, exist_ok=False)
        (self.out / 'map_events').mkdir()
        self.lock = threading.RLock()
        self.bag = rosbag.Bag(str(self.out / 'capture.bag'), 'w', chunk_threshold=MIB)
        self.snapshot_log = (self.out / 'snapshot_headers.jsonl').open('w')
        self.event_log = (self.out / 'events.jsonl').open('w')
        self.types, self.subscribers = {}, []
        self.counts, self.drops, self.states = (collections.Counter() for _ in range(3))
        self.source_seconds = {t: -1 for t in CLOUDS}
        self.packets, self.fields = collections.OrderedDict(), collections.OrderedDict()
        self.cache_bytes = self.raw_bytes = self.map_bytes = self.aux_bytes = 0
        self.goals = self.map_events = self.errors = 0
        self.saved_event_keys = set()
        self.pending_maps = collections.OrderedDict()
        self.began = time.monotonic()
        self.done = False
        self.reason = None
        self.last_state = None
        self.active_seen = self.params_captured = False
        self.peak_rss_mib = 0.
        for topic in SMALL + CLOUDS + (SNAPSHOT,):
            self.subscribers.append(rospy.Subscriber(topic, AnyMsg, self.receive,
                callback_args=topic, queue_size=1,
                buff_size=4*MIB if topic in CLOUDS or topic == SNAPSHOT else MIB,
                tcp_nodelay=True))
        print('READY read-only drive recording: ' + str(self.out), flush=True)
        self.save_context()

    def stop(self, reason):
        self.done = True
        self.reason = self.reason or reason

    def total_bytes(self):
        # Include one buffered bag chunk plus space for indexes on close.
        return self.bag.size + MIB + self.map_bytes + self.aux_bytes

    def append_json(self, handle, row):
        encoded = json_text(row) + '\n'
        if self.total_bytes() + len(encoded.encode('utf-8')) > 248*MIB:
            self.stop('total_256MiB_reserve_for_close')
            return
        handle.write(encoded)
        self.aux_bytes += len(encoded.encode('utf-8'))

    def event(self, kind, **values):
        self.append_json(self.event_log, dict(receive_ros=rospy.get_time(),
            elapsed_s=time.monotonic()-self.began, kind=kind, **values))

    def bag_write(self, topic, raw, connection, stamp):
        if topic in CLOUDS and self.bag.size + len(raw) > 192*MIB:
            self.drops['cloud_after_bag_192MiB'] += 1
            return
        if self.total_bytes() + len(raw) > 248*MIB:
            if topic in CLOUDS:
                self.drops['cloud_total_budget'] += 1
                return
            self.stop('total_256MiB_reserve_for_close')
            return
        key = (connection['type'], connection['md5sum'])
        cls = self.types.get(key)
        if cls is None:
            cls = type('RawRecordedType', (), dict(_type=key[0], _md5sum=key[1],
                _full_text=connection.get('message_definition', '')))
            self.types[key] = cls
        self.bag.write(topic, (key[0], raw, key[1], cls), stamp,
            raw=True, connection_header=connection)
        self.raw_bytes += len(raw)
        self.counts[topic] += 1

    def trim_cache(self):
        while len(self.packets) > 16:
            self.packets.popitem(last=False)
        while len(self.fields) > 6:
            self.fields.popitem(last=False)
        def used():
            unique = {id(v[0]): len(v[0]) for v in
                      list(self.packets.values()) + list(self.fields.values())}
            return sum(unique.values())
        self.cache_bytes = used()
        while self.cache_bytes > 32*MIB and (self.packets or self.fields):
            if self.packets:
                self.packets.popitem(last=False)
            else:
                self.fields.popitem(last=False)
            self.cache_bytes = used()

    def snapshot(self, raw, connection, stamp):
        h = array_header(raw)
        item = (raw, h, stamp.to_sec(), connection)
        self.packets[int(h[2])] = item
        if h[20] == 1:
            key = field_key(h)
            self.fields.pop(key, None)
            self.fields[key] = item
        self.trim_cache()
        self.append_json(self.snapshot_log, dict(receive_ros=stamp.to_sec(),
            header=h, bytes=len(raw)))
        self.counts[SNAPSHOT] += 1
        self.resolve_failure_maps()

    def queue_failure_map(self, status):
        h = status.get('candidate_diagnostics', {}).get('snapshot_header')
        if isinstance(h, list) and len(h) == 32:
            key = (int(h[2]), int(h[14]))
            if key not in self.saved_event_keys and len(self.pending_maps) < 8:
                self.pending_maps.setdefault(key, (time.monotonic(), status))
            self.resolve_failure_maps()

    def resolve_failure_maps(self):
        # Independent TCPROS streams can deliver status before its snapshot.
        # Resolve as soon as both arrive; report an explicit miss after 1 s.
        for key, (began, status) in list(self.pending_maps.items()):
            h = status['candidate_diagnostics']['snapshot_header']
            packet = self.packets.get(key[0])
            exact = packet is not None and int(packet[1][14]) == key[1]
            full = self.fields.get(field_key(h))
            if (exact and (packet[1][20] == 1 or full is not None)) or time.monotonic()-began >= 1.:
                self.dump_failure_map(status)
                self.pending_maps.pop(key, None)

    def dump_failure_map(self, status):
        h = status.get('candidate_diagnostics', {}).get('snapshot_header')
        if not isinstance(h, list) or len(h) != 32:
            return
        seq, version = int(h[2]), int(h[14])
        key = (seq, version)
        if key in self.saved_event_keys or self.map_events >= 8:
            return
        self.saved_event_keys.add(key)
        self.map_events += 1
        prefix = 'event%02d_seq%d_version%d' % (self.map_events, seq, version)
        packet = self.packets.get(seq)
        if packet is not None and int(packet[1][14]) != version:
            packet = None
        full = self.fields.get(field_key(h))
        if packet is not None and packet[1][20] == 1:
            full = packet
        items = []
        if full is not None:
            items.append(('full', full))
        if packet is not None and packet is not full:
            items.append(('packet', packet))
        needed = sum(len(v[0]) for _, v in items)
        row = dict(state=status.get('state'), requested_header=h,
            exact_packet_available=packet is not None,
            matching_full_available=full is not None, files=[])
        if self.map_bytes + needed > 64*MIB or self.total_bytes()+needed > 248*MIB:
            row['missing'] = 'map_dump_budget'
            self.drops['map_dump_budget'] += 1
        else:
            for label, item in items:
                raw, saved_header, receive_ros, connection = item
                name = prefix + '_' + label + '.msg'
                (self.out / 'map_events' / name).write_bytes(raw)
                self.map_bytes += len(raw)
                row['files'].append(dict(path='map_events/'+name,
                    header=saved_header, receive_ros=receive_ros,
                    sha256=hashlib.sha256(raw).hexdigest(), connection_header=connection))
            if packet is None or full is None:
                row['missing'] = 'exact_packet_or_matching_full_not_in_bounded_cache'
        self.event('failure_map', **row)

    def receive(self, msg, topic):
        if self.done:
            return
        try:
            raw, connection, stamp = msg._buff, msg._connection_header, rospy.Time.now()
            with self.lock:
                if self.done:
                    return
                if topic == SNAPSHOT:
                    self.snapshot(raw, connection, stamp)
                    return
                if topic in CLOUDS:
                    if len(raw) < 12:
                        raise ValueError('cloud header too short')
                    _, sec, nsec = struct.unpack_from('<III', raw)
                    if nsec >= 1000000000:
                        raise ValueError('cloud source stamp invalid')
                    # All three branches use the same source-second selector;
                    # original exact nanosecond stamps are kept in the bag.
                    if sec <= self.source_seconds[topic]:
                        self.drops['cloud_1Hz_'+topic] += 1
                        return
                    self.source_seconds[topic] = sec
                self.bag_write(topic, raw, connection, stamp)
                if topic == '/move_base_simple/goal':
                    self.goals += 1
                    self.event('goal', number=self.goals)
                elif topic == '/mppi/status':
                    self.active_seen = True
                    status = json.loads(String().deserialize(raw).data)
                    state = str(status.get('state', 'unknown'))
                    self.states[state] += 1
                    if state != self.last_state:
                        self.event('mppi_state', state=state,
                            cycle_ms=status.get('cycle_ms'))
                        self.last_state = state
                    if any(reason in state for reason in ('no_feasible_candidate',
                            'command_hold_or_braking_rejected',
                            'startup_joint_path_obstacle_rejected',
                            'startup_joint_path_configuration_rejected')):
                        self.queue_failure_map(status)
        except Exception as exc:
            with self.lock:
                self.errors += 1
                if self.errors <= 20:
                    self.event('capture_error', topic=topic,
                        error=type(exc).__name__+': '+str(exc))

    def save_context(self):
        context = {}
        names = ('src/mm_remani/config/mppi.yaml', 'src/mm_remani/config/real_speed.yaml',
                 'src/mm_remani/launch/exp0_real.launch',
                 'src/mm_remani/scripts/whole_body_mppi.py',
                 'src/mm_remani/scripts/velodyne_transformer.py',
                 'src/mm_remani_planner/plan_manage/config/piper_param.yaml',
                 'src/mm_remani_planner/plan_manage/config/fixed_collision_spheres_b.yaml',
                 'src/mm_remani/maps/exp0_planner.pcd',
                 'devel/lib/remani_planner/remani_planner_node', 'rollback/VERSIONS.tsv',
                 'experiments/online_remani_validation/record_drive_session.py')
        for name in names:
            path = WORKSPACE / name
            if not path.is_file():
                context[name] = dict(missing=True)
                continue
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda: stream.read(MIB), b''):
                    digest.update(block)
            context[name] = dict(sha256=digest.hexdigest(), size=path.stat().st_size)
            if path.suffix in ('.yaml', '.launch', '.py', '.tsv'):
                destination = self.out / 'inputs' / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(str(path), str(destination))
        data = json_text(context)+'\n'
        (self.out/'code_and_inputs.json').write_text(data)
        self.aux_bytes += len(data.encode('utf-8'))
        self.aux_bytes += sum(p.stat().st_size for p in (self.out/'inputs').rglob('*') if p.is_file())

    def save_parameters(self):
        params = {}
        for name in ('/whole_body_mppi', '/remani_planner_node', '/robot_self_filter',
                     '/velodyne_transformer', '/global_map_publisher'):
            params[name] = rospy.get_param(name, {})
        data = json.dumps(params, ensure_ascii=False, indent=2)+'\n'
        path = self.out/'runtime_parameters.json'
        self.aux_bytes += len(data.encode('utf-8')) - (path.stat().st_size if path.exists() else 0)
        path.write_text(data)
        self.params_captured = self.active_seen

    def resource_status(self):
        mem = {}
        with open('/proc/meminfo') as stream:
            for line in stream:
                key, value = line.split(':', 1)
                if key == 'MemAvailable':
                    mem['host_available_mib'] = int(value.split()[0])/1024.
                    break
        with open('/proc/self/status') as stream:
            for line in stream:
                if line.startswith('VmRSS:'):
                    mem['process_rss_mib'] = int(line.split()[1])/1024.
                    break
        self.peak_rss_mib = max(self.peak_rss_mib, mem.get('process_rss_mib', 0.))
        return mem

    def status(self, memory):
        return dict(schema=1, output=str(self.out), pid=os.getpid(), running=not self.done,
            reason=self.reason, elapsed_s=time.monotonic()-self.began, goals=self.goals,
            topic_counts=dict(self.counts), known_drops=dict(self.drops),
            mppi_states=dict(self.states), errors=self.errors,
            bag_bytes=self.bag.size, raw_payload_bytes=self.raw_bytes,
            map_dump_bytes=self.map_bytes, map_events=self.map_events,
            cached_snapshot_bytes=self.cache_bytes, peak_rss_mib=self.peak_rss_mib,
            limits='600s; clouds <=1Hz/source second and disabled after bag 192MiB; '
                'snapshot cache <=32MiB; map dumps <=8 events/64MiB; total <=256MiB',
            caveat='Subscriber/TCPROS drops are not measurable here. Bag timestamps are receipt '
                'times; source stamps retained. Same source-second selection does not guarantee '
                'identical cloud nanosecond stamps. Failure map status explicitly reports exact '
                'packet/full availability. No occupancy diagnostic arrays are subscribed.', **memory)

    def run(self):
        memory = {}
        try:
            while not self.done and not rospy.is_shutdown():
                memory = self.resource_status()
                if memory.get('host_available_mib', 1024.) < 768:
                    self.stop('host_MemAvailable_below_768MiB')
                if memory.get('process_rss_mib', 0.) > 320:
                    self.stop('recorder_RSS_above_320MiB')
                if time.monotonic()-self.began >= 600:
                    self.stop('600s_limit')
                disk = os.statvfs(str(self.out))
                if disk.f_bavail*disk.f_frsize < 256*MIB:
                    self.stop('disk_free_below_256MiB')
                if not self.params_captured:
                    try:
                        self.save_parameters()
                    except Exception as exc:
                        with self.lock:
                            self.event('parameter_snapshot_error', error=str(exc))
                with self.lock:
                    self.resolve_failure_maps()
                    self.snapshot_log.flush()
                    self.event_log.flush()
                    data = json.dumps(self.status(memory), ensure_ascii=False, indent=2)+'\n'
                    temporary = self.out/'status.json.tmp'
                    temporary.write_text(data)
                    temporary.replace(self.out/'status.json')
                time.sleep(1.)
        finally:
            self.done = True
            self.reason = self.reason or 'ROS_shutdown'
            for sub in self.subscribers:
                sub.unregister()
            with self.lock:
                for _, pending in list(self.pending_maps.items()):
                    self.dump_failure_map(pending[1])
                self.pending_maps.clear()
                summary = self.status(memory)
                self.bag.close()
                self.snapshot_log.close()
                self.event_log.close()
                summary['bag_bytes'] = (self.out/'capture.bag').stat().st_size
                summary['total_files_bytes'] = sum(p.stat().st_size for p in self.out.rglob('*') if p.is_file())
                (self.out/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
                print('SAVED ' + str(self.out/'summary.json'), flush=True)


if __name__ == '__main__':
    rospy.init_node('drive_evidence_recorder', anonymous=True,
                    disable_rosout=True, disable_signals=True)
    recorder = Recorder()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: recorder.stop('user_interrupt'))
    recorder.run()
