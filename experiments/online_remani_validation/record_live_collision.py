#!/usr/bin/env python3
"""Read-only, bounded ROS1 evidence recorder; source ROS setup externally."""
import collections, datetime, hashlib, json, math, os, resource, shutil, signal, struct, sys, threading, time
from pathlib import Path
import rosbag, rospy
from rospy import AnyMsg
from std_msgs.msg import String, Bool
from rosgraph_msgs.msg import Log

MIB = 1024**2
ROOT = Path('/home/l/remani_ws/experiments/online_remani_validation/results')
CLOUDS = ('/livox/lidar', '/livox/lidar_pcl2', '/cloud_registered',
          '/livox/lidar_no_ground', '/velodyne_points')
SMALL = ('/move_base_simple/goal', '/mppi/status', '/rosout_agg', '/cmd_vel',
         '/odom', '/joint_states_single', '/mm/joint_states', '/planning/trajectory',
         '/planning/finish', '/planning/start', '/planning/safety_speed_limit',
         '/mm_controller_node/trajectory_progress', '/tracking_fault', '/tf', '/tf_static',
         '/dynamic_points', '/piper_execution_state')
DIAG = '/mppi/collision_diagnostic'
GEOMETRY = ('no_feasible_candidate', 'command_hold_or_braking_rejected',
            'startup_joint_path_obstacle_rejected', 'startup_joint_path_configuration_rejected')


def source_stamp(topic, raw):
    if topic not in CLOUDS or len(raw) < 16:
        return None
    _, sec, nsec = struct.unpack_from('<III', raw)
    return sec * 1000000000 + nsec if nsec < 1000000000 else None


def inspect_diagnostic(raw):
    """Read array framing/header without making Python lists of the voxel arrays."""
    off = 4
    dimensions = struct.unpack_from('<I', raw)[0]
    if dimensions > 8:
        raise ValueError('layout dimension count')
    for _ in range(dimensions):
        length = struct.unpack_from('<I', raw, off)[0]
        off += 4 + length + 8
        if off > len(raw):
            raise ValueError('layout overflow')
    off += 4  # layout.data_offset
    count = struct.unpack_from('<I', raw, off)[0]
    off += 4
    if count < 40 or len(raw) != off + 8 * count:
        raise ValueError('serialized length')
    h = struct.unpack_from('<40d', raw, off)
    n = int(h[38])
    if (h[0:2] != (3., 1.) or h[32:34] != (20261112., 1.) or
            not all(math.isfinite(v) for v in h) or not 0 < n <= 500000 or
            h[38] != n or count != 40 + 2*n or h[29] != h[36] or
            any(v < 2 or int(v) != v for v in h[5:8]) or
            int(h[5]*h[6]*h[7]) != n or h[11] <= 0):
        raise ValueError('schema/shape/generation mismatch')
    values = memoryview(raw)[off+320:]
    if sys.byteorder != 'little':
        raise ValueError('recorder requires little-endian host')
    data = values.cast('d')
    if not all(math.isfinite(v) for v in data[:n]):
        raise ValueError('nonfinite ESDF')
    if not all(v in (0., 1., 2., 3.) for v in data[n:]):
        raise ValueError('invalid occupancy bits')
    return dict(header=list(h), cells=n, coherent=True,
                key=[int(h[i]) for i in (14, 29, 34, 35, 36)])


class Recorder:
    def __init__(self):
        now = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
        self.out = ROOT / ('collision_capture_' + now.strftime('%Y%m%d-%H%M%S'))
        self.out.mkdir(parents=True, exist_ok=False)
        workspace = ROOT.parents[2]
        context = {}
        for name in ('src/mm_remani/config/mppi.yaml', 'src/mm_remani/config/real_speed.yaml',
                     'src/mm_remani_planner/plan_manage/config/piper_param.yaml',
                     'src/mm_remani_planner/plan_manage/config/fixed_collision_spheres_b.yaml',
                     'src/mm_remani/maps/exp0_planner.pcd',
                     'devel/lib/remani_planner/remani_planner_node'):
            path = workspace/name
            context[name] = hashlib.sha256(path.read_bytes()).hexdigest()
            if path.suffix in ('.yaml', '.pcd'):
                destination = self.out/'inputs'/name
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(str(path), str(destination))
        (self.out/'code_and_inputs.json').write_text(json.dumps(context, indent=2))
        self.lock = threading.RLock()
        self.bag = rosbag.Bag(str(self.out/'capture.bag'), 'w', chunk_threshold=MIB)
        self.types, self.subscribers = {}, []
        self.pre = collections.deque()
        self.pre_bytes = self.written = 0
        self.counts, self.drops = collections.Counter(), collections.Counter()
        self.stamps = {t: set() for t in CLOUDS}
        self.diagnostics, self.events, self.errors = [], [], []
        self.began, self.goal, self.event = time.monotonic(), None, None
        self.done, self.reason, self.diag_started = False, None, False
        self.last_disk_check = 0.
        for topic in SMALL + CLOUDS:
            self.subscribe(topic)
        print('READY read-only subscriptions registered; awaiting your navigation goal. ' + str(self.out), flush=True)

    def subscribe(self, topic):
        self.subscribers.append(rospy.Subscriber(topic, AnyMsg, self.receive,
                                callback_args=topic, queue_size=1,
                                buff_size=8*MIB if topic in CLOUDS or topic == DIAG else MIB,
                                tcp_nodelay=True))

    def stop(self, reason):
        self.done, self.reason = True, self.reason or reason

    def write(self, topic, raw, header, stamp):
        if self.done:
            self.drops['after_stop'] += 1
            return
        if len(raw) + self.bag.size > 248*MIB:  # reserve close/index overhead
            self.drops['bag_size_limit'] += 1
            self.stop('bag_256MiB_limit')
            return
        if time.monotonic() - self.last_disk_check > 1:
            self.last_disk_check = time.monotonic()
            stat = os.statvfs(str(self.out))
            if stat.f_bavail * stat.f_frsize < 256*MIB:
                self.stop('disk_free_below_256MiB')
                return
        key = (header['type'], header['md5sum'])
        cls = self.types.get(key)
        if cls is None:
            cls = type('RawRecordedType', (), dict(_type=key[0], _md5sum=key[1],
                       _full_text=header.get('message_definition', '')))
            self.types[key] = cls
        self.bag.write(topic, (key[0], raw, key[1], cls), stamp,
                       raw=True, connection_header=header)
        self.counts[topic] += 1
        self.written += len(raw)
        source = source_stamp(topic, raw)
        if source is not None:
            self.stamps[topic].add(source)

    def detect(self, topic, raw):
        reason = None
        if topic == '/mppi/status':
            status = json.loads(String().deserialize(raw).data)
            state = str(status.get('state', ''))
            if any(token in state for token in GEOMETRY):
                reason = state
        elif topic == '/tracking_fault' and Bool().deserialize(raw).data:
            reason = 'tracking_fault_true'
        elif topic == '/rosout_agg':
            msg = Log().deserialize(raw)
            low = msg.msg.lower()
            if (msg.level >= Log.WARN and any(token in low for token in
                    ('collision type=', 'collision_type=', 'body collision',
                     'base collision', 'tracker fault', 'collision check failed',
                     'collision detected', 'collision safety', 'checkcarobscollision failed',
                     'car collision', 'mani collision', 'current traj in collision'))):
                reason = msg.name + ': ' + msg.msg[:500]
        if reason and self.event is None:
            self.event = time.monotonic()
            self.events.append(dict(receive_ros=rospy.get_time(), reason=reason))
            print('EVENT ' + reason, flush=True)

    def receive(self, msg, topic):
        raw, stamp = msg._buff, rospy.Time.now()
        header = dict(msg._connection_header)
        mono = time.monotonic()
        with self.lock:
            if self.done:
                return
            try:
                if topic == '/move_base_simple/goal' and self.goal is None:
                    self.goal = mono
                    print('GOAL received; recording active, maximum 40 seconds.', flush=True)
                    while self.pre:
                        when, t, r, h, ts = self.pre.popleft()
                        if mono-when <= 2.:
                            self.write(t, r, h, ts)
                        else:
                            self.drops['preroll_expired'] += 1
                    self.pre_bytes = 0
                if topic in CLOUDS and self.goal is None:
                    if len(raw) > 8*MIB:
                        self.drops['oversize_cloud'] += 1
                        return
                    self.pre.append((mono, topic, raw, header, stamp))
                    self.pre_bytes += len(raw)
                    while self.pre and (mono-self.pre[0][0] > 2. or self.pre_bytes > 24*MIB):
                        self.pre_bytes -= len(self.pre.popleft()[2])
                        self.drops['preroll_evicted'] += 1
                    return
                if len(raw) > 9*MIB:
                    self.drops['oversize_message'] += 1
                    return
                if topic == DIAG:
                    try:
                        row = inspect_diagnostic(raw)
                        row['receive_ros'] = stamp.to_sec()
                        self.diagnostics.append(row)
                    except (ValueError, struct.error, OverflowError) as exc:
                        self.errors.append('diagnostic: ' + str(exc))
                if self.goal is not None and topic in ('/mppi/status', '/tracking_fault', '/rosout_agg'):
                    self.detect(topic, raw)
                self.write(topic, raw, header, stamp)
            except Exception as exc:
                self.errors.append(type(exc).__name__ + ': ' + str(exc))
                self.stop('recording_error')

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                if self.done:
                    break
                now = time.monotonic()
                if self.goal is None and now-self.began >= 600:
                    self.stop('no_goal_10min_timeout')
                elif self.goal is not None:
                    if not self.diag_started:
                        self.diag_started = True
                        self.subscribe(DIAG)
                        print('DIAGNOSTIC subscribed; same-generation capture requested.', flush=True)
                    if now-self.goal >= 40:
                        self.stop('active_40sec_complete')
                    elif self.event is not None and now-self.goal >= 10 and now-self.event >= 3:
                        self.stop('event_plus_3sec_minimum_10sec_complete')
            time.sleep(.05)
        with self.lock:
            self.done = True
            self.reason = self.reason or 'ROS_shutdown_or_interrupt'
        for sub in self.subscribers:
            sub.unregister()
        with self.lock:
            self.bag.close()
            matching = {}
            for left, right in zip(CLOUDS[2:], CLOUDS[3:]):
                matching[left+' => '+right] = len(self.stamps[left] & self.stamps[right])
            summary = dict(schema=1, reason=self.reason, output=str(self.out),
                elapsed_s=time.monotonic()-self.began, goal_received=self.goal is not None,
                recorded_topics=dict(self.counts), known_drops=dict(self.drops), errors=self.errors,
                raw_payload_bytes=self.written, bag_bytes=(self.out/'capture.bag').stat().st_size,
                source_stamp_counts={t: len(s) for t,s in self.stamps.items()},
                matching_cloud_source_stamps=matching, diagnostic_frames=len(self.diagnostics),
                diagnostics=self.diagnostics, events=self.events,
                peak_rss_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024.,
                limits='24MiB/2s preroll, subscriber queue=1, active<=40s, bag<=256MiB',
                caveat='TCPROS/subscriber drops are not observable here; known_drops excludes them. '
                       'Bag time is receive ROS time; original serialized source stamps are preserved.')
            (self.out/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
            print('SAVED ' + str(self.out/'summary.json'), flush=True)


if __name__ == '__main__':
    rospy.init_node('collision_evidence_recorder', anonymous=True, disable_rosout=True, disable_signals=True)
    recorder = Recorder()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: recorder.stop('user_interrupt'))
    recorder.run()
