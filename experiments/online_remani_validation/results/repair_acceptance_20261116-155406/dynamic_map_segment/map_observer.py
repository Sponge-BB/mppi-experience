#!/usr/bin/python3
"""Bounded read-only collision snapshot companion; --self-test needs no ROS."""
import collections
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import struct
import sys
import threading
import time

MIB = 1024**2
OUT = Path(__file__).resolve().parent
SNAPSHOT = '/mppi/collision_snapshot'
TOPICS = (SNAPSHOT, '/mppi/status', '/rosout_agg', '/move_base_simple/goal')


def array_header(raw):
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
    if count < 32 or len(raw) != offset + count*8:
        raise ValueError('invalid snapshot length')
    h = struct.unpack_from('<32d', raw, offset)
    if h[:2] != (3., 1.) or not all(math.isfinite(v) for v in h):
        raise ValueError('invalid schema/header')
    shape = [int(v) for v in h[5:8]]
    if (min(shape) < 2 or shape != list(h[5:8]) or math.prod(shape) > 500000 or
            h[20] not in (0., 1.) or h[11] <= 0 or h[12] < 1 or h[13] < 0 or
            int(h[12]) != h[12] or int(h[13]) != h[13]):
        raise ValueError('invalid shape/count')
    prefix = 32 + 7*(int(h[12])+int(h[13]))
    if count != prefix + (math.prod(shape) if h[20] else 0):
        raise ValueError('payload/header mismatch')
    return h


def field_key(h):
    return tuple(h[i] for i in (5, 6, 7, 8, 9, 10, 11, 14))


def packet_key(h):
    return (int(h[2]), int(h[14]))


class Cache:
    def __init__(self):
        self.packets = collections.OrderedDict()
        self.fields = collections.OrderedDict()
        self.latest = None
        self.bytes = 0
        self.evictions = 0

    def used(self):
        return sum({id(v['raw']): len(v['raw']) for v in
                    list(self.packets.values())+list(self.fields.values())}.values())

    def add(self, item):
        h = item['header']; key = packet_key(h)
        self.packets.pop(key, None); self.packets[key] = item
        self.latest = h
        if h[20] == 1.:
            fk = field_key(h)
            self.fields.pop(fk, None); self.fields[fk] = item
        while len(self.packets) > 64:
            self.packets.popitem(last=False); self.evictions += 1
        while len(self.fields) > 12:
            self.fields.popitem(last=False); self.evictions += 1
        while self.used() > 48*MIB:
            if len(self.packets) > 1:
                self.packets.popitem(last=False)
            elif self.fields:
                self.fields.popitem(last=False)
            else:
                self.packets.popitem(last=False)
            self.evictions += 1
        self.bytes = self.used()

    def match(self, h):
        exact = self.packets.get(packet_key(h))
        full = self.fields.get(field_key(h))
        if exact is not None and exact['header'][20] == 1.:
            full = exact
        return exact, full


def self_test():
    h = [0.]*32
    for i, v in {0:3,1:1,2:7,5:2,6:2,7:2,11:.1,12:1,14:9,20:1}.items():
        h[i] = float(v)
    def raw(header):
        count = 39+(8 if header[20] else 0)
        return struct.pack('<III',0,0,count)+struct.pack('<%dd'%count,
            *(header+[0.]*(count-32)))
    c = Cache(); full = {'raw':raw(h),'header':array_header(raw(h))}
    c.add(full); partial = list(h); partial[2]=8.; partial[20]=0.
    packet = {'raw':raw(partial),'header':array_header(raw(partial))};c.add(packet)
    assert c.match(partial)==(packet,full)
    wrong = list(partial);wrong[14]=10.
    assert c.match(wrong)==(None,None)
    try: array_header(raw(h)[:-1])
    except ValueError: pass
    else: raise AssertionError('truncation accepted')
    print('SELF_TEST_PASS exact packet/full-field join, version mismatch, truncation; no ROS')


def run():
    import rosbag  # ROS environment validation only; no bag writer or publishers.
    import rospy
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Log
    from roscpp.srv import GetLoggers, SetLoggerLevel
    cache = Cache(); lock = threading.RLock(); stop = threading.Event()
    counts = collections.Counter(); drops = collections.Counter(); errors = []
    pending = []; total_bytes = 0; saved = 0; goals = 0
    last_event = -1e9; first_stop_saved = False; last_state = None
    began = time.monotonic(); reason = ''; peak_rss = 0.
    logger_original = {}; logger_actions = []; logger_restore = {}; rpc_hung = False
    header_path = OUT/'snapshot_headers.jsonl'; event_path = OUT/'map_events.jsonl'
    # Refuse to overwrite an earlier observation in this acceptance case.
    headers = header_path.open('x', buffering=1)
    events = event_path.open('x', buffering=1)
    (OUT/'map_events').mkdir(exist_ok=True)
    rospy.init_node('repair_map_observer', anonymous=True, disable_signals=True,
                    disable_rosout=True)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())

    def resources():
        available = next(int(x.split()[1])/1024 for x in
            Path('/proc/meminfo').read_text().splitlines() if x.startswith('MemAvailable:'))
        rss = next(int(x.split()[1])/1024 for x in
            Path('/proc/self/status').read_text().splitlines() if x.startswith('VmRSS:'))
        return available, rss

    def logger_call(service, cls, *args):
        nonlocal rpc_hung
        result = []
        def call():
            try:
                rospy.wait_for_service(service, timeout=.15)
                result.append((True, rospy.ServiceProxy(service, cls)(*args)))
            except Exception as exc: result.append((False, str(exc)))
        worker = threading.Thread(target=call, daemon=True);worker.start();worker.join(.5)
        if worker.is_alive():
            rpc_hung = True
            raise TimeoutError('logger RPC timed out; outcome unconfirmed: '+service)
        ok, value = result[0]
        if not ok: raise RuntimeError(value)
        return value

    def request(kind, detail, status=None, parking=False):
        nonlocal last_event, first_stop_saved
        if goals <= 0:
            drops['event_before_first_goal'] += 1;return
        now = time.monotonic()
        priority = parking and not first_stop_saved
        if saved+len(pending) >= 32:
            drops['event_limit'] += 1;return
        if not priority and now-last_event < 2.:
            drops['event_throttle'] += 1;return
        refs = []
        explicit = (status or {}).get('candidate_diagnostics',{}).get('snapshot_header')
        if isinstance(explicit, list) and len(explicit)==32 and all(
                isinstance(v,(int,float)) and math.isfinite(v) for v in explicit):
            refs.append(('status_referenced',tuple(explicit)))
        if cache.latest is not None:
            refs.append(('current_at_trigger',cache.latest))
        pending.append(dict(kind=kind,detail=detail,goal_number=goals,
            first_stop_for_goal=priority,receive_ros=rospy.get_time(),
            receive_monotonic=now,deadline=now+1.,refs=refs))
        last_event=now
        if priority:first_stop_saved=True

    def receive(msg, topic):
        nonlocal goals, first_stop_saved, last_state
        received = rospy.Time.now().to_sec(); mono = time.monotonic()
        try:
            with lock:
                if stop.is_set():return
                counts[topic] += 1
                if topic==SNAPSHOT:
                    raw=msg._buff;h=array_header(raw)
                    cache.add(dict(raw=raw,header=h,receive_ros=received,
                        receive_monotonic=mono,connection=dict(msg._connection_header)))
                    headers.write(json.dumps(dict(receive_ros=received,
                        receive_monotonic=mono,header=h,bytes=len(raw)))+'\n')
                elif topic=='/move_base_simple/goal':
                    goals+=1;first_stop_saved=False
                elif topic=='/mppi/status':
                    status=json.loads(String().deserialize(msg._buff).data)
                    state=str(status.get('state','unknown'))
                    if goals and state!=last_state and state not in ('command_delivered','ok'):
                        parking=any(word in state.lower() for word in (
                            'cancel','hold','expired','deadline','timeout','stale',
                            'unavailable','rejected','invalid','infeasible','no_feasible'))
                        request('mppi_state',state,status,parking=parking)
                    last_state=state
                else:
                    entry=Log().deserialize(msg._buff)
                    if any(s in entry.msg for s in (
                            'LiveCollisionStop','MovingHandoff','Emergency stop!')):
                        parking=('LiveCollisionStop' in entry.msg or
                            'Emergency stop!' in entry.msg or any(word in entry.msg.lower()
                            for word in ('reject','expired','cancel','hold')))
                        request('rosout',dict(node=entry.name,stamp=entry.header.stamp.to_sec(),
                            message=entry.msg),parking=parking)
        except Exception as exc:
            with lock:
                drops['callback_error']+=1
                if len(errors)<32:errors.append(str(exc))

    subscribers=[rospy.Subscriber(t,rospy.AnyMsg,receive,callback_args=t,
        queue_size=8,buff_size=8*MIB,tcp_nodelay=True) for t in TOPICS]

    def save_events():
        nonlocal saved,total_bytes
        now=time.monotonic()
        for event in list(pending):
            matches=[(label,h,*cache.match(h)) for label,h in event['refs']]
            if now<event['deadline'] and (not matches or any(
                    exact is None or full is None for _,_,exact,full in matches)):
                continue
            saved+=1;written={};references=[]
            for label,h,exact,full in matches:
                row=dict(role=label,requested_header=h,exact_packet_available=exact is not None,
                    matching_full_available=full is not None,files=[])
                for role,item in (('packet',exact),('full',full)):
                    if item is None:continue
                    digest=hashlib.sha256(item['raw']).hexdigest()
                    if digest not in written:
                        if total_bytes+len(item['raw'])>256*MIB:
                            drops['disk_event_budget']+=1;row['missing']='event_disk_budget';continue
                        name='map_events/event%02d_%s_%s.msg'%(saved,label,role)
                        (OUT/name).write_bytes(item['raw']);total_bytes+=len(item['raw'])
                        written[digest]=name
                    row['files'].append(dict(role=role,path=written[digest],sha256=digest,
                        header=item['header'],receive_ros=item['receive_ros'],
                        receive_monotonic=item['receive_monotonic'],connection=item['connection']))
                if exact is None or full is None:row['missing']='exact_packet_or_matching_full_not_in_cache'
                references.append(row)
            row={k:v for k,v in event.items() if k not in ('refs','deadline')}
            row.update(event_number=saved,references=references,
                missing='no_snapshot_at_trigger' if not matches else None,
                saved_ros=rospy.get_time())
            events.write(json.dumps(row)+'\n');pending.remove(event)

    print('READY map observer '+str(OUT),flush=True)
    last_status=last_logger=-1e9
    try:
        while not stop.is_set() and not rospy.is_shutdown():
            now=time.monotonic()
            if now-began>=1800:reason='1800s_limit';break
            with lock:save_events()
            if now-last_status>=1.:
                available,rss=resources();peak_rss=max(peak_rss,rss)
                if rss>256:reason='self_RSS_above_256MiB';stop.set()
                if available<768:reason='host_MemAvailable_below_768MiB';stop.set()
                with lock:
                    value=dict(pid=os.getpid(),running=not stop.is_set(),elapsed_s=now-began,
                        counts=dict(counts),known_drops=dict(drops),errors=errors,
                        cache_bytes=cache.bytes,cache_evictions=cache.evictions,
                        saved_events=saved,pending_events=len(pending),event_bytes=total_bytes,
                        goals=goals,peak_rss_mib=peak_rss,rss_mib=rss,
                        host_available_mib=available,reason=reason,logger_original=logger_original,
                        logger_actions=logger_actions,logger_restoration=logger_restore,
                        caveat='TCPROS/subscriber drops are not directly observable; cache evictions and explicit limits are counted. No clouds decoded, no controls/parameters published. Only latency logger services are used.')
                    temp=OUT/'map_observer_status.tmp';temp.write_text(json.dumps(value,indent=2)+'\n')
                    temp.replace(OUT/'map_observer_status.json')
                    headers.flush();events.flush()
                last_status=now
            if now-last_logger>=2. and not rpc_hung:
                last_logger=now
                try:
                    values=logger_call('/remani_planner_node/get_loggers',GetLoggers).loggers
                    for entry in values:
                        if entry.name.endswith('.latency') and entry.level.lower()!='debug':
                            logger_original.setdefault(entry.name,entry.level)
                            logger_call('/remani_planner_node/set_logger_level',SetLoggerLevel,
                                        entry.name,'debug')
                            logger_actions.append(dict(name=entry.name,level='debug',confirmed=True))
                except Exception as exc:
                    if len(logger_actions)<32:logger_actions.append(dict(error=str(exc)))
            stop.wait(.1)
    finally:
        stop.set()
        for subscriber in subscribers:subscriber.unregister()
        with lock:
            for event in pending:event['deadline']=0.
            save_events();headers.close();events.close()
        for name,level in logger_original.items():
            try:
                logger_call('/remani_planner_node/set_logger_level',SetLoggerLevel,name,level)
                logger_restore[name]='unconfirmed_prior_RPC_timeout' if rpc_hung else 'confirmed'
            except Exception as exc:logger_restore[name]='unconfirmed: '+str(exc)
        (OUT/'map_observer_closed.json').write_text(json.dumps(dict(reason=reason or
            'requested_stop_or_ros_shutdown',counts=dict(counts),known_drops=dict(drops),
            saved_events=saved,event_bytes=total_bytes,peak_rss_mib=peak_rss,
            logger_original=logger_original,logger_actions=logger_actions,
            logger_restoration=logger_restore,errors=errors),indent=2)+'\n')
        print('SAVED map observer '+str(OUT),flush=True)


if __name__=='__main__':
    if '--self-test' in sys.argv:self_test()
    else:run()
