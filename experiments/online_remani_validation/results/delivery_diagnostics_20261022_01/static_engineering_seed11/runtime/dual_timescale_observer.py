#!/usr/bin/env python3
"""Read-only SIL timeline observer; never publishes or authorizes motion.

Observer receipt time is not the publisher's send time. Tailed MPPI files have
only native ROS event times; their monotonic timestamp is explicitly the later
observer-read time. Controller progress is advisory when MPPI owns execution.
"""
import argparse
import hashlib
import json
from pathlib import Path
import threading
import time


def fnv64(text):
    value=1469598103934665603
    for byte in text.encode('utf-8'):
        value=((value^byte)*1099511628211)&0xffffffffffffffff
    return str(value)


def transaction_metadata(text):
    parts=text.split('\n')
    result={'transaction_sha256':hashlib.sha256(text.encode()).hexdigest(),
            'transaction_fnv64':fnv64(text),'text_bytes':len(text.encode())}
    try:
        count=int(parts[1])
        if not 1<=count<=1000 or len(parts)!=count+2 or len(text)>4000000:
            raise ValueError('invalid transaction count/length')
        result.update(goal_fnv64=fnv64(parts[0]),goal_sha256=hashlib.sha256(parts[0].encode()).hexdigest(),
                      segments=count,message_hashes=[hashlib.sha256(bytes.fromhex(s)).hexdigest() for s in parts[2:]],
                      valid_framing=True)
    except (ValueError,IndexError) as exc:
        result.update(valid_framing=False,error=str(exc))
    return result


class Observer:
    def __init__(self,out):
        import rospy
        from geometry_msgs.msg import TwistStamped,Vector3Stamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Bool,String
        self.rospy=rospy;self.out=out;self.lock=threading.RLock()
        self.log=(out/'observer_events.jsonl').open('w',buffering=1)
        self.offsets={};self.receipts={};self.commits={};self.adopted=set();self.last={};self.last_odom=0.
        self.subscribers=[
            rospy.Subscriber('/planning/execution_reference',String,self.reference,queue_size=100),
            rospy.Subscriber('/sim/mppi_diagnostics',String,self.diagnostic,queue_size=200),
            rospy.Subscriber('/sim/local_execution_cmd',TwistStamped,self.command,queue_size=200),
            rospy.Subscriber('/mm_controller_node/trajectory_progress',Vector3Stamped,
                lambda m:self.progress(m,'planner_progress_input'),queue_size=200),
            rospy.Subscriber('/sim/native_controller_progress',Vector3Stamped,
                lambda m:self.progress(m,'native_controller_advisory'),queue_size=200),
            rospy.Subscriber('/odom',Odometry,self.odom,queue_size=20),
            rospy.Subscriber('/dynamic_obstacle_tracker/perception_healthy',Bool,
                lambda m:self.emit('perception_health',healthy=bool(m.data)),queue_size=20),
        ]
        for topic in ('/sim/raw_cloud','/velodyne_points','/dynamic_points','/sim/classified_cloud'):
            self.subscribers.append(rospy.Subscriber(topic,PointCloud2,
                lambda m,t=topic:self.cloud(m,t),queue_size=10))
        self.emit('observer_start',scope='read-only subscriptions; no command authority',
                  freeze_interpretation='native virtual-clock hold is not proof of actual MPPI stop',
                  file_timestamps='mono is observer-read time; source_ros is source event time')
        self.timer=rospy.Timer(rospy.Duration(.05),self.tail)
        rospy.on_shutdown(self.close)

    def emit(self,event,**fields):
        with self.lock:
            if self.log.closed:return
            self.log.write(json.dumps(dict(event=event,mono=time.monotonic(),sim=self.rospy.get_time(),**fields),
                                      separators=(',',':'),allow_nan=False)+'\n')

    def reference(self,message):
        meta=transaction_metadata(message.data)
        with self.lock:
            if meta.get('valid_framing'):self.receipts[tuple(meta['message_hashes'])]=meta
            self.emit('reference_observer_receipt',topic='/planning/execution_reference',**meta)

    def diagnostic(self,message):
        try:row=json.loads(message.data)
        except (ValueError,TypeError):self.emit('diagnostic_parse_error');return
        keys=('reason','bundle','goal_generation','reference_sha256','reference_error','reference_points',
              'guide_cancelled','authorized','pending_reference_updates','obstacle_tracks','cloud_age','dynamic_age',
              'applied_v','applied_w','nominal_v','nominal_w','kernel_ms','callback_ms','code','validation_code',
              'stop_unverified','accepted_candidate','candidate_count','fixed_arm_error','execution_authority')
        fields={k:row[k] for k in keys if k in row}
        # Preserve diagnostics that may encode infinity for an unready sensor.
        fields=json.loads(json.dumps(fields),parse_constant=lambda _:None)
        fields['source_ros']=float(row.get('t',0.))+1000.
        fields['command_zero']=abs(float(row.get('applied_v',0)))<1e-6 and abs(float(row.get('applied_w',0)))<1e-6
        with self.lock:
            self.emit('mppi_cycle',**fields)
            key=(row.get('goal_generation'),row.get('bundle'))
            if row.get('reference_sha256') and key not in self.adopted:
                self.adopted.add(key)
                self.emit('reference_first_control_cycle',**fields,
                          meaning='reference used by MPPI control cycle; not proof of nonzero motion')

    def command(self,message):
        self.emit('local_execution_command',topic='/sim/local_execution_cmd',stamp=message.header.stamp.to_sec(),
                  v=message.twist.linear.x,w=message.twist.angular.z)

    def progress(self,message,role):
        self.emit('progress',role=role,frame_id=message.header.frame_id,stamp=message.header.stamp.to_sec(),
                  reference_start=message.vector.x,virtual_time=message.vector.y,time_scale=message.vector.z,
                  is_actual_motion_measurement=False)

    def odom(self,message):
        now=time.monotonic()
        with self.lock:
            if now-self.last_odom<.095:return
            self.last_odom=now
            p=message.pose.pose.position;q=message.pose.pose.orientation
            self.emit('measured_motion',stamp=message.header.stamp.to_sec(),x=p.x,y=p.y,
                      quaternion=[q.x,q.y,q.z,q.w],v=message.twist.twist.linear.x,w=message.twist.twist.angular.z)

    def cloud(self,message,topic):
        count=int(message.width)*int(message.height);now=time.monotonic()
        with self.lock:
            key='cloud:'+topic;old=self.last.get(key)
            if old and now-old[0]<.19 and bool(count)==old[1]:return
            self.last[key]=(now,bool(count))
            self.emit('perception_frame_metadata',topic=topic,stamp=message.header.stamp.to_sec(),
                      points=count,frame_id=message.header.frame_id,sequence=message.header.seq,
                      scope='metadata only; nonempty raw cloud is not necessarily a new obstacle')

    def tail(self,_=None):
        with self.lock:
            for name in ('reference_events.jsonl','consumed_references.jsonl'):
                path=self.out/name
                if not path.exists():continue
                with path.open() as stream:
                    stream.seek(self.offsets.get(name,0))
                    while True:
                        start=stream.tell();line=stream.readline()
                        if not line or not line.endswith('\n'):break
                        self.offsets[name]=stream.tell()
                        try:row=json.loads(line)
                        except ValueError:
                            self.emit('source_file_parse_error',source=name,offset=start);continue
                        hashes=tuple(row.get('message_hashes',[]));receipt=self.receipts.get(hashes,{})
                        fields={k:row[k] for k in ('event','revision','epoch','id','stamp','error','sha256') if k in row}
                        source_event=fields.pop('event',None)
                        self.emit('mppi_reference_commit_observed' if name=='reference_events.jsonl' else 'mppi_reference_consumption_observed',
                                  source=name,source_event=source_event,source_ros=float(row.get('t',0))+1000.,
                                  message_hashes=list(hashes),transaction_sha256=receipt.get('transaction_sha256'),
                                  transaction_fnv64=receipt.get('transaction_fnv64'),**fields)

    def close(self):
        with self.lock:
            if self.log.closed:return
            self.tail();self.emit('observer_stop');self.log.close()


def main():
    import rospy
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True)
    args=p.parse_args(rospy.myargv()[1:])
    rospy.init_node('dual_timescale_observer',anonymous=False)
    Observer(args.out.resolve());rospy.spin()


if __name__=='__main__':main()
