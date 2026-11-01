#!/usr/bin/env python3
"""One measured snapshot publisher for BOTH native and local collision queries."""
import hashlib
import json
from pathlib import Path
import threading
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Bool,Float64MultiArray
from classified_frame import collision_frame
from simulator import isolation


def pack_snapshot(field,obs,body,stamp,sequence,config,healthy):
    field=np.asarray(field,dtype=np.float64);obs=np.asarray(obs,dtype=np.float64).reshape((-1,6))
    body=np.asarray(body,dtype=np.float64).reshape((-1,3))
    header=[1.,sequence,stamp,field.shape[1],field.shape[0],-7.5,-5.,.05,
            config['margin'],config['comfort'],config['weight'],len(body),len(obs),
            config['max_age'],config['max_prediction'],float(healthy)]
    return np.ascontiguousarray(np.concatenate([header,body.ravel(),obs.ravel(),field.ravel()]))


class Publisher:
    def __init__(self):
        self.out=Path(rospy.get_param('~out'));self.body=np.loadtxt(self.out/'geometry.csv',delimiter=',')
        self.config=json.loads((self.out/'shared_collision_config.json').read_text())
        self.lock=threading.Lock();self.fixed=None;self.healthy=False;self.sequence=0
        self.log=(self.out/'collision_snapshots.jsonl').open('w',buffering=1)
        self.pub=rospy.Publisher('/sim/unified_collision',Float64MultiArray,queue_size=1,latch=True)
        rospy.Subscriber('/map_generator/global_cloud',PointCloud2,self.map,queue_size=1)
        rospy.Subscriber('/dynamic_obstacle_tracker/perception_healthy',Bool,self.health,queue_size=1)
        rospy.Subscriber('/sim/classified_cloud',PointCloud2,self.cloud,queue_size=1)

    def health(self,m):
        with self.lock:self.healthy=m.data

    def map(self,m):
        if m.header.frame_id=='world':
            fixed=np.asarray(list(pc2.read_points(m,field_names=['x','y','z'],skip_nans=True)),dtype=np.float64).reshape((-1,3))
            with self.lock:self.fixed=fixed

    def cloud(self,m):
        try:
            with self.lock:fixed=self.fixed;healthy=self.healthy
            if fixed is None or m.header.frame_id!='world':return
            stamp=m.header.stamp.to_sec();now=rospy.get_time()
            if not -.05<=now-stamp<=.35:raise ValueError('classified input stale')
            names=['x','y','z','vx','vy','vz','confidence','track_id','dynamic']
            points=np.asarray(list(pc2.read_points(m,field_names=names,skip_nans=False)),dtype=np.float64).reshape((-1,9))
            field,obs,stats=collision_frame(points,fixed)
            self.sequence+=1
            payload=pack_snapshot(field,obs,self.body,stamp,self.sequence,self.config,healthy)
            message=Float64MultiArray();message.data=payload.tolist();self.pub.publish(message)
            # Timing trial: publish the identical payload, omit input-replay dumps.
            self.log.write(json.dumps(dict(sequence=self.sequence,t=now-1000,stamp=stamp,
                healthy=healthy,**stats))+'\n')
        except (ValueError,KeyError) as exc:
            message=Float64MultiArray();self.pub.publish(message)
            self.log.write(json.dumps(dict(t=rospy.get_time()-1000,error=str(exc)))+'\n')


if __name__=='__main__':
    isolation();rospy.init_node('shared_collision_model')
    if not rospy.get_param('/online_validation/enabled',False):raise RuntimeError('missing SIL interlock')
    Publisher();rospy.spin()
