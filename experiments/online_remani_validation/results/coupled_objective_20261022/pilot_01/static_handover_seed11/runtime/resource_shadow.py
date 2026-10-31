#!/usr/bin/env python3
"""V12 compute + execution validation observer; every publisher is isolated."""
import json
import os
from pathlib import Path
import rospy
import local_execution
from mppi_node import Node
from native_reference import NativeReference
from simulator import isolation


class ObserverSession(local_execution.ExecutionSession):
    def authority(self):
        if self.invalid:return self.invalid
        return 'ok' if self.guide is not None else 'waiting_native_trajectory'


class ObserverNode(Node):
    def trajectory(self,message):
        # Retain the last guide across native aborts to keep measuring background
        # workload. No computed control or progress can reach the native graph.
        if message.action in (2,5):return
        with self.lock:
            event=self.native.feed(message,rospy.get_time())
            if event not in ('replace','append') or not hasattr(self,'execution'):return
            self.execution.guide=self.native
            self.execution.revision+=1;self.native.revision=self.execution.revision
            self.execution.invalid=None


def main():
    isolation();rospy.init_node('background_mppi')
    if not rospy.get_param('/online_validation/enabled',False):raise RuntimeError('missing simulation interlock')
    original_publisher=rospy.Publisher;publishers={}
    def isolated_publisher(name,*args,**kwargs):
        target='/resource_shadow'+('/'+name.lstrip('/'))
        publishers[name]=target
        return original_publisher(target,*args,**kwargs)
    rospy.Publisher=isolated_publisher
    local_execution.ExecutionSession=ObserverSession
    node=ObserverNode()  # local_execution_owner=false subscribes native polynomials.
    local_execution.configure(node)
    (node.out/'mppi_pid.txt').write_text(str(os.getpid())+'\n')
    (node.out/'shadow_publishers.json').write_text(json.dumps(publishers,indent=2)+'\n')
    original_diag=node.diag
    log=(node.out/'shadow_diagnostics.jsonl').open('w',buffering=1)
    class Diagnostics:
        def publish(self,message):
            row=json.loads(message.data);row['shadow_only']=True
            row['computed_v']=row.pop('applied_v');row['computed_w']=row.pop('applied_w')
            row['executed_v']=0.;row['executed_w']=0.
            log.write(json.dumps(row)+'\n');original_diag.publish(message)
    node.diag=Diagnostics()
    local_execution.run(node)


if __name__=='__main__':main()
