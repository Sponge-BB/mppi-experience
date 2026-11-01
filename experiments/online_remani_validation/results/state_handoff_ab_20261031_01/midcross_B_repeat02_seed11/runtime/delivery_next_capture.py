"""Event-only cusp telemetry and at most one complete bound collision input."""
import ctypes as ct
import hashlib
import json
import time
import numpy as np

class Capture:
    def __init__(self,node):
        self.node=node;self.events=[];self.previous=None;self.saved=None;self.closed=False
        self.ready=hasattr(node.lib,'online_cusp_phase_debug')
        if not self.ready:raise RuntimeError('required online_cusp_phase_debug ABI missing')
        node.lib.online_cusp_phase_debug.argtypes=[ct.c_void_p,ct.POINTER(ct.c_double)]
        node.lib.online_cusp_phase_debug.restype=ct.c_int

    def sample(self,cycle,now,state,reference,observations,age,field,result,controls,reason,guide):
        if self.closed:return
        values=np.zeros(14,dtype=np.float64)
        code=int(self.node.lib.online_cusp_phase_debug(self.node.handle,values.ctypes.data_as(ct.POINTER(ct.c_double))))
        if code!=14 or values[0]!=1:return
        # Field names/signature are validated against the child header by the
        # runner. The event key is reference identity plus measured phase.
        key=(getattr(guide,'epoch',None),getattr(guide,'revision',None),
            int(values[12]),int(values[13]),int(values[1]),int(values[9]))
        changed=self.previous is not None and key!=self.previous
        phase_changed=changed and key[:4]==self.previous[:4]
        reference_changed=changed and key[:2]!=self.previous[:2]
        if self.previous is None or changed:
            self.events.append(dict(event='cusp_phase',mono=time.monotonic(),t=now-1000,
                cycle_id=cycle,epoch=key[0],revision=key[1],debug=values.tolist(),
                debug_columns=['active','phase','phase_count','begin','end','gear','end_distance',
                    'end_yaw_error','remaining_arc','transitions','event','confirm','hash_high','hash_low'],
                measured_phase_transition=phase_changed,reason=reason,
                solve_state=state.tolist() if state is not None else None,
                guide_start=guide.start,guide_age=now-guide.start))
        self.previous=key
        stopped=getattr(self.node,'stall_streak',0)>=30
        trigger='first_replan_reference_adoption' if reference_changed and key[0]>=2 else \
            'measured_phase_transition' if phase_changed else 'sustained_non_goal_stop' if stopped else None
        payload=getattr(self.node,'delivery_next_solve_payload',None)
        if self.saved is not None or trigger is None or payload is None:return
        if payload[20]!=1 or state is None or reference is None:return
        self.saved=dict(payload=payload,state=state.copy(),reference=reference.copy(),
            observations=observations.copy(),field=field.copy(),result=result.copy(),controls=controls.copy(),
            reference_times=__import__("numpy").concatenate(guide.times),
            metadata=dict(event=trigger,cycle_id=cycle,t=now-1000,mono=time.monotonic(),
                solve_ros_time=getattr(self.node,'delivery_next_solve_clock',None),
                legacy_observed_obstacle_age=age,
                native_dynamic_age_of_bound_payload=max(0.,float(self.node.delivery_next_solve_clock-payload[18])),
                kernel_age_argument_recorded=False,
                epoch=key[0],revision=key[1],cusp_debug=values.tolist(),reason=reason,
                guide_start=guide.start,guide_age=now-guide.start,
                grid=[self.node.nx,self.node.ny,self.node.gx,self.node.gy,self.node.res],
                complete_native3d_esdf=True,exact_bound_solve_frame=True,
                rng_and_sampler_history_recorded=False,
                result_controls_semantics='final publication/validation choice; not necessarily raw kernel winner',
                scope='one exact bound native3d collision snapshot and kernel inputs; not bitwise MPPI decision replay'))

    def close(self):
        if self.closed:return
        self.closed=True
        with (self.node.out/'cusp_phase_events.jsonl').open('w') as stream:
            for event in self.events:stream.write(json.dumps(event,allow_nan=False)+'\n')
        summary=dict(recorded=self.saved is not None,maximum_snapshots=1,
            serialization='on shutdown only; no per-frame ESDF disk export',events=len(self.events))
        if self.saved is not None:
            sample=self.saved;metadata=sample.pop('metadata')
            path=self.node.out/'native3d_event_input.npz'
            np.savez(path,**sample)
            metadata['npz_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
            (self.node.out/'native3d_event_input.json').write_text(json.dumps(metadata,indent=2)+'\n')
            summary.update(trigger=metadata['event'],file=path.name,bytes=path.stat().st_size)
        (self.node.out/'delivery_event_capture.json').write_text(json.dumps(summary,indent=2)+'\n')

def record(node,*args):
    if not hasattr(node,'delivery_event_capture'):
        import rospy
        node.delivery_event_capture=Capture(node)
        rospy.on_shutdown(node.delivery_event_capture.close)
    node.delivery_event_capture.sample(*args)
