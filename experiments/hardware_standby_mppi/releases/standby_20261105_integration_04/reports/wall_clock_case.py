import json,time
from pathlib import Path
import numpy as np
from run_offline_validation import make_fixture, Q_START, StandbyWholeBodyMPPI
from mppi_command_gateway import InputCohort, PlanEnvelope, MppiCommandGateway, NativeLeaseValidator, RecordingSink
r=Path.cwd()
m,c,_,snapshot,state,reference=make_fixture()
cfg=json.loads((r/'config/hardware_shadow.json').read_text())
k=StandbyWholeBodyMPPI(m,cfg['limits'],[c.joint_lower,c.joint_upper,c.joint_velocity,c.joint_acceleration],cfg['postures'],horizon=cfg['horizon'],step=cfg['step'],random_samples=cfg['random_samples'])
sink=RecordingSink();gateway=MppiCommandGateway(c,NativeLeaseValidator(k,config=c),sink)
stamp=time.time();snapshot.header[3]=stamp;snapshot.header[18]=stamp
assert gateway.update_feedback(state,stamp)
assert gateway.set_enabled(True)
source=time.time();t=time.monotonic();plan,info=k.plan(state,reference,Q_START,snapshot,source);solve_ms=(time.monotonic()-t)*1000
assert plan is not None
assert gateway.update_feedback(state,time.time())
cohort=InputCohort('synthetic-wall-clock',tuple(state),stamp,stamp,stamp+.15,'odom')
envelope=PlanEnvelope(1,source,source+.15,cohort.cohort_id,tuple(plan[0][0]))
t=time.monotonic();accepted=gateway.submit(envelope,cohort,snapshot);dispatch_ms=(time.monotonic()-t)*1000
out=dict(synthetic_only=True,clock='actual host wall and monotonic',device_access=False,accepted=accepted,solve_ms=solve_ms,validation_and_recording_ms=dispatch_ms,status=gateway.status(),horizon_s=k.H*k.dt,random_samples=k.random_samples,hardware_motion_validated=False)
(r/'reports/wall_clock_pipeline.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out))
assert accepted, gateway.status()
