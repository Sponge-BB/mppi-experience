"""Interleaved offline NumPy/native comparison; no ROS or hardware imports."""
import os
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):
    os.environ[key] = '1'
import ast
import ctypes
import hashlib
import json
from pathlib import Path
import resource
import statistics
import sys
import time
from unittest.mock import patch
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
sys.path.insert(0,str(ROOT/'src/mm_remani/scripts/mppi'))
import active_arm_mppi
from active_arm_mppi import WholeBodyMPPI,Snapshot
from active_arm_model import PiperArmModel
from standby_whole_body_mppi import StandbyWholeBodyMPPI
config = json.loads((HERE.parent/'after_restart_runtime_config.json').read_text())['params']
params = config['/whole_body_mppi'][2]
model = PiperArmModel.from_parameters(config['/mm_controller_node'][2])
model.lower[5],model.upper[5] = -.35,.35
feedback = []
for line in (HERE.parent/'joint_lead_fullrate.jsonl').open():
    record = json.loads(line)
    if record.get('topic') == '/joint_states_single':feedback.append(record)
pulse = max(feedback,key=lambda r:max(map(abs,r['dq'])))
state = np.r_[np.zeros(5),pulse['q'],pulse['dq']]
tree = ast.parse((ROOT/'src/mm_remani/scripts/whole_body_mppi.py').read_text())
function = next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name == 'checked_first_step')
namespace = {'np':np}
exec(compile(ast.Module(body=[function],type_ignores=[]),'checked_first_step','exec'),namespace)
checked_first_step = namespace['checked_first_step']
original = ctypes.CDLL
library = HERE/'libmppi_position_servo.so'
with patch.object(active_arm_mppi.ctypes,'CDLL',side_effect=lambda ignored:original(str(library))):
    native = active_arm_mppi._load_legacy_velocity_library()
assert native is not None

def kernel(cls=WholeBodyMPPI):
    k = cls(model,params['base_limits'],(model.lower,model.upper,
        params['arm_velocity'],params['arm_acceleration']),params['postures'],
        horizon=params['horizon'],step=1/params['rate_hz'],random_samples=params['random_samples'],seed=11)
    k._legacy_native = native
    return k

def summary(values):
    return dict(median_ms=statistics.median(values),min_ms=min(values),max_ms=max(values),samples_ms=values)

began = time.monotonic()
micro = []
k = kernel()
for batch in (1,3,22,240):
    z = state.copy() if batch == 1 else np.tile(state,(batch,1))
    u = np.zeros(z.shape[:-1]+(8,));u[...,0]=.05;u[...,1]=-.15;u[...,2:]=.15
    timings = {backend:[] for backend in ('numpy','native')}
    for repeat in range(3):
        for backend in (('numpy','native') if repeat%2 == 0 else ('native','numpy')):
            k._legacy_native = native if backend == 'native' else None
            start = time.perf_counter()
            for _ in range(200):k.advance(z,u,.05)
            timings[backend].append((time.perf_counter()-start)*1000/200)
    row = dict(batch=batch,**{key:summary(value) for key,value in timings.items()})
    row['speedup'] = row['numpy']['median_ms']/row['native']['median_ms'];micro.append(row)

h = np.zeros(32);h[11]=.05;h[8:11]=[-2.,-2.,0.]
snapshot = Snapshot(h,np.full((82,82,60),3.),[])
navigation_reference = np.c_[np.linspace(0.,1.,21),np.zeros((21,2)),np.full(21,.05)]
cases=[]
for cls,label in ((WholeBodyMPPI,'navigation'),(StandbyWholeBodyMPPI,'standby')):
    reference = navigation_reference if cls is WholeBodyMPPI else np.zeros((2,4))
    for dq_label in ('zero','recorded_peak'):
        z = state.copy()
        if dq_label == 'zero':z[11:17]=0.
        kernels = {name:kernel(cls) for name in ('numpy','native')}
        kernels['numpy']._legacy_native = None
        for k in kernels.values():k.plan(z,reference,model.q_nominal,snapshot,100.)
        times = {name:[] for name in kernels};delivery = {name:[] for name in kernels};maximum=0.
        for repeat in range(5):
            results={}
            for name in (('numpy','native') if repeat%2 == 0 else ('native','numpy')):
                k = kernels[name]; start=time.perf_counter()
                result,info = k.plan(z,reference,model.q_nominal,snapshot,100.)
                times[name].append((time.perf_counter()-start)*1000)
                assert result is not None
                start=time.perf_counter(); end=checked_first_step(k,z,result[0][0],snapshot,100.)
                delivery[name].append((time.perf_counter()-start)*1000)
                results[name]=(result,end,info)
            for a,b in zip(results['numpy'][0],results['native'][0]):
                maximum=max(maximum,float(np.max(np.abs(a-b))))
            maximum=max(maximum,float(np.max(np.abs(results['numpy'][1]-results['native'][1]))))
            assert results['numpy'][2]['feasible']==results['native'][2]['feasible']
        cases.append(dict(kernel=label,feedback=dq_label,plan={key:summary(v) for key,v in times.items()},
            first_command_check={key:summary(v) for key,v in delivery.items()},maximum_output_difference=maximum,
            plan_speedup=statistics.median(times['numpy'])/statistics.median(times['native'])))
report=dict(microbench=micro,plans=cases,elapsed_s=time.monotonic()-began,
    peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
    library=str(library),library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),installed=False,
    scenario='Recorded q/peak dq, paired same-q zero dq; same runtime limits and Piper model; synthetic clear ESDF and straight/standby references; no dynamic KDTree, ROS, node age/authority/deadline gates.',
    numerical_model='Unchanged velocity_legacy; arm_position_servo remains false; measured dq is not clipped.',
    benchmark_scope='Few interleaved repeats; local microbenchmark, not live percentile prediction.')
(HERE/'benchmark.json').write_text(json.dumps(report,indent=2)+'\n')
for row in micro:print('advance batch',row['batch'],'speedup',round(row['speedup'],2))
for row in cases:
    print(row['kernel'],row['feedback'],'plan',round(row['plan']['numpy']['median_ms'],2),'->',
        round(row['plan']['native']['median_ms'],2),'check',round(row['first_command_check']['numpy']['median_ms'],2),'->',
        round(row['first_command_check']['native']['median_ms'],2),'maxdiff',row['maximum_output_difference'])
print('elapsed_s',round(report['elapsed_s'],3),'peak_rss_kib',report['peak_rss_kib'])
