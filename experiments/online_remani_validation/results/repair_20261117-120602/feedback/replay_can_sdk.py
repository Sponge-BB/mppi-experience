#!/usr/bin/python3
"""Feed saved CAN into installed SDK; no CAN constructor, connect or send."""
import ast,collections,hashlib,json,math,struct,sys,time
from pathlib import Path
from types import SimpleNamespace as NS
import can
from piper_sdk import C_PiperInterface
import piper_sdk.utils.fps as fps_module
ROOT=Path('/home/l/remani_ws')
CASE=ROOT/'experiments/online_remani_validation/results/dynamic_acceptance_20261117-103634'
OUT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'src/piper_ros/src/piper/scripts'))
from piper_feedback_dispatch import FeedbackDispatch
class VirtualClock:
    wall=0.;steady=0.
    def time(self):return self.wall
    def monotonic(self):return self.steady
    def perf_counter(self):return self.steady
clock=VirtualClock();fps_module.time=clock
sdk=C_PiperInterface(can_name='offline_recording',can_auto_init=False)
assert sdk._C_PiperInterface__arm_can is None
assert not sdk._C_PiperInterface__connected
diagnostics = "--diagnostics" in sys.argv
parse=FeedbackDispatch(sdk, diagnostics)
source=ROOT/'src/piper_ros/src/piper/scripts/piper_ctrl_single_node.py'
tree=ast.parse(source.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='C_PiperRosNode')
cls.body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('_read_joint_feedback','_refresh_joint_feedback','_source_deadline')]
ns={'math':math,'time':clock,'SDK_JOINT_GROUPS':('ArmJoint_12','ArmJoint_34','ArmJoint_56'),'FEEDBACK_VALID_AGE':.15}
exec(compile(ast.Module(body=[cls],type_ignores=[]),str(source),'exec'),ns)
node=ns['C_PiperRosNode'].__new__(ns['C_PiperRosNode']);node.piper=sdk
node._last_joint_counts=(0,0,0);node._feedback_stamp=0.;node._feedback_position=None;node._feedback_velocity=None
node._feedback_monotonic=0.;node._feedback_arrivals=()
fmt=struct.Struct('=QQQIB3x8s');received=collections.Counter();advanced=collections.Counter();samples=[];rejected=collections.Counter();last_sample=0.;last_frame={};can_max={};start=time.monotonic()
frames=0;accepted=0
with (CASE/'can_frames_fixed.bin').open('rb') as f:
    while True:
        raw=f.read(fmt.size*4096)
        if not raw:break
        assert len(raw)%fmt.size==0
        for kernel,wall,steady,ident,dlc,data in fmt.iter_unpack(raw):
            ident&=0x1fffffff
            if ident not in (0x2a1,0x2a5,0x2a6,0x2a7,*range(0x261,0x267)):continue
            clock.wall=wall/1e9;clock.steady=steady/1e9
            before=sdk._C_PiperInterface__fps_counter.fps_data.copy()
            parse(can.Message(timestamp=kernel/1e9,arbitration_id=ident,is_extended_id=False,data=data[:dlc]))
            received[hex(ident)]+=1;frames+=1
            if ident in (0x2a5,0x2a6,0x2a7):
                group=ns['SDK_JOINT_GROUPS'][ident-0x2a5]
                if sdk._C_PiperInterface__fps_counter.fps_data[group]>before[group]:advanced[hex(ident)]+=1
                if ident in last_frame:can_max[hex(ident)]=max(can_max.get(hex(ident),0),(kernel-last_frame[ident])/1e6)
                last_frame[ident]=kernel
            if clock.steady-last_sample>=.005:
                last_sample=clock.steady
                old=node._feedback_stamp;node._refresh_joint_feedback()
                if node._feedback_stamp>old:
                    accepted+=1;samples.append((clock.wall,node._feedback_stamp))
                else:
                    stamp,counts,arrivals,q=node._read_joint_feedback()
                    if any(n<=old for n,old in zip(counts,node._last_joint_counts)):rejected['no_new_complete_triplet']+=1
                    elif clock.perf_counter()-min(arrivals)>.15:rejected['old_arrivals']+=1
                    elif max(arrivals)-min(arrivals)>.05:rejected['pair_spread']+=1
                    elif not 0<=clock.time()-stamp<.15:rejected['source_age']+=1
                    elif stamp<=node._feedback_stamp:rejected['nonadvancing_source']+=1
                    else:rejected['other']+=1

gaps=[(b[0]-a[0])*1000 for a,b in zip(samples,samples[1:])]
long=[{'before':a[0],'after':b[0],'gap_ms':(b[0]-a[0])*1000} for a,b in zip(samples,samples[1:]) if b[0]-a[0]>.15]
report={'safety':'no CAN bus created; no ConnectPort or command call; installed SDK only decoded saved frames','frames':frames,'received':received,'sdk_joint_advances':advanced,'can_max_gap_ms':can_max,'accepted_samples':accepted,'acceptance_gaps_over_150ms':long,'max_accepted_gap_ms':max(gaps),'rejected_samples':rejected,'elapsed_s':time.monotonic()-start,'driver_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'sdk_path':__import__('inspect').getfile(C_PiperInterface),'limitation':'Virtual clock reproduces recorded reader timestamps; no actual thread scheduling or publication load is reproduced.'}
(OUT/('can_sdk_diagnostics_replay.json' if diagnostics else 'can_sdk_replay.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
