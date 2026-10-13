#!/usr/bin/env python3
"""Verify data/source integrity and replay recorded CBF infeasibility."""
import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import numpy as np
import yaml
from report import metrics

DP=ct.POINTER(ct.c_double)
def ptr(a):return a.ctypes.data_as(DP)


def audit(out):
    records=[];build=None;hashes={}
    for p in sorted(out.iterdir()):
        if not (p/'result.json').exists():continue
        manifest=json.loads((p/'manifest.json').read_text());build=Path(manifest['build'])
        m,a,e,tr,d,obs=metrics(p);r=json.loads((p/'result.json').read_text())
        assert r['goal_sent'] is not None and r['trajectory_bundles']>0,(p,'no native plan')
        assert np.all(np.diff(a['t'])>0) and m['max_arm_error_rad']<=1e-5,(p,'state/arm invalid')
        assert abs(float(a['clearance'].min())-m['min_clearance_m'])<1e-9
        limits=json.loads((p/'limits.json').read_text());wheel_l=a['v']-.5*limits['track']*a['w'];wheel_r=a['v']+.5*limits['track']*a['w']
        if manifest.get('experiment_profile')=='no_uncertainty':
            assert all(limits[k]==0. for k in ('pos_unc','vel_unc','acc_unc'))
            params=yaml.safe_load((p/'parameters.yaml').read_text())
            for key in ('position_uncertainty','velocity_uncertainty','acceleration_bound'):
                assert params['/remani_planner_node/dynamic_point_flow/'+key]==0.
            assert limits['margin']==.1,'fixed margin unexpectedly changed'
        assert max(abs(wheel_l).max(),abs(wheel_r).max())<=limits['wheel_v']+1e-8
        dt=np.diff(a['t']);acc=np.maximum(abs(np.diff(wheel_l)/dt),abs(np.diff(wheel_r)/dt))
        assert acc.max()<=limits['wheel_a']+1e-6
        for name,digest in manifest.get('experiment_sources_sha256',{}).items():
            f=p/'runtime'/name
            if f.exists():assert hashlib.sha256(f.read_bytes()).hexdigest()==digest,(p,name,'runtime drift')
        parameter_hash=hashlib.sha256((p/'parameters.yaml').read_bytes()).hexdigest();hashes.setdefault(m['planner_variant'],set()).add(parameter_hash)
        note={'trial':p.name,'max_wheel_speed':float(max(abs(wheel_l).max(),abs(wheel_r).max())),'max_wheel_accel':float(acc.max()),'plant_loop_max_dt':float(dt.max()),'arm_error':m['max_arm_error_rad'],'perception_healthy_fraction':m['perception_healthy_fraction']}
        if list(p.glob('*qp_failure.npz')):
            lib=ct.CDLL(str(build/'build/libonline_mppi.so'));lib.online_create.argtypes=[DP,DP,ct.c_int,ct.c_int];lib.online_create.restype=ct.c_void_p;lib.online_destroy.argtypes=[ct.c_void_p]
            lib.online_audit.argtypes=[ct.c_void_p,DP,DP,ct.c_int,ct.c_double,DP,ct.c_int,ct.c_int,ct.c_double,ct.c_double,ct.c_double,DP,DP];lib.online_audit.restype=ct.c_int
            c=np.array(list(limits.values()),dtype=float);body=np.ascontiguousarray(np.loadtxt(str(p/'geometry.csv'),delimiter=','));handle=lib.online_create(ptr(c),ptr(body),len(body),1);snapshots=[]
            for f in sorted(p.glob('*qp_failure.npz')):
                z=np.load(str(f));state=np.ascontiguousarray(z['state']);obstacles=np.ascontiguousarray(z['obs']);field=np.ascontiguousarray(z['field']);nominal=np.ascontiguousarray(z['result'][2:4]);result=np.zeros(8);gx,gy,res=z['grid']
                code=lib.online_audit(handle,ptr(state),ptr(obstacles),len(obstacles),float(z['age']),ptr(field),field.shape[1],field.shape[0],gx,gy,res,ptr(nominal),ptr(result));assert code==0
                snapshots.append({'file':f.name,'t':float(z['t']),'state':state.tolist(),'minimum_barrier_h':result[0],'minimum_margin_clearance_m':result[1],'nominal_min_residual':result[2],'zero_min_residual':result[3],'feasible_samples_of_1681':result[4],'best_sample_residual':result[5],'best_sample_v':result[6],'best_sample_w':result[7],
                    'interpretation':'Finite command-grid check is diagnostic, not proof of continuous infeasibility. Negative margin clearance means conservative safety-envelope overlap, not necessarily geometric collision.'})
            lib.online_destroy(handle);note['qp_replay']=snapshots
        records.append(note)
    assert all(len(v)==1 for v in hashes.values()),'parameters differ between methods'
    root=Path(__file__).resolve().parents[2];manifest=json.loads((build/'build_manifest.json').read_text());changed=[]
    for name,digest in manifest['production_sources_sha256'].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest()!=digest:changed.append(name)
    assert not changed,changed
    summary={'checks':'PASS','production_source_files_unchanged':len(manifest['production_sources_sha256']),'same_native_parameters_all_modes':True,'trials':records,'limitations':['Only single runs per combination; no statistical success-rate claim','Synthetic sensor has perfect pose, no occlusion/noise; plant is numerical, not Gazebo','CBF finite-difference sequential QP has no continuous-time global safety guarantee','Post-filter native speed limit can change the checked action; stop_unverified is not a proof about the final applied command','MPPI uses a conservative 2D measured-cloud distance field, not the native 3D ESDF collision representation']}
    (out/'audit.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n');print(json.dumps(summary,indent=2,ensure_ascii=False))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('out',type=Path);audit(p.parse_args().out)
