#!/usr/bin/env python3
"""Post-run symmetric wheel audit of canonical initial/accepted MINCO states.

No ROS, solver, retiming, mutation or native 3D collision replay. Full projected
geometry is linked from the independently hashed iterate audit, never invented
from wheel feasibility. Native k0 means initial optimization-coordinate MINCO,
not the A* polyline or an executed plant state.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np

from audit_delivery_iterates import (canonical_candidate, canonical_records,
    conjunction, finite, vector_hash)
from report_domain_recovery import assert_plot_quiet, digest, read_json

NATIVE_HASHES={
 'IsNotFeasibie':'6a7380559b2d1bab893b3df03753dc32289715395ae8139a6fca253d72ef34c4',
 'IsTrajSafe':'fe3e99522c45d3d5b4051d314091f73c2682d19ab9194dea896e9738e06f7662',
}
CHANNELS=('left_speed','right_speed','left_acceleration','right_acceleration')


def native_formula_evidence(text):
    import hashlib
    begin=text.index('  bool PolyTrajOptimizer::IsNotFeasibie(')
    middle=text.index('  bool PolyTrajOptimizer::IsTrajSafe(',begin)
    end=text.index('  bool PolyTrajOptimizer::checkCollision(',middle)
    hashes={name:hashlib.sha256(block.rstrip().encode()).hexdigest() for name,block in (
        ('IsNotFeasibie',text[begin:middle]),('IsTrajSafe',text[middle:end]))}
    return dict(known=hashes==NATIVE_HASHES and 'B_h_ << 0.0, -1.0,' in text and
        '1.0,  0.0;' in text,functions_sha256=hashes,expected_functions_sha256=NATIVE_HASHES)


def loaded_values(text,keys):
    values={}
    for key in keys:
        matches=[line.split(':',1)[1].strip() for line in text.splitlines() if line.startswith(key+':')]
        if len(matches)!=1:raise ValueError('Missing/duplicate loaded wheel parameter: '+key)
        try:value=float(matches[0])
        except ValueError:raise ValueError('Nonscalar loaded wheel parameter: '+key)
        if not math.isfinite(value) or value<=0.:raise ValueError('Invalid loaded wheel parameter: '+key)
        values[key]=value
    return values


def parameters(trial):
    manifest=read_json(trial/'manifest.json');build=Path(manifest['build'])
    attestation=read_json(build/'build_manifest.json');source=build/'source/poly_traj_optimizer.cpp'
    expected=attestation['isolated_source_sha256'].get('poly_traj_optimizer.cpp')
    formula=native_formula_evidence(source.read_text())
    path=trial/'effective_real_params.json';raw=read_json(path)
    prefix='/remani_planner_node/mm/mobile_base_'
    keys=dict(wheel_base_m='wheel_base',wheel_radius_m='wheel_radius',
              wheel_speed_limit_rad_s='max_wheel_omega',wheel_acceleration_limit_rad_s2='max_wheel_alpha')
    loaded_path=trial/'parameters.yaml'
    loaded=loaded_values(loaded_path.read_text(),[prefix+key for key in keys.values()])
    values={name:loaded[prefix+key] for name,key in keys.items()}
    exported={name:raw.get(prefix+key) for name,key in keys.items()}
    if exported!=values:raise ValueError('Loaded wheel params differ from frozen exported baseline')
    for name,key in keys.items():
        override=manifest.get('parameter_overrides',{}).get(prefix+key,values[name])
        if override!=values[name]:raise ValueError('Manifest wheel override differs from loaded parameter')
    known=formula['known'] and digest(source)==expected and all(finite(v) and v>0. for v in values.values())
    evidence=dict(known=known,source=str(source),source_sha256=digest(source),
        source_attestation_verified=digest(source)==expected,formula=formula,
        build_manifest_sha256=digest(build/'build_manifest.json'),
        parameter_source=str(path),parameter_sha256=digest(path),exact_parameter_keys={k:prefix+v for k,v in keys.items()},
        loaded_parameter_source=str(loaded_path),loaded_parameter_sha256=digest(loaded_path),
        loaded_exported_wheel_values_equal=True,loaded_manifest_wheel_overrides_consistent=True,
        values=values,native_tolerance_fraction=.05,
        limits_are_planner_params_not_controller_params=True,
        acceleration_limit_phase_snapshot_recorded=False,
        acceleration_limit_scope='loaded attested planner setParam; no per-phase acceleration-limit snapshot',
        native_alpha_limit_setparam_occurrences=source.read_text().count('nh.param("mm/mobile_base_max_wheel_alpha", max_wheel_alpha_'))
    if not known:raise ValueError('Unknown native formula or wheel parameter provenance: '+str(trial))
    return values,evidence


def derivative(piece,time,order):
    coefficients=np.asarray(piece['coeffs'],dtype=float)
    if coefficients.ndim!=2 or coefficients.shape[0]<2 or coefficients.shape[1]!=8 or not np.all(np.isfinite(coefficients)):
        raise ValueError('Invalid polynomial coefficients')
    value=np.zeros(2);power_time=1.
    for power in range(order,8):
        multiplier=math.factorial(power)//math.factorial(power-order)
        value+=(multiplier*power_time)*coefficients[:2,7-power]
        power_time*=time
    if not np.all(np.isfinite(value)):raise ValueError('Nonfinite polynomial derivative')
    return value


def locate(candidate,time):
    if not finite(time) or time<0.:raise ValueError('Invalid sample time')
    parts=candidate['parts'];part=0;local=time
    # Both native SingulTrajData and Trajectory choose the preceding part/piece
    # when t equals its duration (strict >, not >=).
    while part+1<len(parts) and local>parts[part]['duration']:
        local-=parts[part]['duration'];part+=1
    pieces=parts[part]['pieces'];piece=0
    while piece+1<len(pieces) and local>pieces[piece]['T']:
        local-=pieces[piece]['T'];piece+=1
    if local>pieces[piece]['T']+1e-10:raise ValueError('Time outside candidate')
    return part,piece,local


def wheel_values(candidate,time,params):
    part,piece,local=locate(candidate,time);poly=candidate['parts'][part]['pieces'][piece]
    vel,acc,jer=(derivative(poly,local,order) for order in (1,2,3))
    gear=candidate['parts'][part]['gear']
    if gear not in (-1,1):raise ValueError('Unknown gear')
    vx,vy=(float(v) for v in vel);ax,ay=(float(v) for v in acc);jx,jy=(float(v) for v in jer)
    aTv=ax*vx+ay*vy;aTBv=-ax*vy+ay*vx;jTBv=-jx*vy+jy*vx
    squared=vx*vx+vy*vy;norm=math.sqrt(squared)
    if norm<1e-4:norm=1e-4
    if squared<1e-8:squared=1e-8
    inv=1./squared;omega=aTBv*inv
    alpha=jTBv*inv-2.*aTBv*aTv*inv*inv
    base,radius=params['wheel_base_m'],params['wheel_radius_m']
    result=dict(left_speed=(2.*gear*norm-base*omega)/(2.*radius),
        right_speed=(2.*gear*norm+base*omega)/(2.*radius),
        left_acceleration=(2.*gear*aTv/norm-base*alpha)/(2.*radius),
        right_acceleration=(2.*gear*aTv/norm+base*alpha)/(2.*radius))
    if not all(math.isfinite(v) for v in result.values()):raise ValueError('Nonfinite wheel value')
    return dict(t=time,part=part,piece=piece,local_t=local,gear=gear,
        velocity=[vx,vy],acceleration=[ax,ay],jerk=[jx,jy],omega=omega,alpha=alpha,**result)


def native_times(duration):
    if not finite(duration) or duration<=0.:raise ValueError('Invalid trajectory duration')
    end=math.floor(duration/.01)
    if end>2000000:raise ValueError('Unreasonably large diagnostic trajectory')
    time=.05
    for unused in range(5,end):
        yield time;time+=.01


def diagnostic_times(candidate):
    duration=sum(float(part['duration']) for part in candidate['parts'])
    values=[0.,.01,.02,.03,.04,duration,*native_times(duration)]
    elapsed=0.
    for part in candidate['parts']:
        for piece in part['pieces']:
            elapsed+=float(piece['T']);values.append(elapsed)
            if elapsed<duration:values.append(float(np.nextafter(elapsed,math.inf)))
    return sorted(set(t for t in values if 0.<=t<=duration))


def scan_wheels(candidate,times,params,cache=None):
    cache={} if cache is None else cache
    peaks={name:None for name in CHANNELS};counts=Counter();first={name:None for name in CHANNELS}
    total=0
    for time in times:
        total+=1
        if time not in cache:cache[time]=wheel_values(candidate,time,params)
        value=cache[time]
        for name in CHANNELS:
            number=value[name];base_limit=params['wheel_speed_limit_rad_s' if name.endswith('speed') else 'wheel_acceleration_limit_rad_s2']
            # Native comparison is squared value minus squared limit with two
            # 1.05 factors, not a rounded abs(value)>rounded_limit shortcut.
            pen=number*number-base_limit*base_limit*1.05*1.05
            item=dict(value=number,abs_value=abs(number),ratio_to_final_limit=abs(number)/(base_limit*1.05),
                squared_native_penalty=pen,t=time,part=value['part'],piece=value['piece'],local_t=value['local_t'])
            if peaks[name] is None or abs(number)>peaks[name]['abs_value']:peaks[name]=item
            if pen>0.:
                counts[name]+=1
                if first[name] is None:first[name]=item
    return dict(samples=total,peaks=peaks,violating_samples={name:counts[name] for name in CHANNELS},
        first_violation=first,all_wheels_sampled_pass=(not any(counts.values())) if total else None,
        continuous_extrema_proven=False,does_not_early_exit_at_native_first_failure=True)


def analyze_candidate(record,capture,prior,params):
    if (record.get('stage'),record.get('iteration'),vector_hash(record.get('x')))!=(
        prior.get('stage'),prior.get('iteration'),prior.get('candidate_x_hash')):
        raise ValueError('Iterate audit candidate identity mismatch')
    candidate=canonical_candidate(record,capture['frame']['now'])
    if candidate is None:return dict(stage=record.get('stage'),iteration=record.get('iteration'),
        candidate_x_hash=vector_hash(record.get('x')),evidence_valid=False,reason='canonical reconstruction missing')
    duration=sum(float(part['duration']) for part in candidate['parts']);cache={}
    native=scan_wheels(candidate,native_times(duration),params,cache)
    diagnostic=scan_wheels(candidate,diagnostic_times(candidate),params,cache)
    k0=wheel_values(candidate,0.,params)
    native_pass=record.get('native_IsTrajSafe_pass')
    # Native pass proves wheel checks only on its own grid; failure may have
    # been an earlier collision, so never require wheels to fail when it fails.
    native_agreement=(native['all_wheels_sampled_pass'] is not False) if native_pass is True else None
    evidence=prior.get('candidate_evidence_valid') is True and native_agreement is not False
    return dict(stage=record.get('stage'),iteration=record.get('iteration'),solver_elapsed_s=record.get('solver_elapsed_s'),
        candidate_x_hash=vector_hash(record.get('x')),candidate_g_hash=vector_hash(record.get('g')),
        evidence_valid=evidence,reason=None if evidence else 'prior evidence invalid or native-pass wheel disagreement',
        initial_k0_values=k0,k0_scope='t=0 of this canonical MINCO candidate; initial stage is optimizer k0, not A* polyline',
        duration_s=duration,native_10ms=native,diagnostic_10ms_with_boundaries=diagnostic,
        native_IsTrajSafe_pass=native_pass,native_reason=record.get('native_reason'),
        native_pass_implies_sampled_wheel_pass=native_agreement,
        frozen_projected_pass=prior.get('frozen_projected_pass'),
        frozen_projected_20ms=prior.get('frozen_projected_20ms'),both_pass=prior.get('both_pass'),
        not_an_online_delivery=True)


def analyze_trial(trial,prior):
    records,path=canonical_records(trial)
    if path is None:
        return dict(trial=trial.name,evidence_valid=False,captured=False,reason='no canonical capture; not a success',phases=[])
    if prior.get('trial')!=trial.name or prior.get('replay_sha256')!=digest(path) or prior.get('manifest_sha256')!=digest(trial/'manifest.json'):
        raise ValueError('Iterate audit trial provenance mismatch')
    params,proof=parameters(trial);captures=[r for r in records if r.get('kind')=='capture']
    if len(captures)!=1:raise ValueError('Expected one canonical capture')
    capture=captures[0];phases=[]
    for old_phase in prior.get('phases',[]):
        name=old_phase['phase'];rows=[r for r in records if r.get('phase')==name and r.get('kind')=='canonical_candidate']
        old=old_phase.get('candidates',[])
        if len(old)!=len(rows):raise ValueError('Canonical candidate count disagreement')
        summary=old_phase['solve_summary']
        if summary.get('wheel_limit')!=params['wheel_speed_limit_rad_s']:
            raise ValueError('Recorded phase wheel limit differs from effective planner params')
        candidates=[analyze_candidate(record,capture,previous,params) for record,previous in zip(rows,old)]
        counts={stage:dict(count=sum(c['stage']==stage for c in candidates),
            native_wheel_pass=sum(c['stage']==stage and c.get('native_10ms',{}).get('all_wheels_sampled_pass') is True for c in candidates),
            diagnostic_wheel_pass=sum(c['stage']==stage and c.get('diagnostic_10ms_with_boundaries',{}).get('all_wheels_sampled_pass') is True for c in candidates),
            full_projected_pass=sum(c['stage']==stage and c.get('frozen_projected_pass') is True for c in candidates),
            native_and_projected_pass=sum(c['stage']==stage and c.get('both_pass') is True for c in candidates)) for stage in ('initial','accepted')}
        phases.append(dict(phase=name,input_hash=summary.get('input_hash'),
            first_signature=old_phase.get('first_signature'),phase_evidence_valid=conjunction([
                old_phase.get('phase_evidence_valid'),all(c['evidence_valid'] for c in candidates)]),
            counts=counts,candidates=candidates))
    return dict(trial=trial.name,captured=True,replay_sha256=digest(path),replay_path=str(path),
        manifest_sha256=digest(trial/'manifest.json'),run_status_sha256=digest(trial/'run_status.json'),
        parameter_evidence=proof,evidence_valid=conjunction([prior.get('evidence_valid'),all(p['phase_evidence_valid'] is True for p in phases)]),phases=phases)


def report(root,iterate_audit,out):
    root,iterate_audit,out=Path(root).resolve(),Path(iterate_audit).resolve(),Path(out).resolve()
    if out.exists() or out==root or out in root.parents:raise ValueError('Fresh separate output required')
    prior=read_json(iterate_audit)
    if prior.get('schema')!='delivery_iterate_audit_v1' or Path(prior.get('source','')).resolve()!=root:
        raise ValueError('Wrong iterate audit source/schema')
    complete=read_json(root/'complete.json')
    if complete.get('complete') is not True:raise ValueError('Complete batch required')
    jobs=[j for j in complete.get('completed',[]) if j.get('kind')=='paused']
    if len(jobs)!=4:raise ValueError('Expected four predeclared paused captures')
    names=[j['trial'] for j in jobs]
    if len(set(names))!=4 or set(names)!={t['trial'] for t in prior.get('trials',[])}:
        raise ValueError('Iterate audit trial set mismatch')
    assert_plot_quiet(dict(trials=[dict(run_status=read_json(root/name/'run_status.json')) for name in names]))
    by_name={t['trial']:t for t in prior['trials']}
    result=dict(schema='backend_kinematics_audit_v1',source=str(root),iterate_audit=str(iterate_audit),
        iterate_audit_sha256=digest(iterate_audit),auditor_sha256=digest(Path(__file__)),
        trials=[analyze_trial(root/name,by_name[name]) for name in names],
        limitations=['Posterior initial/accepted MINCO candidate audit; optimizer k0 is not original A* polyline.',
            'Native exact 10ms grid starts .05s, excludes final endpoint; diagnostic adds k0, [0,.05), endpoints, exact piece/gear boundaries and nextafter-right samples (native strict > selector).',
            'All four wheel channels scanned symmetrically without stopping at first native rejection; no continuous-time extrema proof.',
            'Native collision and manipulator constraints are inherited only from recorded IsTrajSafe, not reconstructed by wheel equations.',
            'Full frozen projected 20ms robot disk-union audit is reused by exact candidate and source hash; not independent physical truth.',
            'All calculations are post-solver; feasible sample is not a delivered trajectory and does not certify future live FSM gate.',
            'Negative-zero JSON and integer identities use existing canonical parser unchanged; no restriction is relaxed.'])
    if digest(iterate_audit)!=result['iterate_audit_sha256']:raise RuntimeError('Iterate audit changed')
    out.mkdir(parents=True,exist_ok=False)
    (out/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    lines=['固定种子后端：左右轮速度/加速度与完整投影后验审计',
        'k0为优化变量初始MINCO曲线，不是A*原始折线。轮约束左右对称，未在首个失败处截断。','']
    for trial in result['trials']:
        lines.append('{} evidence_valid={}'.format(trial['trial'],trial['evidence_valid']))
        for phase in trial['phases']:lines.append('  {} {}'.format(phase['phase'],phase['counts']))
    lines+=['']+result['limitations']
    (out/'轮速与投影复核.txt').write_text('\n'.join(lines)+'\n')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True);parser.add_argument('--iterate-audit',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True);args=parser.parse_args()
    result=report(args.root,args.iterate_audit,args.out)
    print(json.dumps(dict(out=str(args.out),valid=sum(t['evidence_valid'] is True for t in result['trials']))))
