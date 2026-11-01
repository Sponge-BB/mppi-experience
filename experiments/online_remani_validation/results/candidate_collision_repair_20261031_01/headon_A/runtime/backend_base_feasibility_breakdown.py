#!/usr/bin/env python3
"""Cost-only decomposition of the native base-feasibility integral.

This is independent NumPy arithmetic, guarded against the attested C++ source.
It does not replay native 3-D collision, evaluate complete planner feasibility,
calculate gradients, optimize, or change the previously frozen native probe.
"""
import hashlib

import numpy as np

from backend_component_probe import DEFAULT_BUILD, extract, source_material
from audit_backend_wheel_gradient import function


CHANNELS = ('dense_min_speed', 'linear_speed', 'wheel_speed_left',
            'wheel_speed_right', 'wheel_accel_left', 'wheel_accel_right')


def source_contract(build=DEFAULT_BUILD):
    sources,attestation=source_material(build)
    text=sources['poly_traj_optimizer.cpp']
    wheel=function(text,'feasibilityGradCostCar')
    integral=extract(text,'void PolyTrajOptimizer::addPVAJGradCost2CT(',True)
    smooth=function(text,'smoothedL1');log=function(text,'smoothedLog')
    guards={
      'dense_min_speed':('pen = non_singul_v_ * non_singul_v_ - vel.squaredNorm();',
                        'cost_mm_feasible += wei_feas_ * 1e6 * f;'),
      'linear_speed':('pen = vel.squaredNorm() - max_vel_ * max_vel_;',),
      'wheel_speed_left':('double wheel_omega_left = (2.0 * singul_container_[trajid] * v_norm - mobile_base_wheel_base_ * omega) / (2.0 * mobile_base_wheel_radius_);',
                          'pen = wheel_omega_left * wheel_omega_left - max_wheel_omega_ * max_wheel_omega_;'),
      'wheel_speed_right':('double wheel_omega_right = (2.0 * singul_container_[trajid] * v_norm + mobile_base_wheel_base_ * omega) / (2 * mobile_base_wheel_radius_);',
                           'pen = wheel_omega_right * wheel_omega_right - max_wheel_omega_ * max_wheel_omega_;'),
      'wheel_accel_left':('double wheel_alpha_left = (2.0 * singul_container_[trajid] * aTv / v_norm - mobile_base_wheel_base_ * alpha) / (2.0 * mobile_base_wheel_radius_);',
                          'pen = wheel_alpha_left * wheel_alpha_left - max_wheel_alpha_ * max_wheel_alpha_;'),
      'wheel_accel_right':('double wheel_alpha_right = (2.0 * singul_container_[trajid] * aTv / v_norm + mobile_base_wheel_base_ * alpha) / (2.0 * mobile_base_wheel_radius_);',
                           'pen =  wheel_alpha_right * wheel_alpha_right - max_wheel_alpha_ * max_wheel_alpha_;'),
      'native_kinematics':('if (vTv < 1e-8) { vTv = 1e-8; }',
                           'if (v_norm < 1e-4) { v_norm = 1e-4; }',
                           'const double aTv = acc.transpose() * vel;',
                           'const double aTBv = acc.transpose() * B_h_ * vel;',
                           'const double jTBv = jer.transpose() * B_h_ * vel;',
                           'double omega = aTBv * vTv_inv;',
                           'double alpha = jTBv * vTv_inv - 2.0 * aTBv * aTv * vTv_inv2;')}
    for name,items in guards.items():
        if not all(item in wheel for item in items):raise ValueError('Unrecognized branch: '+name)
    if wheel.count('cost_mm_feasible += wei_feas_ * f;')!=5 or wheel.count('smoothedL1(pen, 0.005, f, df)')!=6:
        raise ValueError('Unknown native branch weights/smoothing')
    sampling_guards=(
      'step = SnapOpt_container_[trajid].get_T1()(i) / K;',
      'omg = (j == 0 || j == K) ? 0.5 : 1.0;',
      'costs(3) += omg * step * cost_mm_feasible;',
      'double min_dense_vel = 0.1;',
      'double pen_min_vel = min_dense_vel * min_dense_vel - vel.head(2).squaredNorm();',
      'if(smoothedLog(pen_min_vel, 0.001, pen_min_vel_log, grad_pen_min_vel_log))',
      'double special_step = step / dense_sample_resolution_;',
      'disQuantity = dense_sample_resolution_ / 2;',
      'special_s1 = special_s1 - step / 2.0;',
      'disQuantity = dense_sample_resolution_;',
      'for(int l = 0; l <= disQuantity; l++)',
      'special_s1 += special_step;',
      'dense_omg = (l == 0 || l == disQuantity) ? 0.5 : 1.0;',
      'double cost_dense_log = pen_min_vel_log * cost_mm_feasible;',
      'costs(3) += dense_omg * special_step * cost_dense_log;',
      's1 += step;')
    if not all(s in integral for s in sampling_guards):raise ValueError('Unknown actual native quadrature')
    if ('f = x - 0.5 * mu;' not in smooth or
        'f = mumxd2 * sqrxdmu * xdmu;' not in smooth or
        'l = 0.5 * e1_2 * e1 * (mu - x) * mu_4;' not in log or
        'l = 0.5 * (x + mu) * e2_2 * e2 * mu_4 + 1.0;' not in log):
        raise ValueError('Unknown native penalty smoothing')
    if 'B_h_ << 0.0, -1.0,\n            1.0,  0.0;' not in text:
        raise ValueError('Unknown B_h orientation')
    sha=lambda s:hashlib.sha256(s.encode()).hexdigest()
    return dict(attested_sources=attestation,
      functions_sha256={name:sha(body) for name,body in (
          ('feasibilityGradCostCar',wheel),('addPVAJGradCost2CT',integral),
          ('smoothedL1',smooth),('smoothedLog',log))},
      branch_formula_guards={name:list(items) for name,items in guards.items()},
      sampling_guards=list(sampling_guards),native_3d_replayed=False,
      scope='cost-only six subbranches of original costs(3); not a hard feasibility verdict')


def smoothed_l1(x, mu=.005):
    """Exact scalar-cost branches from native smoothedL1 (no gradients)."""
    x=np.asarray(x,dtype=float)
    result=np.zeros_like(x)
    middle=(x>=0)&(x<=mu);high=x>mu
    z=x[middle]/mu
    result[middle]=(mu-.5*x[middle])*z*z*z
    result[high]=x[high]-.5*mu
    return result


def smoothed_log(x, mu=.001):
    """Exact native dense activation weight, including both transition halves."""
    x=np.asarray(x,dtype=float);result=np.zeros_like(x)
    left=(x>-mu)&(x<=0);right=(x>0)&(x<=mu);high=x>mu
    e=x[left]+mu
    result[left]=.5*e*e*e*(mu-x[left])/(mu*mu*mu*mu)
    e=x[right]-mu
    result[right]=.5*(x[right]+mu)*e*e*e/(mu*mu*mu*mu)+1.
    result[high]=1.
    return result


def state_values(coefficients, times):
    """Vectorized ascending-power polynomial v/a/j in the native planar dims."""
    times=np.asarray(times,dtype=float)
    powers=np.ones((len(times),8))
    for power in range(1,8):powers[:,power]=powers[:,power-1]*times
    states=[]
    for derivative in (1,2,3):
        basis=np.zeros_like(powers)
        for power in range(derivative,8):
            factor=1
            for k in range(derivative):factor*=power-k
            basis[:,power]=factor*powers[:,power-derivative]
        states.append(basis@np.asarray(coefficients,dtype=float)[:,:2])
    return states


def point_costs(velocity, acceleration, jerk, params, gear, dense):
    """Six cost channels at primary/dense states, before quadrature weighting."""
    v=np.asarray(velocity,dtype=float);a=np.asarray(acceleration,dtype=float)
    j=np.asarray(jerk,dtype=float)
    if v.ndim!=2 or v.shape[1]!=2 or a.shape!=v.shape or j.shape!=v.shape:
        raise ValueError('State arrays must be finite n-by-2')
    if not all(np.all(np.isfinite(s)) for s in (v,a,j)) or gear not in (-1,1):
        raise ValueError('Invalid state/gear')
    weight,maxvel,non_singular,base,radius,max_omega,max_alpha=np.asarray(params,dtype=float)[:7]
    raw_v2=np.einsum('ij,ij->i',v,v);v2=np.maximum(1e-8,raw_v2)
    norm=np.maximum(1e-4,np.sqrt(raw_v2));av=np.einsum('ij,ij->i',a,v)
    abv=a[:,1]*v[:,0]-a[:,0]*v[:,1]
    jbv=j[:,1]*v[:,0]-j[:,0]*v[:,1]
    omega=abv/v2;alpha=jbv/v2-2.*abv*av/(v2*v2)
    omega_left=(2.*gear*norm-base*omega)/(2.*radius)
    omega_right=(2.*gear*norm+base*omega)/(2.*radius)
    accel_left=(2.*gear*av/norm-base*alpha)/(2.*radius)
    accel_right=(2.*gear*av/norm+base*alpha)/(2.*radius)
    channels=np.column_stack((
       1e6*smoothed_l1(non_singular*non_singular-raw_v2)*np.asarray(dense,dtype=float),
       smoothed_l1(raw_v2-maxvel*maxvel),
       smoothed_l1(omega_left*omega_left-max_omega*max_omega),
       smoothed_l1(omega_right*omega_right-max_omega*max_omega),
       smoothed_l1(accel_left*accel_left-max_alpha*max_alpha),
       smoothed_l1(accel_right*accel_right-max_alpha*max_alpha)))
    return weight*channels


def piece_nodes(coefficients,duration,intervals,dense_resolution):
    """Reproduce each actual primary and velocity-triggered dense sample.

    Keep C++ repeated time additions, including the fact that dense evaluation
    uses the basis made *before* special_s1 is incremented. Dense integration
    multiplies every channel by the trigger at its PRIMARY anchor velocity.
    """
    K=int(intervals);D=int(dense_resolution);step=float(duration)/K
    times=[];s=0.
    for unused in range(K+1):times.append(s);s+=step
    v,a,j=state_values(coefficients,times)
    primary_weight=np.full(K+1,step);primary_weight[[0,-1]]*=.5
    gain=smoothed_log(.1*.1-np.einsum('ij,ij->i',v,v))
    active=.1*.1-np.einsum('ij,ij->i',v,v)>-.001
    dense_times=[];dense_weights=[]
    for index in np.flatnonzero(active):
        sample_time=times[index]
        if index==0:count=D//2
        elif index==K:sample_time-=step/2.;count=D//2
        else:sample_time-=step/2.;count=D
        special_step=step/D
        for l in range(count+1):
            dense_times.append(sample_time)
            dense_weights.append((.5 if l in (0,count) else 1.)*special_step*gain[index])
            sample_time+=special_step
    dv,da,dj=state_values(coefficients,dense_times)
    return dict(primary=(v,a,j,primary_weight),dense=(dv,da,dj,np.asarray(dense_weights)),
      primary_count=K+1,dense_anchor_count=int(sum(active)),dense_node_count=len(dense_times))


def breakdown(value, context):
    proof=source_contract()
    params=np.asarray(context.params,dtype=float)
    if params.ndim!=1 or len(params)<10 or not np.all(np.isfinite(params)):
        raise ValueError('Invalid native parameters')
    K,D=int(params[8]),int(params[9])
    if K<1 or D<2 or params[8]!=K or params[9]!=D:raise ValueError('Bad quadrature resolution')
    if len(value['b_parts'])!=len(context.gears) or len(value['times_parts'])!=len(context.gears):
        raise ValueError('Different block counts')
    totals=np.zeros(6);pieces=[];minimum=float('inf')
    primary_count=dense_anchor_count=dense_node_count=0
    for block,(coeff,times,gear) in enumerate(zip(value['b_parts'],value['times_parts'],context.gears)):
        coeff=np.asarray(coeff,dtype=float);times=np.asarray(times,dtype=float)
        if coeff.shape!=(8*len(times),8) or not np.all(np.isfinite(coeff)) or np.any(times<=0):
            raise ValueError('Invalid reconstructed polynomial')
        for piece,duration in enumerate(times):
            nodes=piece_nodes(coeff[piece*8:(piece+1)*8],duration,K,D)
            subtotal=np.zeros(6)
            for kind in ('primary','dense'):
                v,a,j,weights=nodes[kind]
                if len(v):
                    minimum=min(minimum,float(np.min(np.linalg.norm(v,axis=1))))
                    costs=point_costs(v,a,j,params,int(gear),kind=='dense')
                    subtotal+=np.sum(weights[:,None]*costs,axis=0)
            totals+=subtotal
            primary_count+=nodes['primary_count'];dense_anchor_count+=nodes['dense_anchor_count']
            dense_node_count+=nodes['dense_node_count']
            pieces.append(dict(block=block,piece=piece,costs=dict(zip(CHANNELS,subtotal.tolist())),
                primary_count=nodes['primary_count'],dense_anchor_count=nodes['dense_anchor_count'],
                dense_node_count=nodes['dense_node_count']))
    summed=float(sum(totals));expected=value.get('costs',{}).get('wheel')
    relative=None if expected is None else abs(summed-float(expected))/max(1.,abs(summed),abs(float(expected)))
    return dict(costs=dict(zip(CHANNELS,totals.tolist())),sum=summed,
      native_probe_base_feasibility_integral=expected,normalized_sum_error=relative,
      sum_agrees_with_native_probe=None if relative is None else bool(relative<=1e-7),
      minimum_actual_evaluated_speed_m_s=minimum,primary_count=primary_count,
      dense_anchor_count=dense_anchor_count,dense_node_count=dense_node_count,
      pieces=pieces,source_proof=proof,
      warning='base feasibility includes dense minimum speed, not just wheel inequalities; no hard verdict')
