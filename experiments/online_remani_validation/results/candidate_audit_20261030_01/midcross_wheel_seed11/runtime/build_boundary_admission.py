#!/usr/bin/env python3
"""NEW static cooperative start-admission trial; default prepare-only, no ROS.

Derives only from the frozen state_contract_20261023_01 build. OFF records the
same actual-fixed precheck after frontend but executes every original optimizer
path. ON defers ONLY eligible static margin-only actual fixed measured refresh
boundaries, using the existing failed-request/cooldown path. Never a success.
"""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from build_native_v9 import HERE, sha
from build_native_v10 import compile_source

DEFAULT_BASE=HERE/'builds/state_contract_20261023_01'
INCLUDE='#include "boundary_admission.hpp"\n'
INIT='    boundary_admission::initialize(nh);\n'
FSM_CONTEXT_ANCHOR='    coupling_geometry::Scope coupled_geometry;'
FSM_CONTEXT='''
    boundary_admission::RequestContext boundary_admission_request_context(
      local_execution_owner_,have_local_traj_,dynamic_repair,coupled_geometry);
'''
ANCHOR='    dual_probe::emit("stage_begin",dual_probe::Fields().str("stage","optimizer"));'
PRECHECK='''    const double boundary_admission_frontend_wall_ms=(dual_probe::mono()-request_probe.wall)*1000.;
    const int boundary_admission_failures_before=continous_failures_count_;
    const auto boundary_admission_decision=boundary_admission::actualPrecheck(
      iniStates_container,singul_container,start_pt,start_vel,start_acc,start_jerk,
      start_yaw,start_singul,trajectory_start_time,boundary_admission_frontend_wall_ms,t_init.toSec()*1000.,
      boundary_admission_failures_before);
    if(boundary_admission_decision.defer){
      state_contract_trace::queue("boundary_admission_manager_deferred",dual_probe::Fields()
        .str("admission_phase","after_frontend_before_optimizer")
        .str("phase_closed_by","existing_manager_scope_destructor")
        .str("reason",boundary_admission_decision.reason)
        .add("frontend_wall_ms",boundary_admission_frontend_wall_ms)
        .add("continous_failures_count_before",boundary_admission_failures_before)
        .add("continous_failures_count_after",continous_failures_count_)
        .str("failure_counter_policy","unchanged; admission defer is not a solver failure")
        .add("optimizer_started","false").add("manager_success","false"));
      return false; // NOT success; existing caller failure/restore/cooldown path.
    }
'''


def replace_once(text, before, after):
    if text.count(before)!=1:raise ValueError('Ambiguous/missing boundary-admission anchor: '+repr(before))
    return text.replace(before,after,1)


def transform_fsm(text):
    transformed=replace_once(text,'#include "state_contract_trace.hpp"',INCLUDE+'#include "state_contract_trace.hpp"')
    transformed=replace_once(transformed,'    state_contract_trace::initialize(nh);',INIT+'    state_contract_trace::initialize(nh);')
    transformed=replace_once(transformed,FSM_CONTEXT_ANCHOR,FSM_CONTEXT_ANCHOR+FSM_CONTEXT)
    # Exact OFF projection: remove ONLY the three new insertion blocks. The
    # original manager call, all gates, budgets, retry and publication code are
    # byte-for-byte unchanged; no legacy branch is duplicated or rewritten.
    projected=replace_once(transformed,INCLUDE,'')
    projected=replace_once(projected,INIT,'')
    projected=replace_once(projected,FSM_CONTEXT,'')
    if projected!=text:raise AssertionError('OFF projection changed original source')
    return transformed


def transform_manager(text):
    transformed=replace_once(text,'#include "state_contract_trace.hpp"',INCLUDE+'#include "state_contract_trace.hpp"')
    transformed=replace_once(transformed,ANCHOR,PRECHECK+ANCHOR)
    projected=replace_once(transformed,INCLUDE,'')
    projected=replace_once(projected,PRECHECK,'')
    if projected!=text:raise AssertionError('OFF projection changed original manager source')
    return transformed


def inputs(base):
    base=base.resolve();m=json.loads((base/'build_manifest.json').read_text())
    if not m.get('state_contract_experiment') or m.get('state_contract_prepared_only') or m.get('boundary_admission_experiment'):
        raise ValueError('Requires completed original frozen state-contract build')
    for relative,digest in m['isolated_source_sha256'].items():
        if sha(base/'source'/relative)!=digest:raise ValueError('Frozen source integrity failure: '+relative)
    for name,digest in m['native_binaries_sha256'].items():
        if sha(base/'build'/name)!=digest:raise ValueError('Frozen binary integrity failure: '+name)
    for name,digest in m['state_contract_archive_sha256'].items():
        if sha(base/'build'/name)!=digest:raise ValueError('Frozen archive integrity failure: '+name)
    if sha(base/'build/libonline_mppi.so')!=m['mppi_library_sha256']:raise ValueError('Frozen MPPI library changed')
    if 'if(!s.valid||s.gap<-1e-8)' not in (base/'source/coupling_bridge.hpp').read_text():
        raise ValueError('Frozen native static acceptance tolerance differs from same_native_gap_tolerance')
    return m


def self_test(base=DEFAULT_BASE):
    base=base.resolve();inputs(base)
    original=(base/'source/remani_replan_fsm.cpp').read_text();generated=transform_fsm(original)
    manager=transform_manager((base/'source/planner_manager.cpp').read_text())
    assert generated.index('if (!waitForRequiredESDF("REBOUND_REPLAN_GUARD"))') < generated.index(FSM_CONTEXT)
    assert generated.index(FSM_CONTEXT_ANCHOR) < generated.index(FSM_CONTEXT)
    assert 'boundary_admission_decision.defer' not in generated
    assert manager.index('iniStates_container.push_back(headState);') < manager.index(PRECHECK)
    assert manager.index('t_init = ros::Time::now() - t_start;') < manager.index(PRECHECK)
    assert manager.index(PRECHECK)+len(PRECHECK)==manager.index(ANCHOR)
    assert manager.index(PRECHECK) < manager.index('flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs(')
    assert 'return false;' in PRECHECK and 'return true' not in PRECHECK
    assert all(token not in PRECHECK for token in ('.publish(','.sleep(','.setZero(','.setMaxWallTime(','.setAstarMaxSearchTime('))
    assert 'if(!s.valid||s.gap<-1e-8)' in (base/'source/coupling_bridge.hpp').read_text()
    assert 'constexpr double same_native_gap_tolerance=1e-8;' in (HERE/'boundary_admission.hpp').read_text()
    caller=generated[generated.index('    case REPLAN_TRAJ:'):generated.index('    case EXEC_TRAJ:')]
    assert 'planFromLocalTraj(true,false,dynamic_astar_replan_wall_time_,false)' in caller
    assert 'next_cooperative_attempt_=ros::Time::now()+ros::Duration(.75)' in caller
    assert 'retain the guide and retry independently of local execution' in caller
    print('BOUNDARY_ADMISSION_TRANSFORMS_PASSED: exact FSM+manager OFF projection; observed actual fixed head after frontend before optimizer stage; original failure/retry path')
    return {'remani_replan_fsm.cpp':generated,'planner_manager.cpp':manager}


def prepare(base,out):
    base,out=base.resolve(),out.resolve();m=inputs(base)
    transformed={'remani_replan_fsm.cpp':transform_fsm((base/'source/remani_replan_fsm.cpp').read_text()),
                 'planner_manager.cpp':transform_manager((base/'source/planner_manager.cpp').read_text())}
    out.parent.mkdir(parents=True,exist_ok=True);out.mkdir(exist_ok=False)
    shutil.copytree(base/'source',out/'source');(out/'build').mkdir()
    for path in (base/'build').iterdir():
        if path.is_file() and not path.name.startswith('remani_planner_node'):
            shutil.copy2(path,out/'build'/path.name)
    shutil.copy2(HERE/'boundary_admission.hpp',out/'source/boundary_admission.hpp')
    for name,text in transformed.items():(out/'source'/name).write_text(text)
    m.update(boundary_admission_experiment=True,boundary_admission_prepared_only=True,
      boundary_admission_parent=str(base),boundary_admission_parent_manifest_sha256=sha(base/'build_manifest.json'),
      boundary_admission_builder_sha256=sha(Path(__file__)),boundary_admission_switch='BOUNDARY_ADMISSION',
      boundary_admission_required_env={'STATE_CONTRACT_CAPTURE':'1','COUPLING_ENGINEERING':'1','BOUNDARY_ADMISSION_STATIC_SCENE':'1'},
      boundary_admission_schema='boundary_admission_v2_actual_fixed',boundary_admission_event='boundary_admission_precheck',
      boundary_admission_policy='ON only after actual frontend/MINCO/warm/RRT/prefix head+gear arrays exist: same outer fresh valid declared-static empty-obstacle frame; cooperative refresh; actual base p/v/gear exact same-sequence processed measurement plus numerical measured-heading agreement; 0<actual fixed physical and gap<-1e-8 => manager false before optimizer stage; OFF same capture without skipping',
      boundary_admission_native_gap_tolerance=1e-8,
      boundary_admission_native_gap_reject_rule='strict gap < -1e-8, reused from frozen coupling_bridge.hpp native static validator; [-1e-8,0) classified native_tolerance and never deferred',
      boundary_admission_placement='manager after completed frontend iniStates_container/singul_container, before optimizer stage_begin/Optimize; FSM namespace RequestContext RAII only, no desired-preview defer',
      boundary_admission_clock='capture_mono/capture_sim; all new I/O deferred until final Request decision',
      boundary_admission_heading_tolerance='64*machine epsilon in sin(actual fixed body heading-measured yaw), cos(delta)>0; actual p/v/gear comparisons remain EXACT; numerical-only, no geometric threshold change',
      boundary_admission_limitations=['frontend remains executed and timed; only backend optimization is avoided, not frontend/search cost',
        'negative gap within original native numerical tolerance [-1e-8,0) is not evidence of fixed-boundary rejection and is never deferred',
        'actual p/v/gear not exactly equal measured state (including any native roundoff, RRT gear flip or warm-start difference) is capture-only, never deferred',
        'deferral does not increment continous_failures_count_; later A*/RRT fallback history may diverge from OFF and later frontend inputs are not claimed identical',
        'static declaration is external trial configuration, not inferred world truth; empty runtime obstacle list additionally required',
        'invalid/stale/dynamic/physical-collision/zero-gear paths are captured but unchanged and never admitted as recovery',
        'deferral is failure/not-success and publishes nothing; existing 0.75 ROS-second cooldown plus timer/ESDF/graph scheduling is not an exact wall interval',
        'projected frozen static field is not immutable native 3D map replay or replacement for full-margin candidate gates'],
      hard_gates_unchanged=True,all_objective_parameters_unchanged=True,
      search_budget_clock_unchanged=True,thread_policy_unchanged=True,production_modified=False,
      isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()},
      native_binaries_sha256={name:sha(out/'build'/name) for name in ('mm_controller_node','dynamic_obstacle_tracker_node')})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');print('PREPARED',out)


def compile_prepared(out):
    out=out.resolve();m=json.loads((out/'build_manifest.json').read_text())
    if not m.get('boundary_admission_experiment') or not m.get('boundary_admission_prepared_only'):
        raise ValueError('Requires NEW prepared boundary-admission directory')
    for relative,digest in m['isolated_source_sha256'].items():
        if sha(out/'source'/relative)!=digest:raise ValueError('Prepared source changed: '+relative)
    base=Path(m['boundary_admission_parent']);parent=Path(m['parent_build']);inputs(base)
    for name in ('libexp_opt.a','libexp_search.a'):
        if sha(out/'build'/name)!=sha(base/'build'/name):raise ValueError('Unchanged archive drift: '+name)
    link=shlex.split((parent/'build/CMakeFiles/remani_planner_node.dir/link.txt').read_text())
    for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp','planner_manager.cpp'):
        old=next(value for value in link if value.endswith('/'+name+'.o'))
        if name=='remani_planner_node.cpp':
            if sha(out/'source'/name)!=sha(base/'source'/name):raise ValueError('Reused object source changed: '+name)
            for header in ('plan_manage/remani_replan_fsm.h','plan_manage/planner_manager.h'):
                if sha(out/'source'/header)!=sha(base/'source'/header):raise ValueError('Reused object layout changed: '+header)
        # Recompile the three small planner-node translation units from attested
        # source, rather than trust unattested loose object files. Optimizer,
        # native search, MPPI and model/environment archives remain byte-identical.
        _,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        link=[obj if value==old else value for value in link]
    link=[str(out/'build/libexp_opt.a') if value=='libexp_opt.a' else
          str(out/'build/libexp_search.a') if value=='libexp_search.a' else value for value in link]
    link[link.index('-o')+1]=str(out/'build/remani_planner_node')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    if sha(out/'build/libonline_mppi.so')!=m['mppi_library_sha256']:raise ValueError('MPPI library changed')
    m.update(boundary_admission_prepared_only=False,
      native_binaries_sha256={name:sha(out/'build'/name) for name in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');print('BUILT',out)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,default=DEFAULT_BASE)
    p.add_argument('--out',type=Path);p.add_argument('--self-test',action='store_true')
    p.add_argument('--prepare-only',action='store_true');p.add_argument('--compile',action='store_true')
    p.add_argument('--finish-only',action='store_true');a=p.parse_args()
    if a.prepare_only and a.compile:p.error('--prepare-only and --compile are mutually exclusive')
    if a.finish_only and not a.compile:p.error('--finish-only requires --compile')
    if a.self_test:self_test(a.base)
    if a.out:
        if not a.finish_only:prepare(a.base,a.out)
        if a.compile:compile_prepared(a.out)
    elif not a.self_test:p.error('--out or --self-test required')
