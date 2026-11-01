#!/usr/bin/env python3
"""Prepare/build a NEW private state-contract experiment from a frozen domain build.

Default is preparation only: no compiler, ROS graph, model, or hardware invocation.
Use --compile only after reviewing --self-test and the generated source/manifest.
Capture is additive; signed low-speed mode is the only algorithm treatment.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from build_native_v9 import HERE, sha
from build_native_v10 import compile_source

DEFAULT_BASE = HERE / 'builds/domain_recovery_20261023_01'
HELPERS = ('state_contract_probe.hpp', 'state_contract_trace.hpp')


class Transformer:
    def __init__(self, text):
        self.original = text
        self.text = text
        self.changes = []

    def change(self, before, after):
        count = self.text.count(before)
        if count != 1:
            raise ValueError('Ambiguous/missing state-contract anchor (%d): %r' % (count, before[:140]))
        self.text = self.text.replace(before, after, 1)
        self.changes.append((before, after))

    def before(self, anchor, code):
        self.change(anchor, code + anchor)

    def after(self, anchor, code):
        self.change(anchor, anchor + code)

    def finish(self):
        # Exact inverse checks that no byte outside the explicitly audited
        # insertion/wrapper sites changed. The legacy floor remains verbatim.
        inverse = self.text
        for before, after in reversed(self.changes):
            if inverse.count(after) != 1:
                raise AssertionError('Non-invertible transformation')
            inverse = inverse.replace(after, before, 1)
        if inverse != self.original:
            raise AssertionError('Unexpected semantic/source edit outside hooks')
        return self.text


def transform_fsm(text):
    t = Transformer(text)
    t.before('#include "coupling_bridge.hpp"', '#include "state_contract_trace.hpp"\n')
    t.before('    collab::configure(nh);', '    state_contract_trace::initialize(nh);\n')
    callback = '  void REMANIReplanFSM::mmCarOdomCallback(const nav_msgs::OdometryConstPtr &msg)\n  {\n'
    t.after(callback, '''    const std::uint64_t state_contract_seq=state_contract_trace::enabled()?
      state_contract::receive(*msg,tf::getYaw(msg->pose.pose.orientation)):0;
    double state_contract_floor_bias=0.;
''')
    floor = '''    if(mm_state_vel_.head(2).norm() < mobile_base_non_singul_vel_){
      mm_state_vel_(0) = mobile_base_non_singul_vel_ * cos(mm_car_yaw_);
      mm_state_vel_(1) = mobile_base_non_singul_vel_ * sin(mm_car_yaw_);
      mm_car_singul_ = 0;
    }else{
      Eigen::Vector2d car_head(cos(mm_car_yaw_), sin(mm_car_yaw_));
      mm_car_singul_ = car_head.dot(mm_state_vel_.head(2)) >= 0.0 ? 1 : -1;
    }
'''
    t.change(floor, '''    if(state_contract_trace::signedEnabled()){
      const auto regularized=state_contract::signedRegularization(mm_state_vel_(0),mm_state_vel_(1),
        mm_car_yaw_,mobile_base_non_singul_vel_,true,0); // No history/hysteresis.
      mm_state_vel_(0)=regularized.vx;mm_state_vel_(1)=regularized.vy;
      mm_car_singul_=regularized.gear;state_contract_floor_bias=regularized.floor_bias;
      if(!regularized.valid || !std::isfinite(mm_state_pos_(0)) || !std::isfinite(mm_state_pos_(1))){
        state_contract::processed(state_contract_seq,mm_state_vel_(0),mm_state_vel_(1),
          mm_car_singul_,state_contract_floor_bias);
        have_odom_=false;
        throw std::runtime_error("invalid measured state in isolated signed state-contract trial");
      }
    }else{
'''+floor+'''    }
    if(state_contract_trace::enabled())state_contract::processed(state_contract_seq,
      mm_state_vel_(0),mm_state_vel_(1),mm_car_singul_,state_contract_floor_bias);
''')
    request_end = '      .add("esdf_seq",planner_manager_->grid_map_->getESDFObservationSeq()));\n'
    # There are other ESDF records: scope this one by the complete Request block.
    request_begin = text.index('    dual_probe::Request dual_request(')
    request_last = text.index(request_end, request_begin)+len(request_end)
    request = text[request_begin:request_last]
    t.after(request, '''    state_contract_trace::Request state_contract_request(&dual_request.success);
    state_contract_trace::pvaj("state_contract_selected_boundary",start_pos_,start_vel_,start_acc_,start_jer_,
      start_yaw_,start_singul_,state_contract::unavailable(),dual_probe::Fields()
        .add("local_execution_owner",local_execution_owner_?"true":"false")
        .add("have_local_traj",have_local_traj_?"true":"false")
        .str("selected_state_semantics",local_execution_owner_?
          "processed_measured_state_not_prediction":"caller_selected_state_may_be_trajectory_handoff"));
''')
    t.before('    bool plan_success = planner_manager_->reboundReplan(', '''    state_contract_trace::pvaj("state_contract_desired_boundary",desired_start_pt,desired_start_vel,
      desired_start_acc,desired_start_jerk,desired_start_yaw,desired_start_singul,desired_start_time);
''')
    t.after('      execution_reference_pub_.publish(transaction);++coupling_bridge::referenceRevision();', '''
      state_contract_trace::candidate("state_contract_reference_publish",*data,transaction.data);
''')
    abort = '    if(local_execution_owner_){std_msgs::String transaction;transaction.data=execution_goal_key_+"\\n1\\n"+executionHex(msg);execution_reference_pub_.publish(transaction);++coupling_bridge::referenceRevision();}'
    t.after(abort, '''
    if(state_contract_trace::enabled()){
      state_contract_trace::queue("state_contract_abort_publish",dual_probe::Fields()
        .add("stop_p",state_contract::vectorJson(stop_pos)).add("stop_yaw",stop_yaw).add("stop_gear",singul)
        .str("semantics","ACTION_ABORT contains no initial polynomial; not a reference candidate"));
      state_contract_trace::flush();
    }
''')
    return t.finish()


def transform_manager(text):
    t = Transformer(text)
    t.before('#include "target_entry_diagnostics.hpp"', '#include "state_contract_trace.hpp"\n')
    t.after('    resource_probe::Scope request_probe("replan");', '''
    state_contract_trace::pvaj("state_contract_manager_input",start_pt,start_vel,start_acc,start_jerk,
      start_yaw,start_singul,trajectory_start_time);
''')
    t.before('    dual_probe::Scope dual_initialization("initialization",dual_probe::Fields().add("poly_init",flag_polyInit?"true":"false"));', '''    state_contract_trace::pvaj("state_contract_frontend_init",start_pt,start_vel,start_acc,start_jerk,
      start_yaw,start_singul,state_contract::unavailable());
''')
    t.before('    flag_success = ploy_traj_opt_->OptimizeTrajectory_lbfgs(', '''    if(state_contract_trace::enabled())state_contract_trace::queue("state_contract_backend_call",dual_probe::Fields()
      .add("initial_states",state_contract_trace::matricesJson(iniStates_container))
      .add("final_states",state_contract_trace::matricesJson(finStates_container))
      .add("gears",state_contract_trace::signsJson(singul_container)));
''')
    call_end = '                                                            optCps_container, optWps_container, optT_container, optEECps_container);'
    t.after(call_end, '''
    if(state_contract_trace::enabled()){
      state_contract_trace::queue("state_contract_solver_return",dual_probe::Fields()
        .add("success",flag_success?"true":"false"));
      // Keep all new records buffered through epoch choice, latest gates and
      // publication. Request destruction flushes only AFTER the final decision.
    }
''')
    t.before('      if (delta_replan_time > 0) ros::Duration(delta_replan_time).sleep();', '''      if(state_contract_trace::enabled())state_contract_trace::queue("state_contract_epoch_wait_begin",dual_probe::Fields()
        .add("requested_start_epoch",trajectory_start_time).add("wait_requested_s",std::max(0.,delta_replan_time)));
''')
    t.after('      if (delta_replan_time > 0) ros::Duration(delta_replan_time).sleep();', '''
      if(state_contract_trace::enabled())state_contract_trace::queue("state_contract_epoch_wait_end",dual_probe::Fields()
        .add("requested_start_epoch",trajectory_start_time).add("wait_requested_s",std::max(0.,delta_replan_time)));
''')
    t.before('    traj_container_.singul_traj_data.clearSingulTraj();\n    for(unsigned int i = 0; i < singul_container.size(); ++i){', '''    if(state_contract_trace::enabled())state_contract_trace::queue("state_contract_epoch_chosen",dual_probe::Fields()
      .add("requested_start_epoch",trajectory_start_time).add("chosen_start_epoch",traj_start_time)
      .add("have_local_traj",have_local_traj?"true":"false"));
''')
    t.before('    dual_manager.success=true;request_probe.success=true;return true;', '''    state_contract_trace::candidate("state_contract_candidate_ready",traj_container_.singul_traj_data);
''')
    return t.finish()


def transform_optimizer(text):
    t = Transformer(text)
    t.before('#include "domain_lbfgs.hpp"', '#include "state_contract_trace.hpp"\n')
    t.after('    coupling_geometry::Scope coupled_geometry;', '''
    state_contract_trace::BackendRecord state_contract_backend(coupled_geometry,
      iniState_container,finState_container,singul_container,input_hash);
''')
    t.after('    coupling_bridge::Optimizer coupled_optimizer(input_hash,initT_container.size(),requested_pieces,singul_container);', '''
    state_contract_backend.link(coupled_optimizer.id);
''')
    t.before('    Eigen::VectorXd start_pos = iniState.col(0);', '''    if(state_contract_trace::enabled())state_contract_trace::queue("state_contract_native_astar_input",dual_probe::Fields()
      .add("head_state",state_contract_trace::matrixJson(iniState)).add("tail_state",state_contract_trace::matrixJson(finState))
      .add("requested_start_gear",_start_singul).add("requested_start_yaw",start_yaw));
''')
    t.before('    if(status == KinoAstar::NO_PATH || status == KinoAstar::START_COLLISION || status == KinoAstar::GOAL_COLLISION){', '''    if(state_contract_trace::enabled())state_contract_trace::queue("state_contract_native_astar_result",dual_probe::Fields()
      .add("status",status).add("gears",state_contract_trace::signsJson(singul_container)));
''')
    t.before('      frontendMJ_container[i].reset(headState, tailState, piece_num);', '''      state_contract_trace::mincoHead(headState,tailState,int(i),singul_container[i],_start_singul);
''')
    return t.finish()


def transform_search(text):
    t = Transformer(text)
    t.before('#include "path_searching/kino_astar.h"', '#include "state_contract_trace.hpp"\n')
    t.after('  cur_node->singul = getSingularity(start_state_(3));', '''
  state_contract_trace::nativeRoot(start_state,cur_node->singul,non_siguav_);
''')
    return t.finish()


def archive_member(path, name):
    return subprocess.check_output(['ar', 'p', str(path), name])


def inputs(base):
    base = base.resolve()
    manifest = json.loads((base/'build_manifest.json').read_text())
    if not manifest.get('domain_recovery_experiment') or manifest.get('state_contract_experiment'):
        raise ValueError('Requires original frozen domain_recovery_20261023_01 build')
    for relative, digest in manifest['isolated_source_sha256'].items():
        if sha(base/'source'/relative) != digest:
            raise ValueError('Frozen source integrity failure: '+relative)
    for name,digest in manifest['native_binaries_sha256'].items():
        if sha(base/'build'/name) != digest:
            raise ValueError('Frozen binary integrity failure: '+name)
    if sha(base/'build/libonline_mppi.so') != manifest['mppi_library_sha256']:
        raise ValueError('Frozen MPPI library integrity failure')
    parent = Path(manifest['parent_build'])
    search_archive = Path(manifest['coupling_parent'])/'build/libexp_search.a'
    member = 'kino_astar.cpp.o'
    if archive_member(search_archive,member) != archive_member(parent/'build/libexp_search.a',member):
        raise ValueError('Frozen search native member differs from reviewed parent source lineage')
    native = parent/'source/mm_remani_planner/path_searching/src/kino_astar.cpp'
    expected_native=manifest['production_sources_sha256']['src/mm_remani_planner/path_searching/src/kino_astar.cpp']
    if sha(native) != expected_native:
        raise ValueError('Native parent source differs from frozen manifest attestation')
    return manifest,parent,search_archive,native


def transformations(base):
    manifest,parent,search,native = inputs(base)
    transformed = {}
    for name, function in (('remani_replan_fsm.cpp',transform_fsm),
                           ('planner_manager.cpp',transform_manager),
                           ('poly_traj_optimizer.cpp',transform_optimizer)):
        transformed[name] = function((base/'source'/name).read_text())
    transformed['kino_astar.cpp'] = transform_search(native.read_text())
    return manifest,parent,search,native,transformed


def self_test(base=DEFAULT_BASE):
    manifest,parent,search,native,transformed = transformations(base)
    fsm = transformed['remani_replan_fsm.cpp']
    assert 'true,0); // No history/hysteresis.' in fsm
    assert 'state_contract::receive(*msg,tf::getYaw(msg->pose.pose.orientation))' in fsm
    assert fsm.index('state_contract::receive(') < fsm.index('    // std::cout << "odom:')
    assert 'cur_node->singul = getSingularity(start_state_(3));\n  state_contract_trace::nativeRoot' in transformed['kino_astar.cpp']
    native_old = native.read_text()
    assert transformed['kino_astar.cpp'].count('ros::Time::now()') == native_old.count('ros::Time::now()')
    assert 'collab_budget' not in transformed['kino_astar.cpp']
    optimizer = transformed['poly_traj_optimizer.cpp']
    assert optimizer.index('coupling_geometry::Scope coupled_geometry;') < optimizer.index('state_contract_trace::BackendRecord')
    assert optimizer.index('state_contract_trace::BackendRecord') < optimizer.index('if(coupling_geometry::enabled()&&!coupled_geometry.valid())')
    assert optimizer.count('coupling_native::addGeometry') == (base/'source/poly_traj_optimizer.cpp').read_text().count('coupling_native::addGeometry')
    assert 'state_contract_trace::flush()' not in transformed['planner_manager.cpp']
    assert 'state_contract_trace::Request state_contract_request(&dual_request.success)' in fsm
    print('STATE_CONTRACT_TRANSFORMS_PASSED: exact reversible hooks; legacy floor verbatim; native strict selector observed; clocks/gates unchanged; no manager flush before publication')
    return transformed


def prepare(base, out):
    base,out = base.resolve(),out.resolve()
    manifest,parent,search,native,transformed = transformations(base)
    out.parent.mkdir(parents=True,exist_ok=True)
    out.mkdir(exist_ok=False)
    shutil.copytree(base/'source',out/'source')
    (out/'build').mkdir()
    for name in HELPERS:
        shutil.copy2(HERE/name,out/'source'/name)
    for name,text in transformed.items():
        (out/'source'/name).write_text(text)
    for name in ('mm_controller_node','dynamic_obstacle_tracker_node','libonline_mppi.so','libexp_opt.a'):
        shutil.copy2(base/'build'/name,out/'build'/name)
    shutil.copy2(search,out/'build/libexp_search.a')
    manifest.update(state_contract_experiment=True,state_contract_prepared_only=True,
      state_contract_parent=str(base),state_contract_parent_manifest_sha256=sha(base/'build_manifest.json'),
      state_contract_search_archive=str(search),state_contract_native_source=str(native),
      state_contract_native_source_sha256=sha(native),
      state_contract_native_source_attestation='domain manifest production_sources_sha256 src/mm_remani_planner/path_searching/src/kino_astar.cpp',
      state_contract_native_member_sha256=hashlib.sha256(archive_member(search,'kino_astar.cpp.o')).hexdigest(),
      state_contract_builder_sha256=sha(Path(__file__)),
      state_contract_switches=['STATE_CONTRACT_CAPTURE','STATE_CONTRACT_SIGNED_LOW_SPEED'],
      state_contract_signed_guards='capture=1, COUPLING_ENGINEERING=1, isolated SIL + sim time; no automatic env mutation',
      state_contract_policy='legacy OFF verbatim; signed |longitudinal|>1e-4 only; previous_direction=0; <=16 ULP strict floor',
      state_contract_trace_schema='state_contract_trace_v1',
      state_contract_event_clocks=['capture_mono','capture_sim'],
      state_contract_io='new records buffered through epoch/latest gates/publication; Request destructor flushes only after final request decision; immutable frame serialization post-decision only',
      state_contract_limitations=['projected static field is not immutable native 3D map replay',
        'capture overhead is shared diagnostic treatment, not timing-neutral',
        'raw callback can be old while ROS spin is blocked; join independent incoming odom by exact serialized message identity',
        'native strict selector absent when collision/fallback path skips it; never infer that missing event means gear0'],
      clearance_al_code_present=False,clearance_al_enabled=False,
      hard_gates_unchanged=True,all_objective_parameters_unchanged=True,
      search_budget_clock_unchanged=True,thread_policy_unchanged=True,production_modified=False,
      isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()})
    # Prepared source is deliberately not labeled a runnable derived planner.
    manifest['native_binaries_sha256']={name:sha(out/'build'/name) for name in ('mm_controller_node','dynamic_obstacle_tracker_node')}
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('PREPARED',out)
    return out


def compile_prepared(out):
    out=out.resolve()
    manifest=json.loads((out/'build_manifest.json').read_text())
    if not manifest.get('state_contract_experiment') or not manifest.get('state_contract_prepared_only'):
        raise ValueError('Requires a new prepared state-contract directory')
    for relative,digest in manifest['isolated_source_sha256'].items():
        if sha(out/'source'/relative)!=digest:raise ValueError('Prepared source changed: '+relative)
    parent=Path(manifest['parent_build'])
    for name,target,archive in (('poly_traj_optimizer.cpp','exp_opt','libexp_opt.a'),
                                ('kino_astar.cpp','exp_search','libexp_search.a')):
        _,obj=compile_source(parent,out,target,out/'source'/name,'',out/'source')
        subprocess.run(['ar','r',str(out/'build'/archive),obj],check=True)
    link=shlex.split((parent/'build/CMakeFiles/remani_planner_node.dir/link.txt').read_text())
    for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp','planner_manager.cpp'):
        old=next(x for x in link if x.endswith('/'+name+'.o'))
        _,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        link=[obj if value==old else value for value in link]
    link=[str(out/'build/libexp_opt.a') if value=='libexp_opt.a' else
          str(out/'build/libexp_search.a') if value=='libexp_search.a' else value for value in link]
    link[link.index('-o')+1]=str(out/'build/remani_planner_node')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    manifest.update(state_contract_prepared_only=False,
      native_binaries_sha256={name:sha(out/'build'/name) for name in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')},
      state_contract_archive_sha256={name:sha(out/'build'/name) for name in ('libexp_opt.a','libexp_search.a')})
    if sha(out/'build/libonline_mppi.so') != manifest['mppi_library_sha256']:
        raise ValueError('Frozen MPPI library changed')
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('BUILT',out)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,default=DEFAULT_BASE)
    parser.add_argument('--out',type=Path)
    parser.add_argument('--self-test',action='store_true')
    parser.add_argument('--prepare-only',action='store_true',help='Explicit alias for the default no-compile operation')
    parser.add_argument('--compile',action='store_true',help='Explicit opt-in compilation, after preparation')
    parser.add_argument('--finish-only',action='store_true',help='Compile an existing prepared --out; requires --compile')
    args=parser.parse_args()
    if args.prepare_only and args.compile:parser.error('--prepare-only and --compile are mutually exclusive')
    if args.self_test:self_test(args.base)
    if args.finish_only and not args.compile:parser.error('--finish-only requires --compile')
    if args.out:
        if not args.finish_only:prepare(args.base,args.out)
        if args.compile:compile_prepared(args.out)
    elif not args.self_test:parser.error('--out or --self-test is required')
