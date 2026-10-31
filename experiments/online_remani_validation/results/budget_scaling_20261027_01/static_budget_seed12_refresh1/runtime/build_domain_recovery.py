#!/usr/bin/env python3
"""Derive a private bounded-domain line-search experiment; never production.

Both paired modes fail closed on a terminal negative solver status, avoiding
publication from callback caches left at a rejected trial. This common safety
change is explicit: old diagnostic trials are not the paired OFF baseline.
"""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from audit_dual_timescale import build_integrity
from build_native_v9 import HERE, sha, replace
from build_native_v10 import compile_source
from domain_step_control import transform_lbfgs


def transform_optimizer(text):
    text = '#include "domain_lbfgs.hpp"\n' + text
    text = text.replace('lbfgs::', 'domain_lbfgs::')
    marker = '    int result = domain_lbfgs::lbfgs_optimize('
    text = replace(text, marker, '''    domain_step::reset();
    int result;
    try {
    result = domain_lbfgs::lbfgs_optimize(''')
    marker = '        lbfgs_params);'
    text = replace(text, marker, marker+'''
    } catch(const domain_step::RejectedTrial& error) {
      diagnosis.reason="initial_static_map_domain";
      dual_probe::emit("domain_initial_reject",dual_probe::Fields()
        .str("reason",error.what()).add("body_index",error.body_index));
      return false; // Initial inadmissibility cannot masquerade as convergence.
    }
    dual_probe::emit("domain_solver_status",dual_probe::Fields()
      .add("enabled",int(coupling_native::domainRecoveryEnabled()))
      .add("rejected_trials",domain_step::rejectedTrials())
      .add("accepted_steps_after_rejection",domain_step::acceptedStepsAfterRejection())
      .add("lbfgs_result",result));
''')
    marker = '    }catch(const std::exception& error){'
    text = replace(text, marker, '''    }catch(const domain_step::RejectedTrial&){
      if(!x.allFinite() || !std::isfinite(total_smoo_cost) || !total_obs_feas_costs.allFinite()){
        opt->force_stop_type_=STOP_FOR_ERROR;g.setZero();return 1.0e100;
      }
      throw; // No fictitious objective/gradient; only private trial search catches it.
'''+marker)
    marker = '    // test collision'
    text = replace(text, marker, '''    if(result<0){
      diagnosis.reason="terminal_solver_failure";
      dual_probe::emit("domain_terminal_reject",dual_probe::Fields().add("lbfgs_result",result));
      return false; // x/g/f rollback does not restore callback-owned polynomial caches.
    }

'''+marker)
    return text


def build(base, out):
    base, out = base.resolve(), out.resolve()
    if not build_integrity(base, {})['pass_integrity']:
        raise ValueError('Base integrity failed')
    m = json.loads((base/'build_manifest.json').read_text())
    if not m.get('delivery_diagnostics') or m.get('domain_recovery_experiment'):
        raise ValueError('Requires original frozen diagnostic build')
    if shutil.disk_usage(out.parent).free < 2*1024**3:
        raise RuntimeError('Less than 2GiB free; no new build')
    out.mkdir(parents=True, exist_ok=False)
    shutil.copytree(base/'source', out/'source')
    shutil.copytree(base/'build', out/'build')
    for name in ('coupling_collision.hpp','coupling_geometry_ros.hpp',
                 'coupling_native_integration.hpp','domain_step_rejection.hpp'):
        shutil.copy2(HERE/name, out/'source'/name)
    parent = Path(m['parent_build'])
    native_header = parent/'source/mm_remani_planner/traj_opt/include/optimizer/lbfgs.hpp'
    (out/'source/domain_lbfgs.hpp').write_text(transform_lbfgs(native_header.read_text()))
    source = out/'source/poly_traj_optimizer.cpp'
    source.write_text(transform_optimizer(source.read_text()))
    _, opt = compile_source(parent, out, 'exp_opt', source, '', out/'source')
    subprocess.run(['ar','r',str(out/'build/libexp_opt.a'),opt],check=True)
    link = shlex.split((parent/'build/CMakeFiles/remani_planner_node.dir/link.txt').read_text())
    for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp','planner_manager.cpp'):
        old = next(x for x in link if x.endswith('/'+name+'.o'))
        if name=='remani_planner_node.cpp':
            obj = out/'build'/(name+'.o')
            if not obj.exists():
                _,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        else:
            _,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        link=[str(obj) if x==old else x for x in link]
    search=Path(m['coupling_parent'])/'build/libexp_search.a'
    link=[str(out/'build/libexp_opt.a') if x=='libexp_opt.a' else
          str(search) if x=='libexp_search.a' else x for x in link]
    link[link.index('-o')+1]=str(out/'build/remani_planner_node')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    m.update(domain_recovery_experiment=True,domain_parent=str(base),
        domain_parent_manifest_sha256=sha(base/'build_manifest.json'),
        domain_original_lbfgs_sha256=sha(native_header),
        domain_switch='COUPLING_DOMAIN_RECOVERY',
        domain_scope='typed finite static-domain line-search rejection; no objective extension',
        terminal_solver_failure_fail_closed=True,
        common_pair_safety_change='Both OFF/ON reject every terminal negative solver result before consuming mutable caches',
        all_objective_parameters_unchanged=True,geometry_margin_unchanged=True,
        mppi_policy_unchanged=True,production_modified=False,
        isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()},
        native_binaries_sha256={n:sha(out/'build'/n) for n in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')},
        domain_builder_sha256=sha(Path(__file__)))
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    if not build_integrity(out,{})['pass_integrity']:
        raise RuntimeError('Derived build integrity failed')
    if sha(out/'build/libonline_mppi.so')!=json.loads((base/'build_manifest.json').read_text())['mppi_library_sha256']:
        raise RuntimeError('MPPI library unexpectedly changed')
    print('BUILT',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,default=HERE/'builds/delivery_diagnostics_20261022_01')
    p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();build(a.base,a.out)
