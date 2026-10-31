#!/usr/bin/env python3
"""Prepare/build diagnostic-only current-branch search/solver probes; no ROS."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from build_native_v9 import HERE,sha
from build_native_v10 import compile_source
from run_coupled_objective import check_build

BASE=HERE/'builds/boundary_admission_20261024_02'


def validate_base(base):
    m,_=check_build(base)
    if not m.get('boundary_admission_experiment') or m.get('boundary_admission_prepared_only'):
        raise ValueError('Requires attested compiled actual-fixed parent')
    for relative,digest in m['isolated_source_sha256'].items():
        if sha(base/'source'/relative)!=digest:raise ValueError('Changed parent source: '+relative)
    if (base/'unmeasured_superseded.json').exists():raise ValueError('Superseded build')
    return m


def prepare(base,out):
    from patch_delivery_search import transform_search
    from patch_delivery_replay import transform_optimizer
    base,out=base.resolve(),out.resolve();m=validate_base(base)
    out.mkdir(parents=True,exist_ok=False)
    shutil.copytree(base/'source',out/'source');shutil.copytree(base/'build',out/'build')
    for name in ('search_exit_probe.hpp','delivery_replay_probe.hpp'):
        shutil.copy2(HERE/name,out/'source'/name)
    changes={'kino_astar.cpp':transform_search,'poly_traj_optimizer.cpp':transform_optimizer}
    for name,transform in changes.items():
        original=(base/'source'/name).read_text();modified=transform(original)
        if original==modified:raise ValueError('No diagnostic transform: '+name)
        (out/'source'/name).write_text(modified)
    m.update(delivery_stability_experiment=True,delivery_stability_prepared_only=True,
        delivery_stability_parent=str(base),delivery_stability_parent_manifest_sha256=sha(base/'build_manifest.json'),
        delivery_stability_builder_sha256=sha(Path(__file__)),
        delivery_stability_scope='newly captured same-process paused SIL solver diagnosis; NOT exact historical replay or navigation performance',
        diagnostic_candidates_never_published=True,production_modified=False,
        diagnostic_declarations=dict(unchanged_wheel_and_geometry=True,admission_off=True,clearance_recovery_off=True,
            legacy_prefix_off=True,legacy_rolling_off=True,prediction_soft_weights=0.,normal_budget_s=.6,
            longer_budget_diagnostic_only=True,accepted_iterates_not_cost_callback_trials=True),
        isolated_source_sha256={str(p.relative_to(out/'source')):sha(p) for p in (out/'source').rglob('*') if p.is_file()})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    print('PREPARED',out,flush=True)


def compile_prepared(out):
    out=out.resolve();m=json.loads((out/'build_manifest.json').read_text())
    if not m.get('delivery_stability_prepared_only'):raise ValueError('Not a fresh prepared diagnostic build')
    base=Path(m['delivery_stability_parent']);validate_base(base)
    for name,digest in m['isolated_source_sha256'].items():
        if sha(out/'source'/name)!=digest:raise ValueError('Prepared source changed: '+name)
    parent=Path(m['parent_build'])
    # Do not trust loose parent .o files. Recompile all changed archives and
    # three planner-node TUs from this attested source/header snapshot.
    for target,name,archive in (('exp_search','kino_astar.cpp','libexp_search.a'),
                                ('exp_opt','poly_traj_optimizer.cpp','libexp_opt.a')):
        _,obj=compile_source(parent,out,target,out/'source'/name,'unused',out/'source')
        subprocess.run(['ar','r',str(out/'build'/archive),obj],check=True)
    link=shlex.split((parent/'build/CMakeFiles/remani_planner_node.dir/link.txt').read_text())
    for name in ('remani_planner_node.cpp','remani_replan_fsm.cpp','planner_manager.cpp'):
        old=next(x for x in link if x.endswith('/'+name+'.o'))
        _,obj=compile_source(parent,out,'remani_planner_node',out/'source'/name,old,out/'source')
        link=[obj if x==old else x for x in link]
    link=[str(out/'build'/x) if x in ('libexp_opt.a','libexp_search.a') else x for x in link]
    link[link.index('-o')+1]=str(out/'build/remani_planner_node')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    if sha(out/'build/libonline_mppi.so')!=m['mppi_library_sha256']:raise ValueError('MPPI library changed')
    m.update(delivery_stability_prepared_only=False,
        delivery_stability_archive_sha256={n:sha(out/'build'/n) for n in ('libexp_opt.a','libexp_search.a')},
        parent_state_contract_archive_sha256=m.get('state_contract_archive_sha256'),
        state_contract_archive_sha256={n:sha(out/'build'/n) for n in ('libexp_opt.a','libexp_search.a')},
        native_binaries_sha256={n:sha(out/'build'/n) for n in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');print('BUILT',out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--base',type=Path,default=BASE)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--compile',action='store_true')
    p.add_argument('--finish-only',action='store_true');a=p.parse_args()
    if a.finish_only and not a.compile:p.error('--finish-only requires --compile')
    if not a.finish_only:prepare(a.base,a.out)
    if a.compile:compile_prepared(a.out)
