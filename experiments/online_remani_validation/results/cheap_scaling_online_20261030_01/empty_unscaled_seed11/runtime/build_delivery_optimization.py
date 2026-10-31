#!/usr/bin/env python3
"""Private native67 child: exact dynamic query + accepted timeout candidate.

No production/default changes. Only optimizer and GridMap are rebuilt; the
unchanged environment members, search, model, controller and MPPI are reused.
"""
import argparse
import copy
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from build_state_contract import Transformer
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_feasibility_stop import source_inventory
from build_urdf_spheres_b import recipes, verify_recipe, ARCHIVES, PLANNER, FROZEN
from build_native67_probe_lite import check as check_parent
from run_coupled_objective import check_build
from patch_delivery_candidate import transform_optimizer
from patch_delivery_fast_query import transform_grid_map

HERE = Path(__file__).resolve().parent
BASE = HERE/'builds/native67_probe_20261030_02'
DEFAULT = HERE/'builds/delivery_optimization_20261030_01'
MAP = FROZEN/'source/mm_remani_planner/plan_env/src/grid_map.cpp'
HELPERS = ('delivery_accepted_candidate.hpp', 'delivery_fast_query.hpp')
IMPLEMENTATION = ('build_delivery_optimization.py', 'patch_delivery_candidate.py',
                  'patch_delivery_fast_query.py', *HELPERS)


def optimizer_source(text):
    t = Transformer(transform_optimizer(text))
    t.change('''      const double budgets[]={.60,3.00};
      const char* labels[]={"native67_original_060","native67_extended_300"};''', '''      const double budgets[]={.60,.60};
      const char* labels[]={"delivery_baseline_060","delivery_optimized_060"};
      // Only this paused, never-published diagnostic changes flags in process.
      const char* query_env=std::getenv("DELIVERY_FAST_QUERY");
      const char* candidate_env=std::getenv("DELIVERY_ACCEPT_CANDIDATE");
      const bool had_query=query_env!=nullptr,had_candidate=candidate_env!=nullptr;
      const std::string saved_query=query_env?query_env:"";
      const std::string saved_candidate=candidate_env?candidate_env:"";
      delivery_replay::Finally restore_delivery_switches([&](){
        if(had_query)setenv("DELIVERY_FAST_QUERY",saved_query.c_str(),1);
        else unsetenv("DELIVERY_FAST_QUERY");
        if(had_candidate)setenv("DELIVERY_ACCEPT_CANDIDATE",saved_candidate.c_str(),1);
        else unsetenv("DELIVERY_ACCEPT_CANDIDATE");
      });''')
    t.after('''        for(size_t i=0;i<2;++i){''', '''
          setenv("DELIVERY_FAST_QUERY",i?"1":"0",1);
          setenv("DELIVERY_ACCEPT_CANDIDATE",i?"1":"0",1);''')
    return t.finish()


def recipe(out):
    r = recipes(out)
    r['compile_commands'] = [x for x in r['compile_commands'] if x['target']=='exp_opt']
    r['archive_commands'] = [x for x in r['archive_commands'] if x[2].endswith('/libexp_opt.a')]
    r['link_commands'] = {'remani_planner_node':r['link_commands']['remani_planner_node']}
    flags = FROZEN/'build/CMakeFiles/exp_env.dir/flags.make'
    lines = flags.read_text().splitlines()
    options = []
    for key in ('CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS'):
        value = [line.split(' = ',1)[1] for line in lines if line.startswith(key+' = ')]
        if len(value)!=1:raise ValueError('Ambiguous GridMap compiler flags: '+key)
        options += shlex.split(value[0])
    r['inherited_flags_sha256'][str(flags)]=sha(flags)
    r['unchanged_source_inputs_sha256'][str(MAP)]=sha(MAP)
    obj=out/'build/grid_map.cpp.o'
    r['compile_commands'].insert(0,dict(target='exp_env',source=str(out/'source/grid_map.cpp'),
        object=str(obj),command=['/usr/bin/c++',*options,'-I'+str(out/'source'),
            '-c',str(out/'source/grid_map.cpp'),'-o',str(obj)]))
    r['archive_commands'].insert(0,['ar','rcs',str(out/'build/libexp_env.a'),str(obj)])
    old=str(FROZEN/'build/libexp_env.a')
    link=r['link_commands']['remani_planner_node']
    if link.count(old)!=1:raise ValueError('Expected one environment archive')
    link[link.index(old)]=str(out/'build/libexp_env.a')
    return r


def prepare(out=DEFAULT):
    out=Path(out).resolve()
    if out.exists():raise FileExistsError(out)
    parent=check_parent(BASE)
    out.mkdir(parents=True);shutil.copytree(BASE/'source',out/'source');(out/'build').mkdir()
    (out/'source/poly_traj_optimizer.cpp').write_text(optimizer_source((BASE/'source/poly_traj_optimizer.cpp').read_text()))
    (out/'source/grid_map.cpp').write_text(transform_grid_map(MAP.read_text()))
    for name in HELPERS:shutil.copy2(HERE/name,out/'source'/name)
    reuse=['mm_controller_node','dynamic_obstacle_tracker_node','libonline_mppi.so','libexp_model.a','libexp_search.a']
    reuse += [name+'.o' for name in PLANNER]
    for name in reuse:shutil.copy2(BASE/'build'/name,out/'build'/name)
    shutil.copy2(FROZEN/'build/libexp_env.a',out/'build/libexp_env.a')
    members=subprocess.check_output(['ar','t',str(out/'build/libexp_env.a')],text=True).splitlines()
    if members.count('grid_map.cpp.o')!=1:raise ValueError('GridMap archive member missing/ambiguous')
    r=recipe(out);verify_recipe(r)
    m=copy.deepcopy(parent)
    m.update(prepared_only=True,parent_build=str(BASE),source_root=str(out/'source'),
        delivery_optimization_experiment=True,
        delivery_parent_manifest_sha256=sha(BASE/'build_manifest.json'),
        delivery_implementation_sha256={name:sha(HERE/name) for name in IMPLEMENTATION},
        delivery_reused_artifacts_sha256={name:sha(out/'build'/name) for name in reuse},
        delivery_env_original_members=members,delivery_recipe=r,
        delivery_source_changes=['poly_traj_optimizer.cpp','grid_map.cpp',*HELPERS],
        delivery_switches=['DELIVERY_FAST_QUERY','DELIVERY_ACCEPT_CANDIDATE'],
        delivery_preserved=dict(wheel_limits=True,collision_geometry=True,native_acceptance=True,
            latest_data_fsm_checks=True,optimizer_budget=True,planner_cooldown=True,mppi=True),
        production_modified=False,production_default=False,
        scope='isolated fixed-arm numerical SIL delivery optimization; not deployed to real robot',
        isolated_source_sha256=source_inventory(out/'source'))
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    check(out);return out


def check(out=DEFAULT):
    out=Path(out).resolve();m=json.loads((out/'build_manifest.json').read_text())
    check_parent(BASE);checked_hash(BASE/'build_manifest.json',m['delivery_parent_manifest_sha256'])
    for name,digest in m['delivery_implementation_sha256'].items():checked_hash(HERE/name,digest)
    if source_inventory(out/'source')!=m['isolated_source_sha256']:raise ValueError('Child source changed')
    if recipe(out)!=m['delivery_recipe']:raise ValueError('Compile/link recipe changed')
    verify_recipe(m['delivery_recipe'])
    for name,digest in m['delivery_reused_artifacts_sha256'].items():
        checked_hash(out/'build'/name,digest);checked_hash(BASE/'build'/name,digest)
    members=subprocess.check_output(['ar','t',str(out/'build/libexp_env.a')],text=True).splitlines()
    if members!=m['delivery_env_original_members']:raise ValueError('Environment archive inventory changed')
    # Compare actual archive bytes of every reused member, not just filenames.
    for member in members:
        if member=='grid_map.cpp.o':continue
        old=subprocess.check_output(['ar','p',str(FROZEN/'build/libexp_env.a'),member])
        new=subprocess.check_output(['ar','p',str(out/'build/libexp_env.a'),member])
        if old!=new:raise ValueError('Unexpected environment member change: '+member)
    if not m['prepared_only']:
        for name,digest in m['delivery_archives_sha256'].items():checked_hash(out/'build'/name,digest)
        check_build(out)
    return m


def compile_prepared(out=DEFAULT):
    out=Path(out).resolve();m=check(out)
    if not m['prepared_only']:raise ValueError('Already compiled')
    r=m['delivery_recipe']
    for item in r['compile_commands']:
        print('COMPILE',item['target'],Path(item['source']).name,flush=True)
        subprocess.run(item['command'],check=True)
    for cmd in r['archive_commands']:subprocess.run(cmd,check=True)
    for cmd in r['link_commands'].values():subprocess.run(cmd,cwd=r['cwd'],check=True)
    check(out)
    m.update(prepared_only=False,native_binaries_sha256={n:sha(out/'build'/n) for n in NATIVE},
        mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
        urdf_spheres_b_archive_sha256={n:sha(out/'build'/n) for n in ARCHIVES},
        delivery_archives_sha256={n:sha(out/'build'/n) for n in (*ARCHIVES,'libexp_env.a')})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n')
    check(out);return out


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=DEFAULT)
    g=p.add_mutually_exclusive_group();g.add_argument('--compile',action='store_true');g.add_argument('--check',action='store_true')
    a=p.parse_args()
    result=compile_prepared(a.out) if a.compile else check(a.out) if a.check else prepare(a.out)
    print('CHECKED' if a.check else result)
