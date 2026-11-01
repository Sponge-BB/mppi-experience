#!/usr/bin/env python3
"""Add dynamic-map version evidence and pin the existing KD for posterior audit."""
import argparse
import copy
import json
from pathlib import Path
import shutil
import subprocess

from build_shared_local_safety import NATIVE, checked_hash, sha
from build_feasibility_stop import source_inventory
from build_urdf_spheres_b import recipes, verify_recipe, ARCHIVES, PLANNER, FROZEN
from build_candidate_audit import check as check_parent
from run_coupled_objective import check_build
from patch_candidate_audit_consistency import transform_helper, transform_optimizer, transform_finish

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/candidate_audit_20261030_01'
DEFAULT=HERE/'builds/candidate_audit_20261030_02'
IMPLEMENTATION=('build_candidate_audit_consistency.py','patch_candidate_audit_consistency.py')


def recipe(out):
    r=recipes(out)
    r['compile_commands']=[x for x in r['compile_commands'] if x['target']=='exp_opt']
    r['archive_commands']=[x for x in r['archive_commands'] if x[2].endswith('/libexp_opt.a')]
    link=r['link_commands']['remani_planner_node']
    old=str(FROZEN/'build/libexp_env.a')
    if link.count(old)!=1:raise ValueError('Expected exactly one environment archive')
    link[link.index(old)]=str(out/'build/libexp_env.a');link.append('-lz')
    r['link_commands']={'remani_planner_node':link};return r


def prepare(out=DEFAULT):
    out=Path(out).resolve()
    if out.exists():raise FileExistsError(out)
    parent=check_parent(BASE)
    out.mkdir(parents=True);shutil.copytree(BASE/'source',out/'source');(out/'build').mkdir()
    for name,fn in (('candidate_audit.hpp',transform_helper),('poly_traj_optimizer.cpp',transform_optimizer),
                    ('candidate_audit_finish.inc',transform_finish)):
        source=out/'source'/name;source.write_text(fn(source.read_text()))
    reuse=['mm_controller_node','dynamic_obstacle_tracker_node','libonline_mppi.so',
           'libexp_model.a','libexp_search.a','libexp_env.a']
    reuse += [name+'.o' for name in PLANNER]
    for name in reuse:shutil.copy2(BASE/'build'/name,out/'build'/name)
    r=recipe(out);verify_recipe(r)
    m=copy.deepcopy(parent)
    m.update(prepared_only=True,parent_build=str(BASE),source_root=str(out/'source'),
        candidate_audit_consistency_experiment=True,
        candidate_audit_consistency_parent_manifest_sha256=sha(BASE/'build_manifest.json'),
        candidate_audit_consistency_implementation_sha256={name:sha(HERE/name) for name in IMPLEMENTATION},
        candidate_audit_consistency_reused_artifacts_sha256={name:sha(out/'build'/name) for name in reuse},
        candidate_audit_consistency_recipe=r,
        scope='native rejected-candidate diagnosis with actual dynamic overlay version history; posterior audit owns frozen existing KD',
        isolated_source_sha256=source_inventory(out/'source'))
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');check(out);return out


def check(out=DEFAULT):
    out=Path(out).resolve();m=json.loads((out/'build_manifest.json').read_text())
    check_parent(BASE);checked_hash(BASE/'build_manifest.json',m['candidate_audit_consistency_parent_manifest_sha256'])
    for name,digest in m['candidate_audit_consistency_implementation_sha256'].items():checked_hash(HERE/name,digest)
    if source_inventory(out/'source')!=m['isolated_source_sha256']:raise ValueError('Child source changed')
    if recipe(out)!=m['candidate_audit_consistency_recipe']:raise ValueError('Recipe changed')
    verify_recipe(m['candidate_audit_consistency_recipe'])
    for name,digest in m['candidate_audit_consistency_reused_artifacts_sha256'].items():
        checked_hash(out/'build'/name,digest);checked_hash(BASE/'build'/name,digest)
    if not m['prepared_only']:
        for name,digest in m['candidate_audit_consistency_archives_sha256'].items():checked_hash(out/'build'/name,digest)
        check_build(out)
    return m


def compile_prepared(out=DEFAULT):
    out=Path(out).resolve();m=check(out)
    if not m['prepared_only']:raise ValueError('Already compiled')
    r=m['candidate_audit_consistency_recipe']
    for item in r['compile_commands']:
        print('COMPILE',item['target'],Path(item['source']).name,flush=True)
        subprocess.run(item['command'],check=True)
    for cmd in r['archive_commands']:subprocess.run(cmd,check=True)
    for cmd in r['link_commands'].values():subprocess.run(cmd,cwd=r['cwd'],check=True)
    check(out)
    m.update(prepared_only=False,native_binaries_sha256={n:sha(out/'build'/n) for n in NATIVE},
        mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
        urdf_spheres_b_archive_sha256={n:sha(out/'build'/n) for n in ARCHIVES},
        candidate_audit_consistency_archives_sha256={n:sha(out/'build'/n) for n in (*ARCHIVES,'libexp_env.a')})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');check(out);return out


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=DEFAULT)
    g=p.add_mutually_exclusive_group();g.add_argument('--compile',action='store_true');g.add_argument('--check',action='store_true')
    a=p.parse_args()
    result=compile_prepared(a.out) if a.compile else check(a.out) if a.check else prepare(a.out)
    print('CHECKED' if a.check else result)
