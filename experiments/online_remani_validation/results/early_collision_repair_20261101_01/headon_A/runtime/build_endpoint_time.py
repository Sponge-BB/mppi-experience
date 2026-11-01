#!/usr/bin/env python3
"""New optimizer-only endpoint-time seed diagnostic; never mutate its parent."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

from build_shared_local_safety import NATIVE, checked_hash, sha
from build_feasibility_stop import source_inventory
from build_urdf_spheres_b import ARCHIVES, verify_recipe
from build_wheel_cause import check as check_parent, recipe as wheel_recipe, REBUILT
from run_coupled_objective import check_build, check_space
from patch_endpoint_time import transform_optimizer, transform_finish

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/wheel_cause_20261030_04'
DEFAULT=HERE/'builds/endpoint_time_20261030_01'
HELPERS=('endpoint_time_replay.hpp','endpoint_time_replay.inc')
IMPLEMENTATION=('build_endpoint_time.py','patch_endpoint_time.py',*HELPERS)
RESERVE_BYTES=250*1024**2
BUILD_ALLOWANCE_BYTES=12*1024**2


def recipe(out):
    return wheel_recipe(Path(out).resolve())


def _artifacts():
    paths=sorted((BASE/'build').iterdir())
    if any(path.is_symlink() or not path.is_file() for path in paths):
        raise ValueError('Immutable parent must contain regular artifacts only')
    if not set(REBUILT).issubset(path.name for path in paths):
        raise ValueError('Parent rebuilt artifacts are missing')
    return [path.name for path in paths if path.name not in REBUILT]


def _space(path):
    free=check_space(path,RESERVE_BYTES+BUILD_ALLOWANCE_BYTES)
    print('STORAGE free_mib=%d reserve_mib=250 build_allowance_mib=12'%(free//1024**2),flush=True)
    return free


def prepare(out=DEFAULT):
    out=Path(out).resolve()
    if out.exists():raise FileExistsError(out)
    if BASE in out.parents or out in BASE.parents:raise ValueError('Child must be independent of parent')
    parent=check_parent(BASE)
    if parent.get('prepared_only'):raise ValueError('Completed ownership-fixed parent required')
    free=_space(HERE);reused=_artifacts();r=recipe(out);verify_recipe(r)
    out.mkdir(parents=True);shutil.copytree(BASE/'source',out/'source');(out/'build').mkdir()
    for name in HELPERS:shutil.copy2(HERE/name,out/'source'/name)
    for name,fn in (('poly_traj_optimizer.cpp',transform_optimizer),('candidate_audit_finish.inc',transform_finish)):
        path=out/'source'/name;path.write_text(fn(path.read_text()))
    for name in reused:os.link(BASE/'build'/name,out/'build'/name)
    m=copy.deepcopy(parent)
    m.update(prepared_only=True,parent_build=str(BASE),source_root=str(out/'source'),
        endpoint_time_experiment=True,
        endpoint_time_parent_manifest_sha256=sha(BASE/'build_manifest.json'),
        endpoint_time_implementation_sha256={name:sha(HERE/name) for name in IMPLEMENTATION},
        endpoint_time_reused_artifacts_sha256={name:sha(out/'build'/name) for name in reused},
        endpoint_time_reuse_mode='immutable hardlinks excluding every compiler/archive/link output',
        endpoint_time_recipe=r,endpoint_time_rebuilt_artifacts=list(REBUILT),
        endpoint_time_source_changes=['poly_traj_optimizer.cpp','candidate_audit_finish.inc',*HELPERS],
        endpoint_time_storage=dict(free_before_prepare_bytes=free,reserve_bytes=RESERVE_BYTES,
            build_allowance_bytes=BUILD_ALLOWANCE_BYTES,historical_files_deleted=False),
        endpoint_time_groups=[dict(label='original_060',budget_s=.6,endpoint_time_fix=False),
                              dict(label='endpoint150_060',budget_s=.6,endpoint_time_fix=True)],
        endpoint_time_scope='paired diagnostic seed transformation; default-off child frontend writeback gate; no historical-case import',
        endpoint_time_child_frontend_fix_available=True,endpoint_time_online_capture_frontend_fix_enabled=False,
        endpoint_time_frontend_fix_env='ENDPOINT_TIME_FIX (unset/0 is original output; 1 writes scaled t_list back once)',
        production_modified=False,production_default=False,
        isolated_source_sha256=source_inventory(out/'source'))
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');check(out);return out


def check(out=DEFAULT):
    out=Path(out).resolve();m=json.loads((out/'build_manifest.json').read_text())
    if not m.get('endpoint_time_experiment'):raise ValueError('Not an endpoint-time child')
    check_parent(BASE);checked_hash(BASE/'build_manifest.json',m['endpoint_time_parent_manifest_sha256'])
    for name,digest in m['endpoint_time_implementation_sha256'].items():checked_hash(HERE/name,digest)
    if source_inventory(out/'source')!=m['isolated_source_sha256']:raise ValueError('Child source changed')
    if recipe(out)!=m['endpoint_time_recipe']:raise ValueError('Recipe changed')
    verify_recipe(m['endpoint_time_recipe']);reused=m['endpoint_time_reused_artifacts_sha256']
    if sorted(reused)!=_artifacts() or set(reused).intersection(REBUILT):raise ValueError('Unsafe artifact inventory')
    for name,digest in reused.items():
        child,parent=out/'build'/name,BASE/'build'/name
        if child.is_symlink() or not child.samefile(parent):raise ValueError('Parent hardlink lost: '+name)
        checked_hash(child,digest);checked_hash(parent,digest)
    if not m['prepared_only']:
        for name,digest in m['endpoint_time_compiled_artifacts_sha256'].items():checked_hash(out/'build'/name,digest)
        check_build(out)
    return m


def compile_prepared(out=DEFAULT):
    out=Path(out).resolve();m=check(out)
    if not m['prepared_only']:raise ValueError('Already compiled; use a new child for changes')
    _space(out)
    for name in REBUILT:
        path=out/'build'/name
        if path.exists() or path.is_symlink():raise FileExistsError('Retain partial output: '+str(path))
    r=m['endpoint_time_recipe']
    for item in r['compile_commands']:
        print('COMPILE',item['target'],Path(item['source']).name,flush=True);subprocess.run(item['command'],check=True)
    for command in r['archive_commands']:subprocess.run(command,check=True)
    for command in r['link_commands'].values():subprocess.run(command,cwd=r['cwd'],check=True)
    check(out)
    m.update(prepared_only=False,native_binaries_sha256={name:sha(out/'build'/name) for name in NATIVE},
        mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
        urdf_spheres_b_archive_sha256={name:sha(out/'build'/name) for name in ARCHIVES},
        endpoint_time_compiled_artifacts_sha256={name:sha(out/'build'/name) for name in REBUILT})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');check(out);return out


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,default=DEFAULT)
    g=p.add_mutually_exclusive_group();g.add_argument('--compile',action='store_true');g.add_argument('--check',action='store_true')
    g.add_argument('--print-recipe',action='store_true');a=p.parse_args()
    if a.print_recipe:print(json.dumps(recipe(a.out),indent=2))
    else:
        result=compile_prepared(a.out) if a.compile else check(a.out) if a.check else prepare(a.out)
        print('CHECKED' if a.check else result)
