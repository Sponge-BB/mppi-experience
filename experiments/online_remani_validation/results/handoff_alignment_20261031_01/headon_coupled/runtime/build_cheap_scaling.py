#!/usr/bin/env python3
"""Build one optimizer-only cheap-coordinate child, reusing immutable products."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

from build_endpoint_time import check as check_parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import ARCHIVES, verify_recipe
from build_wheel_cause import REBUILT, recipe
from patch_cheap_scaling import transform_header, transform_optimizer
from run_coupled_objective import check_build

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/endpoint_time_20261030_01'
DEFAULT=HERE/'builds/cheap_scaling_20261030_01'
HEADER=HERE.parents[1]/'src/mm_remani_planner/traj_opt/include/optimizer/poly_traj_optimizer.hpp'
IMPLEMENTATION=('build_cheap_scaling.py','patch_cheap_scaling.py','cheap_variable_scaling.hpp')
RESERVE=100*1024**2
ALLOWANCE=12*1024**2


def space(path):
    free=shutil.disk_usage(path).free
    if free<RESERVE+ALLOWANCE:raise RuntimeError('Need 100MiB reserve plus 12MiB build allowance')
    print('STORAGE free_mib=%d reserve_mib=100 build_allowance_mib=12' % (free//1024**2),flush=True)
    return free


def artifacts():
    return [p.name for p in sorted((BASE/'build').iterdir()) if p.name not in REBUILT]


def prepare(out=DEFAULT):
    out=Path(out).resolve()
    if out.exists():raise FileExistsError(out)
    parent=check_parent(BASE)
    if parent.get('prepared_only'):raise ValueError('Completed parent required')
    free=space(HERE); r=recipe(out);verify_recipe(r)
    out.mkdir(parents=True);shutil.copytree(BASE/'source',out/'source');(out/'build').mkdir()
    shutil.copy2(HERE/'cheap_variable_scaling.hpp',out/'source/cheap_variable_scaling.hpp')
    copied=out/'source/include/optimizer/poly_traj_optimizer.hpp';copied.parent.mkdir(parents=True,exist_ok=True)
    copied.write_text(transform_header(HEADER.read_text()))
    optimizer=out/'source/poly_traj_optimizer.cpp';optimizer.write_text(transform_optimizer(optimizer.read_text()))
    reused=artifacts()
    for name in reused:os.link(BASE/'build'/name,out/'build'/name)
    m=copy.deepcopy(parent)
    m.update(prepared_only=True,source_root=str(out/'source'),parent_build=str(BASE),
        cheap_scaling_experiment=True,cheap_scaling_parent_manifest_sha256=sha(BASE/'build_manifest.json'),
        cheap_scaling_implementation_sha256={n:sha(HERE/n) for n in IMPLEMENTATION},
        cheap_scaling_original_header_sha256=sha(HEADER),cheap_scaling_recipe=r,
        cheap_scaling_reused_artifacts_sha256={n:sha(out/'build'/n) for n in reused},
        cheap_scaling_storage=dict(free_before_prepare_bytes=free,reserve_bytes=RESERVE,build_allowance_bytes=ALLOWANCE),
        cheap_scaling_source_changes=['poly_traj_optimizer.cpp','include/optimizer/poly_traj_optimizer.hpp','cheap_variable_scaling.hpp'],
        cheap_scaling_switch='CHEAP_VARIABLE_SCALING (default 0)',
        cheap_scaling_scope='fixed-arm original initialization; proxy only for numerical scale; full native objective/gates unchanged; preparation included in budget',
        solver_overlay_pin_without_heavy_audit=True,production_modified=False,production_default=False,
        isolated_source_sha256=source_inventory(out/'source'))
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');check(out);return out


def check(out=DEFAULT):
    out=Path(out).resolve();m=json.loads((out/'build_manifest.json').read_text())
    if not m.get('cheap_scaling_experiment'):raise ValueError('Not cheap-scaling child')
    check_parent(BASE);checked_hash(BASE/'build_manifest.json',m['cheap_scaling_parent_manifest_sha256'])
    checked_hash(HEADER,m['cheap_scaling_original_header_sha256'])
    for n,d in m['cheap_scaling_implementation_sha256'].items():checked_hash(HERE/n,d)
    if source_inventory(out/'source')!=m['isolated_source_sha256']:raise ValueError('Child source changed')
    if recipe(out)!=m['cheap_scaling_recipe']:raise ValueError('Recipe changed')
    verify_recipe(m['cheap_scaling_recipe'])
    for n,d in m['cheap_scaling_reused_artifacts_sha256'].items():
        if not (out/'build'/n).samefile(BASE/'build'/n):raise ValueError('Immutable hardlink lost: '+n)
        checked_hash(out/'build'/n,d)
    if not m['prepared_only']:
        for n,d in m['cheap_scaling_compiled_artifacts_sha256'].items():checked_hash(out/'build'/n,d)
        check_build(out)
    return m


def compile_prepared(out=DEFAULT):
    out=Path(out).resolve();m=check(out)
    if not m['prepared_only']:raise ValueError('Already compiled')
    space(out)
    for n in REBUILT:
        if (out/'build'/n).exists():raise FileExistsError('Retain partial build: '+str(out/'build'/n))
    r=m['cheap_scaling_recipe']
    for item in r['compile_commands']:subprocess.run(item['command'],check=True)
    for command in r['archive_commands']:subprocess.run(command,check=True)
    for command in r['link_commands'].values():subprocess.run(command,cwd=r['cwd'],check=True)
    m.update(prepared_only=False,native_binaries_sha256={n:sha(out/'build'/n) for n in NATIVE},
        mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
        urdf_spheres_b_archive_sha256={n:sha(out/'build'/n) for n in ARCHIVES},
        cheap_scaling_compiled_artifacts_sha256={n:sha(out/'build'/n) for n in REBUILT})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');check(out);return out


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,default=DEFAULT)
    g=p.add_mutually_exclusive_group();g.add_argument('--compile',action='store_true');g.add_argument('--check',action='store_true')
    a=p.parse_args();print(check(a.out) if a.check else compile_prepared(a.out) if a.compile else prepare(a.out))
