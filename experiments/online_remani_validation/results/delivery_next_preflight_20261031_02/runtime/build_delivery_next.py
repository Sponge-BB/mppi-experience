#!/usr/bin/env python3
"""Three-object isolated child for delivery exit and measured gear association."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

from build_cheap_scaling import check as check_parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import ARCHIVES, recipes, verify_recipe, FROZEN
from run_coupled_objective import check_build
from patch_delivery_next_fsm import transform_fsm

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/cheap_scaling_20261030_01'
DEFAULT=HERE/'builds/delivery_next_20261031_01'
REBUILT=('poly_traj_optimizer.cpp.o','remani_replan_fsm.cpp.o','online_api.cpp.o',
         'libexp_opt.a','remani_planner_node','libonline_mppi.so')


def recipe(out):
    r=recipes(out)
    wanted={'poly_traj_optimizer.cpp','remani_replan_fsm.cpp','online_api.cpp'}
    r['compile_commands']=[c for c in r['compile_commands'] if Path(c['source']).name in wanted]
    r['archive_commands']=[c for c in r['archive_commands'] if c[2].endswith('/libexp_opt.a')]
    link=r['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN/'build/libexp_env.a'))]=str(Path(out).resolve()/'build/libexp_env.a')
    link.append('-lz')
    return r


def implementation_files():
    return ('build_delivery_next.py','patch_delivery_next_fsm.py',
            'early_delivery_patch.py','early_delivery_gate.hpp','cusp_phase_patch.py','cusp_phase_cursor.hpp')


def prepare(out=DEFAULT):
    from early_delivery_patch import transform_optimizer, transform_header
    from cusp_phase_patch import transform_kernel, transform_v12, transform_api
    out=Path(out).resolve()
    if out.exists():raise FileExistsError(out)
    parent=check_parent(BASE)
    out.mkdir(parents=True);shutil.copytree(BASE/'source',out/'source');(out/'build').mkdir()
    edits={'poly_traj_optimizer.cpp':transform_optimizer,
           'include/optimizer/poly_traj_optimizer.hpp':transform_header,
           'remani_replan_fsm.cpp':transform_fsm,
           'online_kernel.hpp':transform_kernel,'mppi_v12.hpp':transform_v12,
           'online_api.cpp':transform_api}
    for name,transform in edits.items():
        p=out/'source'/name;p.write_text(transform(p.read_text()))
    for name in ('early_delivery_gate.hpp','cusp_phase_cursor.hpp'):
        shutil.copy2(HERE/name,out/'source'/name)
    reused={}
    for p in sorted((BASE/'build').iterdir()):
        if p.name not in REBUILT:
            os.link(p,out/'build'/p.name);reused[p.name]=sha(p)
    r=recipe(out);verify_recipe(r)
    m=copy.deepcopy(parent)
    m.update(prepared_only=True,source_root=str(out/'source'),parent_build=str(BASE),
        delivery_next_experiment=True,delivery_next_parent_manifest_sha256=sha(BASE/'build_manifest.json'),
        delivery_next_implementation_sha256={n:sha(HERE/n) for n in implementation_files()},
        delivery_next_recipe=r,delivery_next_reused_sha256=reused,
        delivery_next_changes=list(edits)+['early_delivery_gate.hpp','cusp_phase_cursor.hpp'],
        isolated_source_sha256=source_inventory(out/'source'),production_modified=False,production_default=False,
        scope='fixed-arm numerical ROS SIL; measured gear association and full-native feasible early exit')
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');return out


def check(out=DEFAULT):
    out=Path(out).resolve();m=json.loads((out/'build_manifest.json').read_text())
    check_parent(BASE);checked_hash(BASE/'build_manifest.json',m['delivery_next_parent_manifest_sha256'])
    if source_inventory(out/'source')!=m['isolated_source_sha256']:raise ValueError('Child source changed')
    for n,d in m['delivery_next_implementation_sha256'].items():checked_hash(HERE/n,d)
    verify_recipe(m['delivery_next_recipe'])
    for n,d in m['delivery_next_reused_sha256'].items():
        checked_hash(out/'build'/n,d)
        if not (out/'build'/n).samefile(BASE/'build'/n):raise ValueError('Immutable reuse changed')
    if not m['prepared_only']:
        for n,d in m['delivery_next_compiled_sha256'].items():checked_hash(out/'build'/n,d)
        check_build(out)
    return m


def compile_prepared(out=DEFAULT):
    out=Path(out).resolve();m=check(out)
    if not m['prepared_only']:raise ValueError('Already compiled')
    for n in REBUILT:
        if (out/'build'/n).exists():raise FileExistsError(out/'build'/n)
    r=m['delivery_next_recipe']
    for c in r['compile_commands']:
        print('COMPILE',Path(c['source']).name,flush=True);subprocess.run(c['command'],check=True)
    for c in r['archive_commands']:subprocess.run(c,check=True)
    for c in r['link_commands'].values():subprocess.run(c,cwd=r['cwd'],check=True)
    m.update(prepared_only=False,native_binaries_sha256={n:sha(out/'build'/n) for n in NATIVE},
        mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
        urdf_spheres_b_archive_sha256={n:sha(out/'build'/n) for n in ARCHIVES},
        delivery_next_compiled_sha256={n:sha(out/'build'/n) for n in REBUILT})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');check(out);return out


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,default=DEFAULT)
    g=p.add_mutually_exclusive_group();g.add_argument('--compile',action='store_true');g.add_argument('--check',action='store_true')
    a=p.parse_args();print(check(a.out) if a.check else compile_prepared(a.out) if a.compile else prepare(a.out))
