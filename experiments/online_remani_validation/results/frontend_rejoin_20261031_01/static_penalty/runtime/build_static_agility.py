#!/usr/bin/env python3
"""MPPI-only child of the measured state-handoff build; production unchanged."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
BASE = HERE/'builds/state_handoff_20261031_01'
OUT = HERE/'builds/static_agility_20261031_01'
sys.path.insert(0, str(HERE/'results/state_handoff_ab_20261031_01/tools'))
import build_state_handoff as parent
from build_feasibility_stop import source_inventory
from build_urdf_spheres_b import recipes, verify_recipe
from build_shared_local_safety import sha, checked_hash
from run_coupled_objective import check_build


def prepare():
    m = parent.check()
    OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE/'source', OUT/'source')
    (OUT/'build').mkdir()
    reused = {}
    for p in (BASE/'build').iterdir():
        if p.name not in ('online_api.cpp.o', 'libonline_mppi.so'):
            os.link(p, OUT/'build'/p.name)
            reused[p.name] = sha(p)
    m = copy.deepcopy(m)
    m.update(prepared_only=True, source_root=str(OUT/'source'),
             static_agility_parent=str(BASE), static_agility_parent_sha256=sha(BASE/'build_manifest.json'),
             static_agility_reused=reused, production_modified=False,
             static_agility_scope='Static proposal activation only; all collision/actuator gates unchanged')
    (OUT/'build_manifest.json').write_text(json.dumps(m, indent=2)+'\n')


def check():
    parent.check()
    m = json.loads((OUT/'build_manifest.json').read_text())
    checked_hash(BASE/'build_manifest.json', m['static_agility_parent_sha256'])
    for name, digest in m['static_agility_reused'].items():
        checked_hash(OUT/'build'/name, digest)
    if not m['prepared_only']:
        if source_inventory(OUT/'source') != m['isolated_source_sha256']:
            raise ValueError('Child sources changed after compilation')
        check_build(OUT)
    return m


def compile_child():
    m = check()
    if not m['prepared_only']:
        raise RuntimeError('Completed build is immutable')
    r = recipes(OUT)
    verify_recipe(r)
    command = next(c['command'] for c in r['compile_commands'] if c['target']=='online_mppi')
    subprocess.run(command, cwd=r['cwd'], check=True)
    subprocess.run(r['link_commands']['online_mppi'], cwd=r['cwd'], check=True)
    m.update(prepared_only=False, mppi_library_sha256=sha(OUT/'build/libonline_mppi.so'),
             isolated_source_sha256=source_inventory(OUT/'source'),
             static_agility_compile=command, static_agility_link=r['link_commands']['online_mppi'])
    (OUT/'build_manifest.json').write_text(json.dumps(m, indent=2)+'\n')
    check()
    print('STATIC_AGILITY_BUILD_COMPLETE', OUT, flush=True)


if __name__ == '__main__':
    p=argparse.ArgumentParser();p.add_argument('action', choices=['prepare','compile','check'])
    p.add_argument('--out',type=Path,default=OUT)
    a=p.parse_args()
    OUT=a.out.resolve()
    {'prepare':prepare,'compile':compile_child,'check':check}[a.action]()
