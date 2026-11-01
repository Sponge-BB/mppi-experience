#!/usr/bin/env python3
"""Fix only capture serialization; preserve failed run/build and reuse planner objects."""
import argparse
import copy
import json
from pathlib import Path
import shutil
import subprocess
from build_state_contract import Transformer
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_feasibility_stop import source_inventory
from build_urdf_spheres_b import recipes, verify_recipe, ARCHIVES, PLANNER
from build_native67_probe import check as check_parent
from run_coupled_objective import check_build

HERE=Path(__file__).resolve().parent
BASE=HERE/'builds/native67_probe_20261030_01'
DEFAULT=HERE/'builds/native67_probe_20261030_02'
OLD='delivery_diag::frameJson({frozen_frame.snapshot,frozen_frame.now})'
NEW='''(std::string("{\\\"now\\\":")+delivery_replay::num(frozen_frame.now)+
          ",\\\"sequence\\\":"+delivery_replay::num(frozen_frame.snapshot?frozen_frame.snapshot->sequence:0.)+
          ",\\\"stamp\\\":"+delivery_replay::num(frozen_frame.snapshot?frozen_frame.snapshot->stamp:0.)+
          ",\\\"snapshot_present\\\":"+(frozen_frame.snapshot?"true":"false")+
          ",\\\"snapshot_payload_dumped\\\":false}")'''


def transform(text):
    t=Transformer(text);t.change(OLD,NEW);return t.finish()


def prepare(out=DEFAULT):
    out=Path(out).resolve()
    if out.exists():raise FileExistsError(out)
    parent=check_parent(BASE)
    out.mkdir(parents=True);shutil.copytree(BASE/'source',out/'source');(out/'build').mkdir()
    source=out/'source/poly_traj_optimizer.cpp';source.write_text(transform(source.read_text()))
    r=recipes(out);r['compile_commands']=[x for x in r['compile_commands'] if x['target']=='exp_opt']
    r['archive_commands']=[x for x in r['archive_commands'] if x[2].endswith('/libexp_opt.a')]
    r['link_commands']={'remani_planner_node':r['link_commands']['remani_planner_node']}
    verify_recipe(r)
    reuse=['mm_controller_node','dynamic_obstacle_tracker_node','libonline_mppi.so','libexp_model.a','libexp_search.a']
    reuse += [name+'.o' for name in PLANNER]
    for name in reuse:shutil.copy2(BASE/'build'/name,out/'build'/name)
    m=copy.deepcopy(parent)
    m.update(prepared_only=True,parent_build=str(BASE),source_root=str(out/'source'),
      native67_lite_capture_experiment=True,native67_lite_parent_manifest_sha256=sha(BASE/'build_manifest.json'),
      native67_lite_recipe=r,native67_lite_builder_sha256=sha(Path(__file__)),
      native67_lite_reused_artifacts_sha256={name:sha(out/'build'/name) for name in reuse},
      native67_lite_source_changes=['poly_traj_optimizer.cpp'],
      native67_lite_capture_scope='frame now/sequence/stamp only; same immutable pointer, raw/esdf sequences and full dynamic snapshot checks unchanged; no diagnostic snapshot files',
      native67_recipe=r,isolated_source_sha256=source_inventory(out/'source'))
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');check(out);return out


def check(out=DEFAULT):
    out=Path(out).resolve();m=json.loads((out/'build_manifest.json').read_text())
    check_parent(BASE);checked_hash(BASE/'build_manifest.json',m['native67_lite_parent_manifest_sha256'])
    checked_hash(Path(__file__),m['native67_lite_builder_sha256'])
    if source_inventory(out/'source')!=m['isolated_source_sha256']:raise ValueError('Child source changed')
    expected=transform((BASE/'source/poly_traj_optimizer.cpp').read_text())
    if (out/'source/poly_traj_optimizer.cpp').read_text()!=expected:raise ValueError('Unexpected capture patch')
    for name,digest in m['native67_lite_reused_artifacts_sha256'].items():
        checked_hash(out/'build'/name,digest);checked_hash(BASE/'build'/name,digest)
    verify_recipe(m['native67_lite_recipe'])
    if not m['prepared_only']:check_build(out)
    return m


def compile_prepared(out=DEFAULT):
    out=Path(out).resolve();m=check(out)
    if not m['prepared_only']:raise ValueError('Already built')
    r=m['native67_lite_recipe']
    for item in r['compile_commands']:
        print('COMPILE',item['target'],Path(item['source']).name,flush=True)
        subprocess.run(item['command'],check=True)
    for c in r['archive_commands']:subprocess.run(c,check=True)
    for c in r['link_commands'].values():subprocess.run(c,cwd=r['cwd'],check=True)
    check(out)
    m.update(prepared_only=False,native_binaries_sha256={n:sha(out/'build'/n) for n in NATIVE},
      mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
      urdf_spheres_b_archive_sha256={n:sha(out/'build'/n) for n in ARCHIVES})
    (out/'build_manifest.json').write_text(json.dumps(m,indent=2)+'\n');check(out);return out


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,default=DEFAULT)
    g=p.add_mutually_exclusive_group();g.add_argument('--compile',action='store_true');g.add_argument('--check',action='store_true')
    a=p.parse_args()
    print(compile_prepared(a.out) if a.compile else check(a.out) if a.check else prepare(a.out))
