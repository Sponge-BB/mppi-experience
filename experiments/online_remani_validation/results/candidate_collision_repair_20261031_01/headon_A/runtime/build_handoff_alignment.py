#!/usr/bin/env python3
"""Isolated rejection-source telemetry; original decisions and MPPI retained."""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import subprocess

import build_frontend_rejoin_v2 as parent
from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import FROZEN, recipes, verify_recipe
from native_collision_runtime import replace_once
from run_coupled_objective import check_build

HERE = Path(__file__).resolve().parent
BASE = HERE / 'builds/frontend_rejoin_20261031_02'
OUT = HERE / 'builds/handoff_alignment_20261031_01'
MAP_HEADER = FROZEN / 'source/mm_remani_planner/plan_env/include/plan_env/grid_map.h'
REBUILT = ('poly_traj_optimizer.cpp.o', 'libexp_opt.a', 'remani_planner_node')

FEEDBACK_TRACE = r'''
    if (handoff_alignment::enabled()) {
      const auto& frozen = coupled_optimizer.prediction;
      std::ostringstream rows; rows << std::setprecision(17) << '[';
      for (std::size_t i=0;i<frozen.seed.rows.size();++i) {
        if(i)rows<<','; rows<<'[';
        for(std::size_t j=0;j<8;++j){if(j)rows<<',';rows<<frozen.seed.rows[i][j];}
        rows<<']';
      }
      rows<<']';
      dual_probe::emit("handoff_feedback_input",dual_probe::Fields()
        .add("scope_id",coupled_optimizer.id)
        .add("prediction_valid",frozen.valid?"true":"false").str("prediction_reason",frozen.reason)
        .add("request_time",frozen.request_time).add("seed_stamp",frozen.seed.stamp)
        .add("seed_odom_stamp",frozen.seed.odom).add("seed_dynamic_stamp",frozen.seed.dynamic)
        .add("seed_revision",frozen.seed.revision).add("seed_generation",frozen.seed.generation)
        .add("execution_weight",coupled_optimizer.config.execution_weight)
        .add("handover_weight",coupled_optimizer.config.delivery_weight)
        .add("frozen_rows",rows.str()).add("input_hash",input_hash)
        .str("semantics","frozen checked MPPI prediction; soft spatial targets, not committed future states"));
    }
'''


def transform(text):
    text = replace_once(text, '#include "early_delivery_gate.hpp"',
                        '#include "early_delivery_gate.hpp"\n#include "handoff_alignment_probe.hpp"')
    anchor = '    coupling_bridge::Optimizer coupled_optimizer(input_hash,initT_container.size(),requested_pieces,singul_container);'
    text = replace_once(text, anchor, anchor + '\n' + FEEDBACK_TRACE)
    text = replace_once(text,
        '      if(checkPlanningCollision(traj_data, t, coll_type)){\n',
        '      if(checkPlanningCollision(traj_data, t, coll_type)){\n'
        '        handoff_alignment::planningReject(traj_data,t,coll_type,mm_config_,grid_map_);\n')
    return replace_once(text, '    good_traj = IsTrajSafe(singul_traj_data);',
        '    { handoff_alignment::FinalGate final_gate;\n'
        '      good_traj = IsTrajSafe(singul_traj_data); }')


def parent_check():
    parent.OUT = BASE
    return parent.check()


def prepare():
    manifest = parent_check()
    OUT.mkdir(exist_ok=False)
    shutil.copytree(BASE / 'source', OUT / 'source')
    (OUT / 'build').mkdir()
    path = OUT / 'source/poly_traj_optimizer.cpp'
    path.write_text(transform(path.read_text()))
    shutil.copy2(HERE / 'handoff_alignment_probe.hpp', OUT / 'source/handoff_alignment_probe.hpp')
    path = OUT / 'source/include/plan_env/grid_map.h'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(replace_once(MAP_HEADER.read_text(), '  friend class GridMapParallelTestAccessor;',
        '  friend class GridMapParallelTestAccessor;\n  friend class HandoffAlignmentMapAccessor;'))
    reused = {}
    for path in (BASE / 'build').iterdir():
        if path.name not in REBUILT:
            os.link(path, OUT / 'build' / path.name)
            reused[path.name] = sha(path)
    recipe = recipes(OUT)
    recipe['compile_commands'] = [c for c in recipe['compile_commands']
                                  if Path(c['source']).name == 'poly_traj_optimizer.cpp']
    recipe['archive_commands'] = [c for c in recipe['archive_commands'] if c[2].endswith('/libexp_opt.a')]
    recipe['link_commands'] = {'remani_planner_node': recipe['link_commands']['remani_planner_node']}
    link = recipe['link_commands']['remani_planner_node']
    link[link.index(str(FROZEN / 'build/libexp_env.a'))] = str(OUT / 'build/libexp_env.a')
    link.append('-lz')
    verify_recipe(recipe)
    manifest = copy.deepcopy(manifest)
    manifest.update(prepared_only=True, source_root=str(OUT / 'source'), parent_build=str(BASE),
        handoff_alignment_experiment=True, handoff_alignment_parent_sha256=sha(BASE / 'build_manifest.json'),
        handoff_alignment_reused_sha256=reused, handoff_alignment_recipe=recipe,
        handoff_alignment_implementation_sha256={name: sha(HERE / name) for name in
            ('build_handoff_alignment.py', 'handoff_alignment_probe.hpp')},
        handoff_alignment_map_header_source=dict(path=str(MAP_HEADER), sha256=sha(MAP_HEADER)),
        handoff_alignment_scope='read-only failed-sphere provenance and frozen fast prediction; execution soft weight runtime switch',
        isolated_source_sha256=source_inventory(OUT / 'source'), production_modified=False)
    (OUT / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return check()


def check():
    parent_check()
    manifest = json.loads((OUT / 'build_manifest.json').read_text())
    checked_hash(BASE / 'build_manifest.json', manifest['handoff_alignment_parent_sha256'])
    for name, digest in manifest['handoff_alignment_implementation_sha256'].items():
        checked_hash(HERE / name, digest)
    for name, digest in manifest['handoff_alignment_reused_sha256'].items():
        checked_hash(OUT / 'build' / name, digest)
    origin = manifest['handoff_alignment_map_header_source']
    checked_hash(Path(origin['path']), origin['sha256'])
    if source_inventory(OUT / 'source') != manifest['isolated_source_sha256']:
        raise ValueError('Child sources changed')
    verify_recipe(manifest['handoff_alignment_recipe'])
    if not manifest['prepared_only']:
        for name, digest in manifest['handoff_alignment_compiled_sha256'].items():
            checked_hash(OUT / 'build' / name, digest)
        check_build(OUT)
    return manifest


def compile_child():
    manifest = check()
    if not manifest['prepared_only']:
        raise RuntimeError('Completed build is immutable')
    recipe = manifest['handoff_alignment_recipe']
    for command in recipe['compile_commands']:
        print('COMPILE', Path(command['source']).name, flush=True)
        subprocess.run(command['command'], cwd=recipe['cwd'], check=True)
    for command in recipe['archive_commands']:
        subprocess.run(command, cwd=recipe['cwd'], check=True)
    subprocess.run(recipe['link_commands']['remani_planner_node'], cwd=recipe['cwd'], check=True)
    manifest.update(prepared_only=False,
        native_binaries_sha256={name: sha(OUT / 'build' / name) for name in NATIVE},
        handoff_alignment_compiled_sha256={name: sha(OUT / 'build' / name) for name in REBUILT})
    (OUT / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return check()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('action', choices=('prepare', 'compile', 'check'))
    args = parser.parse_args()
    {'prepare': prepare, 'compile': compile_child, 'check': check}[args.action]()
    print('HANDOFF_ALIGNMENT_BUILD', args.action, OUT, 'OK', flush=True)
