#!/usr/bin/env python3
"""Derive a diagnostic-only build from frozen coupling build04, not production."""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess

from build_native_v9 import HERE, sha, replace
from build_native_v10 import compile_source
from audit_dual_timescale import build_integrity


def build(base, out):
    base, out = base.resolve(), out.resolve()
    integrity = build_integrity(base, {})
    if not integrity['pass_integrity']:
        raise ValueError(str(integrity['errors']))
    manifest = json.loads((base/'build_manifest.json').read_text())
    if not manifest.get('coupling_objective'):
        raise ValueError('Requires the frozen objective-coupling build')
    out.mkdir(parents=True, exist_ok=False)
    shutil.copytree(base/'source', out/'source')
    shutil.copytree(base/'build', out/'build')
    for name in ('delivery_diagnostics.hpp', 'target_entry_diagnostics.hpp',
                 'coupling_bridge.hpp', 'coupling_native_integration.hpp'):
        shutil.copy2(HERE/name, out/'source'/name)
    source = out/'source/planner_manager.cpp'
    text = '#include "target_entry_diagnostics.hpp"\n' + source.read_text()
    marker = '      const bool final_target_occupied ='
    text = replace(text, marker, '''      const auto diagnostic_raw_before = grid_map_->getDynamicObservationSeq();
      const auto diagnostic_esdf_before = grid_map_->getESDFObservationSeq();
''' + marker)
    marker = '''                 final_coll_type);
        return false;'''
    text = replace(text, marker, '''                 final_coll_type);
        target_entry_diagnostics::rejected(*mm_config_,*grid_map_,
            Eigen::Vector3d(global_end_pt(0),global_end_pt(1),global_end_yaw),
            planning_horizen,search_start-global_start,project_from_current,final_coll_type,
            diagnostic_raw_before,diagnostic_esdf_before);
        return false;''')
    source.write_text(text)
    parent = Path(manifest['parent_build'])
    _, opt = compile_source(parent, out, 'exp_opt', out/'source/poly_traj_optimizer.cpp', '', out/'source')
    subprocess.run(['ar', 'r', str(out/'build/libexp_opt.a'), opt], check=True)
    link = shlex.split((parent/'build/CMakeFiles/remani_planner_node.dir/link.txt').read_text())
    for name in ('remani_planner_node.cpp', 'remani_replan_fsm.cpp', 'planner_manager.cpp'):
        old = next(x for x in link if x.endswith('/'+name+'.o'))
        if name == 'remani_planner_node.cpp':
            obj = out/'build'/(name+'.o')
            if not obj.exists():
                _, obj = compile_source(parent, out, 'remani_planner_node', out/'source'/name, old, out/'source')
        else:
            _, obj = compile_source(parent, out, 'remani_planner_node', out/'source'/name, old, out/'source')
        link = [str(obj) if x == old else x for x in link]
    search = Path(manifest['coupling_parent'])/'build/libexp_search.a'
    link = [str(out/'build/libexp_opt.a') if x == 'libexp_opt.a' else
            str(search) if x == 'libexp_search.a' else x for x in link]
    link[link.index('-o')+1] = str(out/'build/remani_planner_node')
    subprocess.run(link, cwd=str(parent/'build'), check=True)
    manifest.update(delivery_diagnostics=True, diagnostic_parent=str(base),
        diagnostic_parent_manifest_sha256=sha(base/'build_manifest.json'),
        algorithm_parameters_unchanged=True, diagnostic_scope='read-only candidate and failed-sample capture; timing not a performance comparison',
        isolated_source_sha256={str(p.relative_to(out/'source')): sha(p) for p in (out/'source').rglob('*') if p.is_file()},
        native_binaries_sha256={n: sha(out/'build'/n) for n in ('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')},
        diagnostic_builder_sha256=sha(Path(__file__)))
    (out/'build_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    if not build_integrity(out, {})['pass_integrity']:
        raise RuntimeError('New diagnostic build failed integrity')
    print('BUILT', out, flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base', type=Path, default=HERE/'builds/coupled_objective_20261022_04')
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    build(a.base, a.out)
