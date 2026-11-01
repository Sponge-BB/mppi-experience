#!/usr/bin/env python3
"""Independent V12 cusp ablation builds, never edits baseline/production."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess

from build_shared_local_safety import checked_hash, compile_recipe, sha
from build_native_v10 import compile_source
import mppi_cusp_patch as patch

HERE=Path(__file__).resolve().parent
NATIVE=('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')


def build(base,out,variant):
    base,out=Path(base).resolve(),Path(out).resolve()
    manifest=json.loads((base/'build_manifest.json').read_text())
    if variant not in ('phase','phase_objective','gated_phase','gated_objective'):raise ValueError('invalid ablation')
    if base!=HERE/'builds/native_v12_07':raise ValueError('this controlled patch requires frozen native_v12_07')
    for name,digest in manifest['isolated_source_sha256'].items():checked_hash(base/'source'/name,digest)
    checked_hash(base/'build/libonline_mppi.so',manifest['mppi_library_sha256'])
    modified={
        'online_kernel.hpp':patch.transform_kernel((base/'source/online_kernel.hpp').read_text()),
        'online_api.cpp':patch.transform_api((base/'source/online_api.cpp').read_text(),variant.startswith('gated_')),
        'mppi_v12.hpp':patch.transform_v12((base/'source/mppi_v12.hpp').read_text(),variant.endswith('objective')),
    }
    parent=Path(manifest['parent_build']).resolve();recipe=compile_recipe(parent,out)
    out.mkdir(parents=True,exist_ok=False);(out/'build').mkdir()
    shutil.copytree(base/'source',out/'source')
    for name,content in modified.items():(out/'source'/name).write_text(content)
    shutil.copy2(HERE/'mppi_cusp_cursor.hpp',out/'source/mppi_cusp_cursor.hpp')
    reused={}
    for name in NATIVE:
        expected=sha(base/'build'/name);shutil.copy2(base/'build'/name,out/'build'/name)
        checked_hash(out/'build'/name,expected);reused[name]=expected
    _,obj=compile_source(parent,out,'online_mppi',out/'source/online_api.cpp',recipe['old_object'],out/'source')
    link=[obj if x==recipe['old_object'] else x for x in recipe['link']]
    link[link.index('-o')+1]=str(out/'build/libonline_mppi.so')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    for name,digest in manifest['isolated_source_sha256'].items():checked_hash(base/'source'/name,digest)
    manifest.update(execution_parent=str(base),cusp_parent=str(base),cusp_variant=variant,
        controller_revision='v12_cusp_'+variant,only_local_mppi_recompiled=True,
        native_binaries_sha256=reused,mppi_library_sha256=sha(out/'build/libonline_mppi.so'),
        isolated_source_sha256={p.relative_to(out/'source').as_posix():sha(p) for p in (out/'source').rglob('*') if p.is_file()},
        cusp_builder_sha256=sha(Path(__file__)),cusp_patcher_sha256=sha(HERE/'mppi_cusp_patch.py'),
        cusp_common_helpers_sha256=sha(HERE/'directed_reference_patch.py'),
        cusp_changes=['measured-only phase cursor with bounded projection/connector ranges',
            'two actual samples at cusp with position/yaw/linear/angular/progress gates',
            'clear old warm/route association while retaining previous issued actuator command',
            'stage-goal proposal and merit' if variant.endswith('objective') else 'original full-goal merit retained; nonfinal goalCapture disabled',
            'local cusp association activates within 20cm remaining and distance, only for unoccupied full-body cusp; blocked cusps retain unbounded legacy detour association' if variant.startswith('gated_') else 'strict complete active-phase association'],
        hard_geometry_margin_limits_unchanged=True,publication_validator_unchanged=True,
        production_default=False,scope='Controlled ideal-observation numerical cusp experiment, not real-vehicle deployment')
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('CUSP_BUILD',variant,out,flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,default=HERE/'builds/native_v12_07')
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--variant',choices=('phase','phase_objective','gated_phase','gated_objective'),required=True)
    a=p.parse_args();build(a.base,a.out,a.variant)
