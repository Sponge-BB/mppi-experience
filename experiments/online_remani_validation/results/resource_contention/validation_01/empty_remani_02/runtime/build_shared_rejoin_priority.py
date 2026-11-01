#!/usr/bin/env python3
"""Independent shared static checked-native-rejoin priority build (MPPI only).

This changes proposal-selection priority, not execution authorization. Native
binaries and every hard collision/publication/braking check remain unchanged.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
from build_native_v10 import compile_source
from build_shared_local_safety import (
    HERE,NATIVE,sha,checked_hash,validate_base,compile_recipe,replace_once,
)

CHANGES=[
    'Legacy recovery wrapper preserves V7 checked-native static REJOIN ahead of its alternative-recovery arbitration; re-evaluate the same native reference with the current shared collision model, retain nominal feasibility/common comfort, and preserve publication-time latest-frame rollout/braking checks',
]
MARKER='    bool failed=avoidance_debug[7]>0;'
INJECTION=MARKER+'''
    // A V7 checked native REJOIN already has nominal feasibility and the
    // four-cycle common-comfort confirmation. Do not replace it with a
    // connector-policy recovery primitive from an earlier reference.
    const auto native_rejoin_model=shared_collision::snapshot();
    if(shared_collision::enabled()&&native_rejoin_model&&native_rejoin_model->healthy&&
        z.os.empty()&&!failed&&decision_debug[0]==3.&&decision_debug[6]==0.&&!warm.empty()){
      auto native_rejoin=evaluate(s,z,sc,arcs(sc),warm);
      if(native_rejoin.feasible&&native_rejoin.min_gap>native_rejoin_model->comfort-c.margin){
        recovering=false;recovery_clear=0;return original;
      }
    }'''


def transform(kernel):
    changed=replace_once(kernel,MARKER,INJECTION,'Legacy checked-native-rejoin priority')
    if replace_once(changed,INJECTION,MARKER,'checked-native-rejoin inverse')!=kernel:
        raise ValueError('unexpected kernel change outside declared rejoin-priority insertion')
    return changed


def build(base,out):
    base=Path(base).resolve();out=Path(out).resolve();manifest_path=base/'build_manifest.json'
    original_manifest_hash=sha(manifest_path);manifest=json.loads(manifest_path.read_text())
    verified=validate_base(base,manifest)
    if not manifest.get('only_local_mppi_recompiled') or not manifest.get('local_safety_parent'):
        raise ValueError('base must include the completed shared local-state/rejoin safety rebuild')
    parent=Path(manifest['parent_build']).resolve()
    if out==base or base in out.parents or out==parent or parent in out.parents:
        raise ValueError('output must be independent of immutable base and parent build')
    if out.exists():raise FileExistsError('refusing to overwrite existing build: '+str(out))
    kernel=transform((base/'source/online_kernel.hpp').read_text())
    recipe=compile_recipe(parent,out)
    out.mkdir(parents=True,exist_ok=False);(out/'build').mkdir()
    shutil.copytree(base/'source',out/'source')
    (out/'source/online_kernel.hpp').write_text(kernel)
    reuse={}
    for name in NATIVE:
        source=base/'build'/name;destination=out/'build'/name
        checked_hash(source,verified['native_sha256'][name]);shutil.copy2(source,destination)
        copied=checked_hash(destination,verified['native_sha256'][name])
        reuse[name]=dict(origin=str(source),destination=str(destination),
            base_manifest_sha256=verified['native_sha256'][name],destination_sha256=copied,unchanged=True)
    _,obj=compile_source(parent,out,'online_mppi',out/'source/online_api.cpp',recipe['old_object'],out/'source')
    if obj!=recipe['expected_object']:raise RuntimeError('compiler helper changed output object naming')
    link=[obj if arg==recipe['old_object'] else arg for arg in recipe['link']]
    link[link.index('-o')+1]=str(out/'build/libonline_mppi.so')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    if sha(manifest_path)!=original_manifest_hash:raise ValueError('base manifest changed during build')
    validate_base(base,manifest)
    for filename,expected in recipe['parent_link_archives_sha256'].items():checked_hash(Path(filename),expected)
    checked_hash(Path(recipe['parent_flags_path']),recipe['flags_sha256'])
    checked_hash(Path(recipe['parent_link_path']),recipe['link_recipe_sha256'])
    source_hashes={p.relative_to(out/'source').as_posix():sha(p) for p in (out/'source').rglob('*') if p.is_file()}
    if set(source_hashes)!=set(verified['source_sha256']):raise ValueError('source inventory changed unexpectedly')
    changed={name for name,digest in source_hashes.items() if digest!=verified['source_sha256'][name]}
    if changed!={'online_kernel.hpp'}:raise ValueError('rejoin-priority patch changed more than its sole kernel insertion')
    mppi_hash=sha(out/'build/libonline_mppi.so')
    if mppi_hash==verified['mppi_sha256']:raise ValueError('new MPPI library equals unpatched base library')
    inherited_changes=list(manifest.get('local_safety_changes',[]))
    manifest.update(controller_revision=manifest['controller_revision']+'_checked_native_rejoin_priority',
        execution_parent=str(base),local_safety_parent=str(base),
        local_safety_changes=CHANGES,rejoin_priority_changes=CHANGES,
        inherited_local_safety_changes=inherited_changes,
        controller_changes=list(manifest.get('controller_changes',[]))+CHANGES,
        local_safety_base_manifest_sha256=original_manifest_hash,
        local_safety_base_source_sha256=verified['source_sha256'],
        local_safety_base_mppi_library_sha256=verified['mppi_sha256'],
        native_reuse_origin=str(base),native_reuse_evidence=reuse,
        native_binaries_sha256={name:checked_hash(out/'build'/name,verified['native_sha256'][name]) for name in NATIVE},
        mppi_library_sha256=mppi_hash,isolated_source_sha256=source_hashes,
        builder_sha256=sha(Path(__file__)),only_local_mppi_recompiled=True,
        non_goal_policy_unchanged=False,non_goal_policy_changes=CHANGES,
        hard_geometry_margin_limits_goal_unchanged=True,
        local_safety_patch_scope='Execution/proposal priority changes: preserve only shared healthy static V7 checked native REJOIN (mode3, no intervention, nonempty warm), independently re-evaluated feasible on the same native reference above common comfort. Other recovery thresholds, ordinary candidate arbitration, geometry, hard margin, limits, goal policy and publication-time latest-frame rollout/braking remain unchanged.',
        local_safety_preserved_source_sha256={name:digest for name,digest in source_hashes.items() if name!='online_kernel.hpp'},
        local_safety_compile_recipe={**recipe,'actual_link_command':link,
            'helper_source_sha256':sha(HERE/'build_native_v10.py'),
            'validation_helper_source_sha256':sha(HERE/'build_shared_local_safety.py')})
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('BUILT CHECKED NATIVE REJOIN PRIORITY',out,flush=True)
    return out


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,default=HERE/'builds/v12_shared_08')
    parser.add_argument('--out',type=Path,required=True,help='fresh independent output, normally builds/v12_shared_09')
    args=parser.parse_args();build(args.base,args.out)
