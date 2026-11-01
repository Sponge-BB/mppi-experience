#!/usr/bin/env python3
"""Experimental directed-phase wrapper around immutable V12 shared build09.

Only the local MPPI library is rebuilt. The complete native reference is kept;
this is not a Python reference slice, a production default or a native rebuild.
"""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
from build_native_v10 import compile_source
from build_shared_local_safety import HERE,NATIVE,sha,checked_hash,validate_base,compile_recipe

PATCHED_SOURCES={
    'online_kernel.hpp':'transform_kernel',
    'online_api.cpp':'transform_api',
    'mppi_v12.hpp':'transform_v12',
}
HEADER='directed_reference.hpp'
PATCHER='directed_reference_patch.py'
CHANGES=[
    'Bind the complete native reference to a measured-state directed-phase cursor; control association and connectors use the active gear/segment without Python slicing or hiding the complete reference provenance',
    'Intermediate directed endpoints are cusp transitions rather than task completion; final goalCapture eligibility is restricted to the final phase while the full-bundle final endpoint remains available',
    'Preserve the native planner/controller/detector and shared collision implementation byte-for-byte; local selected proposals remain subject to latest-frame publication rollout and braking validation',
]


def load_patcher(path,expected):
    """Reject a moving patch module rather than record the wrong provenance."""
    checked_hash(path,expected)
    spec=importlib.util.spec_from_file_location('_directed_reference_build_patch',path)
    if spec is None or spec.loader is None:raise ValueError('cannot load directed-reference patch interfaces')
    module=importlib.util.module_from_spec(spec)
    # Standard module registration also supports future annotations/dataclass
    # helpers in a patch module without altering any operational Python file.
    previous=sys.modules.get(spec.name);sys.modules[spec.name]=module
    try:spec.loader.exec_module(module)
    except Exception:
        if previous is None:sys.modules.pop(spec.name,None)
        else:sys.modules[spec.name]=previous
        raise
    checked_hash(path,expected)
    for name in PATCHED_SOURCES.values():
        if not callable(getattr(module,name,None)):raise ValueError('missing directed-reference patch function: '+name)
    return module


def build(base,out):
    base=Path(base).resolve();out=Path(out).resolve();manifest_path=base/'build_manifest.json'
    original_manifest_hash=sha(manifest_path);manifest=json.loads(manifest_path.read_text())
    verified=validate_base(base,manifest)
    if not manifest.get('only_local_mppi_recompiled') or not manifest.get('rejoin_priority_changes'):
        raise ValueError('base must be the completed shared checked-native-rejoin priority build')
    if manifest.get('directed_reference') or HEADER in verified['source_sha256']:
        raise ValueError('directed-reference wrapper requires an unpatched immutable base')
    parent=Path(manifest['parent_build']).resolve()
    if out==base or base in out.parents or out==parent or parent in out.parents:
        raise ValueError('output must be independent of the immutable base and compiler parent')
    if out.exists():raise FileExistsError('refusing to overwrite existing build: '+str(out))
    header_path=HERE/HEADER;patcher_path=HERE/PATCHER
    header_hash=sha(header_path);patcher_hash=sha(patcher_path)
    checked_hash(header_path,header_hash);patcher=load_patcher(patcher_path,patcher_hash)
    transformed={}
    for name,method in PATCHED_SOURCES.items():
        original=(base/'source'/name).read_text()
        modified=getattr(patcher,method)(original)
        if not isinstance(modified,str) or modified==original:
            raise ValueError('directed-reference patch must return changed source text: '+method)
        transformed[name]=modified
    recipe=compile_recipe(parent,out)
    out.mkdir(parents=True,exist_ok=False);(out/'build').mkdir()
    shutil.copytree(base/'source',out/'source')
    shutil.copy2(header_path,out/'source'/HEADER)
    checked_hash(out/'source'/HEADER,header_hash)
    for name,text in transformed.items():(out/'source'/name).write_text(text)
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
    if sha(manifest_path)!=original_manifest_hash:raise ValueError('base manifest changed during directed build')
    validate_base(base,manifest)
    checked_hash(header_path,header_hash);checked_hash(patcher_path,patcher_hash)
    for filename,expected in recipe['parent_link_archives_sha256'].items():checked_hash(Path(filename),expected)
    checked_hash(Path(recipe['parent_flags_path']),recipe['flags_sha256'])
    checked_hash(Path(recipe['parent_link_path']),recipe['link_recipe_sha256'])
    source_hashes={p.relative_to(out/'source').as_posix():sha(p) for p in (out/'source').rglob('*') if p.is_file()}
    if set(source_hashes)!=set(verified['source_sha256'])|{HEADER}:
        raise ValueError('directed build may add only its declared header to the source inventory')
    changed={name for name in verified['source_sha256'] if source_hashes[name]!=verified['source_sha256'][name]}
    if changed!=set(PATCHED_SOURCES):raise ValueError('directed build changed sources outside its declared three-file scope')
    checked_hash(out/'source/shared_collision.hpp',manifest['shared_collision_source_sha256'])
    mppi_hash=sha(out/'build/libonline_mppi.so')
    if mppi_hash==verified['mppi_sha256']:raise ValueError('directed library unexpectedly equals its unpatched base')
    inherited_changes=list(manifest.get('local_safety_changes',[]))
    manifest.update(controller_revision=manifest['controller_revision']+'_directed_reference_phase',
        execution_parent=str(base),local_safety_parent=str(base),directed_reference_parent=str(base),
        local_safety_changes=CHANGES,directed_reference_changes=CHANGES,
        inherited_local_safety_changes=inherited_changes,
        controller_changes=list(manifest.get('controller_changes',[]))+CHANGES,
        local_safety_base_manifest_sha256=original_manifest_hash,
        local_safety_base_source_sha256=verified['source_sha256'],
        local_safety_base_mppi_library_sha256=verified['mppi_sha256'],
        native_reuse_origin=str(base),native_reuse_evidence=reuse,
        native_binaries_sha256={name:checked_hash(out/'build'/name,verified['native_sha256'][name]) for name in NATIVE},
        mppi_library_sha256=mppi_hash,isolated_source_sha256=source_hashes,
        builder_sha256=sha(Path(__file__)),only_local_mppi_recompiled=True,
        directed_reference=True,directed_reference_source_sha256=header_hash,
        directed_reference_patch_sha256=patcher_hash,
        directed_reference_patch_interfaces=PATCHED_SOURCES,
        full_native_bundle_terminal_retained=True,
        directed_reference_debug_api=dict(name='online_directed_cursor_debug',
            signature='void(void*, double*); caller provides 12 doubles',
            fields=['ready','phase','phase_count','active_begin','active_end','active_gear',
                'current_cusp_distance','transitions','event_1start_2switch_0none','previous_phase',
                'reference_fnv_hi32','reference_fnv_lo32'],
            identity_scope='FNV64 reference-byte telemetry, NOT cryptographic SHA256 provenance'),
        directed_cursor_scope='Measured-state phase association; immutable per-plan active range; complete native bundle/future and final bundle endpoint retained; no Python reference slicing or actor-truth input.',
        directed_goal_semantics='Final goalCapture mode eligibility changes: nonfinal directed endpoints are cusps, not task goals. Endpoint source remains the full native bundle, not an independently supplied Python task-goal context.',
        directed_reference_production_default=False,
        directed_reference_caveat='Experimental directed-phase-only wrapper over immutable shared build09; production and operational Python are untouched. This build manifest is not an execution, collision or real-vehicle safety certificate.',
        non_goal_policy_unchanged=False,non_goal_policy_changes=CHANGES,
        hard_geometry_margin_limits_goal_unchanged=False,hard_geometry_margin_limits_unchanged=True,
        local_safety_patch_scope='Directed association, phase/cusp sequencing, native/local route context and final-phase goal eligibility change. Native binaries, shared collision model, inherited parameters and hard publication rollout/braking validation remain unchanged.',
        local_safety_preserved_source_sha256={name:digest for name,digest in source_hashes.items() if name not in PATCHED_SOURCES and name!=HEADER},
        local_safety_compile_recipe={**recipe,'actual_link_command':link,
            'helper_source_sha256':sha(HERE/'build_native_v10.py'),
            'validation_helper_source_sha256':sha(HERE/'build_shared_local_safety.py'),
            'directed_patch_source_sha256':patcher_hash})
    metadata=getattr(patcher,'MANIFEST_METADATA',None)
    if metadata is not None:
        if not isinstance(metadata,dict):raise ValueError('directed patch MANIFEST_METADATA must be a dictionary')
        # Keep patch-supplied feature details separate so it cannot override
        # verified parent/binary/collision/provenance fields.
        manifest['directed_reference_patch_metadata']=metadata
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('BUILT DIRECTED REFERENCE PHASE',out,flush=True)
    return out


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,default=HERE/'builds/v12_shared_09')
    parser.add_argument('--out',type=Path,required=True,help='fresh independent output, normally builds/v12_shared_10')
    args=parser.parse_args();build(args.base,args.out)
