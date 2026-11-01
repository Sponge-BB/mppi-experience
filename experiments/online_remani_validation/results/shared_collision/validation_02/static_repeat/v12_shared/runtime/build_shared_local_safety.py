#!/usr/bin/env python3
"""Immutable V12 shared state-safety patch; rebuild only the MPPI C API library.

The native planner/controller/detector are reused only after complete manifest
checks and byte-identical copying. No production file or previous build is
edited, and no ROS process is started by this builder.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess
from build_native_v10 import compile_source

HERE=Path(__file__).resolve().parent
NATIVE=('remani_planner_node','mm_controller_node','dynamic_obstacle_tracker_node')
PATCH_FILES=('online_kernel.hpp','online_api.cpp')
CHANGES=[
    'BaseMPPI.feedback returns immediately when the control warm start is empty; no warm[0] access after a reference reset or early goal return',
    'online_reference_changed clears inherited LocalGuideMPPI.side so a retired static bypass does not bias the new reference through switch_cost',
]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def checked_hash(path,expected):
    if not isinstance(expected,str) or not re.fullmatch(r'[0-9a-f]{64}',expected):
        raise ValueError('invalid SHA256 in base manifest: '+str(path))
    if not path.is_file() or path.is_symlink():
        raise ValueError('missing or symlinked immutable input: '+str(path))
    observed=sha(path)
    if observed!=expected:raise ValueError('base manifest SHA256 mismatch: '+str(path))
    return observed


def validate_base(base,manifest):
    if not manifest.get('shared_collision') or not manifest.get('v12_controller'):
        raise ValueError('base must be a completed V12 shared-collision build')
    sources=manifest.get('isolated_source_sha256',{})
    if not sources or not all(name in sources for name in (*PATCH_FILES,'mppi_v12.hpp','shared_collision.hpp')):
        raise ValueError('base manifest lacks the complete isolated source inventory')
    for name,expected in sources.items():
        relative=Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('unsafe source path in base manifest: '+name)
        checked_hash(base/'source'/relative,expected)
    inventory=set()
    for path in (base/'source').rglob('*'):
        if path.is_symlink():raise ValueError('symlink in base source inventory: '+str(path))
        if path.is_file():inventory.add(path.relative_to(base/'source').as_posix())
    if inventory!=set(sources):raise ValueError('base source inventory differs from its manifest')
    native=manifest.get('native_binaries_sha256',{})
    if set(native)!=set(NATIVE):raise ValueError('base manifest must identify exactly the three reused native binaries')
    for name in NATIVE:checked_hash(base/'build'/name,native[name])
    checked_hash(base/'build/libonline_mppi.so',manifest['mppi_library_sha256'])
    checked_hash(base/'source/shared_collision.hpp',manifest['shared_collision_source_sha256'])
    return dict(source_sha256=dict(sources),native_sha256=dict(native),
                mppi_sha256=manifest['mppi_library_sha256'])


def replace_once(text,old,new,label):
    count=text.count(old)
    if count!=1:raise ValueError('expected exactly one '+label+' marker, found '+str(count))
    return text.replace(old,new,1)


def transform(kernel,api):
    feedback='  void feedback(V actual){double fraction=c.dt/c.rdt;for(size_t k=0;k+1<warm.size();++k)warm[k]=(1-fraction)*warm[k]+fraction*warm[k+1];warm[0]=.5*warm[0]+.5*actual;}'
    guarded=feedback.replace('{double fraction','{if(warm.empty())return;double fraction',1)
    reference='''extern "C" void online_reference_changed(void*p){
  if(p){auto&m=static_cast<Online*>(p)->m;m.route.path.clear();m.published_route.clear();m.recovering=false;m.recovery_clear=0;m.committed_side=0;m.clear_count=0;m.referenceChanged();}
}'''
    cleared=reference.replace('m.route.path.clear();','m.side=0;m.route.path.clear();',1)
    changed_kernel=replace_once(kernel,feedback,guarded,'BaseMPPI feedback')
    changed_api=replace_once(api,reference,cleared,'online_reference_changed')
    # Inverse substitution proves no unrelated formula or control policy was
    # modified inside these two otherwise byte-identical translation units.
    if replace_once(changed_kernel,guarded,feedback,'feedback inverse')!=kernel:
        raise ValueError('unexpected non-feedback kernel change')
    if replace_once(changed_api,cleared,reference,'reference-reset inverse')!=api:
        raise ValueError('unexpected non-reset C API change')
    return changed_kernel,changed_api


def compile_recipe(parent,out):
    target=parent/'build/CMakeFiles/online_mppi.dir'
    flags_path=target/'flags.make';link_path=target/'link.txt'
    flags=flags_path.read_text()
    include_rows=[line for line in flags.splitlines() if line.startswith('CXX_INCLUDES =')]
    flag_rows=[line for line in flags.splitlines() if line.startswith('CXX_FLAGS =')]
    if len(include_rows)!=1 or len(flag_rows)!=1:raise ValueError('ambiguous parent MPPI compiler flags')
    # compile_source uses the already established -O2 C++17 configuration.
    # Do not silently change optimization relative to this controlled patch.
    parent_flags=shlex.split(flag_rows[0].split('=',1)[1])
    if '-O2' not in parent_flags or any(x.startswith('-O') and x!='-O2' for x in parent_flags) or '-ffast-math' in parent_flags:
        raise ValueError('parent MPPI flags no longer match the inherited -O2/no-fast-math profile')
    link=shlex.split(link_path.read_text())
    objects=[arg for arg in link if arg.endswith('/online_api.cpp.o') or arg=='online_api.cpp.o']
    if len(objects)!=1 or link.count('-o')!=1 or link.index('-o')+1>=len(link):
        raise ValueError('ambiguous parent MPPI link recipe')
    old=objects[0];source=out/'source/online_api.cpp';obj=out/'build/online_api.cpp.o'
    expected_compile=['/usr/bin/c++','-std=gnu++17','-O2','-DNDEBUG','-fPIC',
        '-I'+str(out/'source'),*shlex.split(include_rows[0].split('=',1)[1]),
        '-c',str(source),'-o',str(obj)]
    archives={}
    for arg in link:
        if arg.endswith('.a'):
            archive=Path(arg)
            if not archive.is_absolute():archive=parent/'build'/archive
            if not archive.is_file():raise ValueError('missing parent link archive: '+str(archive))
            archives[str(archive.resolve())]=sha(archive)
    return dict(link=link,old_object=old,expected_object=str(obj),compile_command=expected_compile,
        flags_sha256=sha(flags_path),link_recipe_sha256=sha(link_path),
        parent_flags_path=str(flags_path),parent_link_path=str(link_path),
        parent_link_archives_sha256=archives)


def build(base,out):
    base=Path(base).resolve();out=Path(out).resolve();manifest_path=base/'build_manifest.json'
    original_manifest_hash=sha(manifest_path);manifest=json.loads(manifest_path.read_text())
    verified=validate_base(base,manifest)
    parent=Path(manifest['parent_build']).resolve()
    if out==base or base in out.parents or out==parent or parent in out.parents:
        raise ValueError('output must be independent of the immutable base and parent build')
    if out.exists():raise FileExistsError('refusing to overwrite existing build: '+str(out))
    original_kernel=(base/'source/online_kernel.hpp').read_text()
    original_api=(base/'source/online_api.cpp').read_text()
    kernel,api=transform(original_kernel,original_api)
    recipe=compile_recipe(parent,out)
    out.mkdir(parents=True,exist_ok=False);(out/'build').mkdir()
    shutil.copytree(base/'source',out/'source')
    (out/'source/online_kernel.hpp').write_text(kernel)
    (out/'source/online_api.cpp').write_text(api)
    reuse={}
    for name in NATIVE:
        source=base/'build'/name;destination=out/'build'/name
        checked_hash(source,verified['native_sha256'][name]);shutil.copy2(source,destination)
        copied=checked_hash(destination,verified['native_sha256'][name])
        reuse[name]=dict(origin=str(source),destination=str(destination),
            base_manifest_sha256=verified['native_sha256'][name],destination_sha256=copied,unchanged=True)
    _,obj=compile_source(parent,out,'online_mppi',out/'source/online_api.cpp',recipe['old_object'],out/'source')
    if obj!=recipe['expected_object']:raise RuntimeError('compiler helper changed its output object naming')
    link=[obj if arg==recipe['old_object'] else arg for arg in recipe['link']]
    link[link.index('-o')+1]=str(out/'build/libonline_mppi.so')
    subprocess.run(link,cwd=str(parent/'build'),check=True)
    # Refuse to publish a completed manifest if a supposedly frozen dependency
    # changed while copying/compiling. Leave incomplete output for diagnosis.
    if sha(manifest_path)!=original_manifest_hash:raise ValueError('base manifest changed during the build')
    validate_base(base,manifest)
    for filename,expected in recipe['parent_link_archives_sha256'].items():checked_hash(Path(filename),expected)
    checked_hash(Path(recipe['parent_flags_path']),recipe['flags_sha256'])
    checked_hash(Path(recipe['parent_link_path']),recipe['link_recipe_sha256'])
    source_hashes={p.relative_to(out/'source').as_posix():sha(p) for p in (out/'source').rglob('*') if p.is_file()}
    if set(source_hashes)!=set(verified['source_sha256']):
        raise ValueError('safety patch changed the complete source inventory')
    changed={name for name in source_hashes if source_hashes[name]!=verified['source_sha256'][name]}
    if changed!=set(PATCH_FILES):
        raise ValueError('safety patch changed a source outside the exact two-file scope')
    mppi_hash=sha(out/'build/libonline_mppi.so')
    if mppi_hash==verified['mppi_sha256']:raise ValueError('new MPPI library unexpectedly equals the unpatched library')
    manifest.update(controller_revision=manifest['controller_revision']+'_local_state_safety',
        execution_parent=str(base),local_safety_parent=str(base),local_safety_changes=CHANGES,
        controller_changes=list(manifest.get('controller_changes',[]))+CHANGES,
        local_safety_base_manifest_sha256=original_manifest_hash,
        local_safety_base_source_sha256=verified['source_sha256'],
        local_safety_base_mppi_library_sha256=verified['mppi_sha256'],
        native_reuse_origin=str(base),native_reuse_evidence=reuse,
        native_binaries_sha256={name:checked_hash(out/'build'/name,verified['native_sha256'][name]) for name in NATIVE},
        mppi_library_sha256=mppi_hash,isolated_source_sha256=source_hashes,
        builder_sha256=sha(Path(__file__)),only_local_mppi_recompiled=True,
        non_goal_policy_unchanged=True,
        local_safety_patch_scope='Only empty-warm feedback guard and retired static-side reset; no objective, weight, geometry, margin, speed, horizon, goal or non-goal stopping policy changes.',
        local_safety_preserved_source_sha256={name:digest for name,digest in source_hashes.items() if name not in PATCH_FILES},
        local_safety_compile_recipe={**recipe,'actual_link_command':link,
            'helper_source_sha256':sha(HERE/'build_native_v10.py')})
    (out/'build_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print('BUILT LOCAL STATE SAFETY',out,flush=True)
    return out


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,default=HERE/'builds/v12_shared_06')
    parser.add_argument('--out',type=Path,required=True,help='fresh independent output, normally builds/v12_shared_07')
    args=parser.parse_args();build(args.base,args.out)
