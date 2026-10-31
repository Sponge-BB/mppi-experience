#!/usr/bin/env python3
"""Re-attest an unchanged isolated candidate-B runtime after real-car deployment.

No compilation, ROS launch, production edit, or old-manifest rewrite occurs.
The production hashes are a before/after mutation audit, not the compiler
inputs of these frozen experiment binaries. Preserve that distinction here.
"""
import argparse
import copy
import json
from pathlib import Path
import shutil

from build_feasibility_stop import source_inventory
from build_shared_local_safety import NATIVE, checked_hash, sha
from build_urdf_spheres_b import CAPABILITIES, ARCHIVES, verify_recipe


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
BASE = HERE / 'builds/urdf_spheres_b_20261029_01'
DEFAULT_OUT = HERE / 'builds/urdf_sphere_timing_20261029_01'
DEPLOYED = ROOT / 'backups/real_urdf_spheres_b_20261029_01/deployed.json'


def validate_parent(base):
    """Verify frozen inputs directly, without the obsolete production baseline."""
    base = Path(base).resolve()
    manifest_path = base / 'build_manifest.json'
    if manifest_path.is_symlink():
        raise ValueError('Parent manifest must be a regular snapshot')
    manifest = json.loads(manifest_path.read_text())
    if (not manifest.get('urdf_spheres_b_experiment') or manifest.get('prepared_only') or
            not manifest.get('coupling_objective') or not manifest.get('dual_timescale_probe')):
        raise ValueError('A completed isolated candidate-B parent is required')
    sources = manifest.get('isolated_source_sha256', {})
    if not sources or source_inventory(base / 'source') != sources:
        raise ValueError('Parent source inventory changed')
    native = manifest.get('native_binaries_sha256', {})
    if set(native) != set(NATIVE):
        raise ValueError('Exactly three frozen native runtimes are required')
    for name, digest in native.items():
        checked_hash(base / 'build' / name, digest)
    checked_hash(base / 'build/libonline_mppi.so', manifest['mppi_library_sha256'])
    archives = manifest.get('urdf_spheres_b_archive_sha256', {})
    if set(archives) != set(ARCHIVES):
        raise ValueError('All three candidate-B native archives are required')
    for name, digest in archives.items():
        checked_hash(base / 'build' / name, digest)
    verify_recipe(manifest['urdf_spheres_b_recipe'])
    checked_hash(base / 'source/shared_collision.hpp', manifest['shared_collision_source_sha256'])
    return manifest


def production_snapshot(parent, root=ROOT, deployed=DEPLOYED):
    """Permit only the seven already recorded deployed-source replacements."""
    root, deployed = Path(root).resolve(), Path(deployed).resolve()
    deployment = json.loads(deployed.read_text())
    before = parent.get('production_sources_sha256', {})
    recorded = deployment.get('files_sha256', {})
    if not before or not recorded:
        raise ValueError('Production/deployment provenance is missing')
    allowed = {name for name in before if name in recorded and name.startswith('src/')}
    if len(allowed) != 7:
        raise ValueError('Expected exactly seven previously deployed source replacements')
    current, changed = dict(before), {}
    for name, digest in before.items():
        relative = Path(name)
        if relative.is_absolute() or '..' in relative.parts:
            raise ValueError('Unsafe production snapshot path')
        path = root / name
        actual = sha(path)
        if actual != digest:
            if name not in allowed or actual != recorded[name]:
                raise ValueError('Unrecorded production change: ' + name)
            changed[name] = dict(before=digest, deployed=actual)
            current[name] = actual
        elif name in allowed and actual != recorded[name]:
            raise ValueError('Recorded deployment is not the current source: ' + name)
    # The new sphere YAML was absent from the old 98-file snapshot, so audit it
    # separately instead of silently changing the old audit's file membership.
    deployment_sources = {}
    for name, digest in recorded.items():
        if name.startswith('src/'):
            checked_hash(root / name, digest)
            deployment_sources[name] = digest
    return current, dict(deployment_record=str(deployed), deployment_record_sha256=sha(deployed),
        allowed_source_replacements=sorted(allowed), applied_source_replacements=changed,
        deployment_sources_sha256=deployment_sources,
        parent_production_sources_sha256=copy.deepcopy(before),
        interpretation='Current production mutation-audit baseline only; NOT compiler inputs or a rebuild')


def prepare(out=DEFAULT_OUT, base=BASE, root=ROOT, deployed=DEPLOYED):
    """Return (new manifest, new path); copy only sources and audited products."""
    out, base = Path(out).resolve(), Path(base).resolve()
    if out == base or base in out.parents or out in base.parents:
        raise ValueError('New build must be independent of its immutable parent')
    if out.exists():
        raise FileExistsError('Refusing to overwrite existing experiment build: ' + str(out))
    parent = validate_parent(base)
    parent_sha = sha(base / 'build_manifest.json')
    current, audit = production_snapshot(parent, root, deployed)
    runtime = dict(parent['native_binaries_sha256'])
    runtime['libonline_mppi.so'] = parent['mppi_library_sha256']
    archives = dict(parent['urdf_spheres_b_archive_sha256'])
    out.mkdir(parents=True)
    shutil.copytree(base / 'source', out / 'source')
    (out / 'build').mkdir()
    for name, digest in {**runtime, **archives}.items():
        shutil.copy2(base / 'build' / name, out / 'build' / name)
        checked_hash(out / 'build' / name, digest)
    if source_inventory(out / 'source') != parent['isolated_source_sha256']:
        raise ValueError('Copied source inventory does not match frozen parent')
    checked_hash(base / 'build_manifest.json', parent_sha)
    # Recheck the deployment and all recorded production inputs after copying.
    current_after, audit_after = production_snapshot(parent, root, deployed)
    if current_after != current or audit_after != audit:
        raise ValueError('Production/deployment changed during preparation')
    manifest = {key: copy.deepcopy(parent[key]) for key in CAPABILITIES if key in parent}
    manifest.update(production_sources_sha256=current, prepared_only=False,
        urdf_spheres_b_experiment=True, urdf_sphere_timing_experiment=True,
        source_root=str(out / 'source'), parent_build=str(base),
        isolated_source_sha256=copy.deepcopy(parent['isolated_source_sha256']),
        native_binaries_sha256=copy.deepcopy(parent['native_binaries_sha256']),
        mppi_library_sha256=parent['mppi_library_sha256'],
        urdf_spheres_b_archive_sha256=archives,
        urdf_spheres_b_geometry=copy.deepcopy(parent['urdf_spheres_b_geometry']),
        sphere_timing_preparer_sha256=sha(Path(__file__)),
        sphere_timing_parent_manifest_sha256=parent_sha,
        sphere_timing_parent_source_sha256=copy.deepcopy(parent['isolated_source_sha256']),
        sphere_timing_parent_runtime_sha256=runtime,
        sphere_timing_parent_archives_sha256=archives,
        sphere_timing_parent_recipe=copy.deepcopy(parent['urdf_spheres_b_recipe']),
        sphere_timing_production_rebaseline=audit,
        binary_unchanged_from_parent=True, compiled_in_this_preparation=False,
        production_modified_by_this_preparation=False,
        sphere_timing_geometry_note='140/47 refer to the prior deployed Piper fixed-structure approximations; not original REMANI baseline counts. Arm remains 20 spheres.',
        scope='Frozen candidate-B numerical SIL runtime, copied without recompilation; production audit re-baselined only after documented real-car deployment. Not a hardware launch.')
    (out / 'build_manifest.json').write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    return manifest, out


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    manifest, out = prepare(args.out)
    from run_coupled_objective import check_build
    check_build(out)
    print('PREPARED_UNCHANGED_SPHERE_TIMING_RUNTIME', out)
