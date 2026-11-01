#!/usr/bin/env python3
"""MPPI-only isolated child; the frozen realtime parent remains untouched."""
import argparse
import copy
import json
from pathlib import Path
import shutil
import subprocess

import build_native_realtime as previous
from build_shared_local_safety import checked_hash, sha
from run_coupled_objective import check_build
from run_dual_timescale import verify_production

HERE = Path(__file__).resolve().parent
BASE = HERE / 'builds/native_realtime_20261027_01'
DEFAULT_OUT = HERE / 'builds/native_dynamic_fast_20261028_01'
PATCHED = ('mppi_v12.hpp',)
HEADER = 'native_dynamic_fast_probe.hpp'
BUILD_INPUTS = ('build_native_dynamic_fast.py', 'patch_native_dynamic_fast.py', HEADER)


def validate(base):
    base = Path(base).resolve()
    if base != BASE.resolve():
        raise ValueError('Requires frozen native_realtime_20261027_01 parent')
    manifest = previous.check(base)
    if manifest.get('prepared_only'):
        raise ValueError('Completed realtime parent required')
    return manifest


def recipe(parent, base, out):
    value = copy.deepcopy(parent['native_realtime_recipe'])
    item = value['mppi']
    for key in ('compile_command', 'actual_link_command'):
        item[key] = [arg.replace(str(base), str(out)) for arg in item[key]]
    item['expected_object'] = item['expected_object'].replace(str(base), str(out))
    previous.verify_recipe(value)
    return value


def transformed_sources(base):
    import patch_native_dynamic_fast as patch
    original = {name: (base / 'source' / name).read_text() for name in PATCHED}
    changed = patch.patch_sources(original)
    if set(changed) != set(PATCHED):
        raise ValueError('Unexpected dynamic source edit scope')
    for name, text in changed.items():
        if text == original[name] or patch.project_off(text, name) != original[name]:
            raise ValueError('Source transform not reversible: ' + name)
    return changed


def verify_sources(out, manifest, base):
    inventory = previous.previous.native_recipe.source_inventory(out / 'source')
    if inventory != manifest['isolated_source_sha256']:
        raise ValueError('Child source inventory drift')
    expected = dict(manifest['native_dynamic_fast_parent_source_sha256'])
    for name in (*PATCHED, HEADER):
        expected[name] = sha(out / 'source' / name)
    if inventory != expected:
        raise ValueError('Source edited outside declared dynamic MPPI files')
    for name, text in transformed_sources(base).items():
        if (out / 'source' / name).read_text() != text:
            raise ValueError('Deterministic source transform mismatch: ' + name)
    checked_hash(out / 'source' / HEADER, manifest['native_dynamic_fast_inputs_sha256'][HEADER])
    previous.prove_mppi_only_header_dependency(out)


def prepare(base, out):
    raw = Path(out)
    if raw.is_symlink():
        raise ValueError('Output may not be a symlink')
    base, out = Path(base).resolve(), raw.resolve()
    parent = validate(base)
    if out.exists() or out == base or base in out.parents or out in base.parents:
        raise ValueError('Independent new output required; refusing overwrite')
    changed = transformed_sources(base)
    if HEADER in parent['isolated_source_sha256']:
        raise ValueError('Unexpected pre-existing child header')
    inputs = {name: sha(HERE / name) for name in BUILD_INPUTS}
    compiler = recipe(parent, base, out)
    out.mkdir(parents=True, exist_ok=False)
    (out / 'build').mkdir()
    shutil.copytree(base / 'source', out / 'source')
    for name, text in changed.items():
        (out / 'source' / name).write_text(text)
    shutil.copy2(HERE / HEADER, out / 'source' / HEADER)
    for name in previous.PRODUCTS:
        shutil.copy2(base / 'build' / name, out / 'build' / name)
    manifest = copy.deepcopy(parent)
    manifest.update(prepared_only=True, native_dynamic_fast_experiment=True,
        native_dynamic_fast_prepared_only=True, native_dynamic_fast_parent=str(base),
        native_dynamic_fast_parent_manifest_sha256=sha(base / 'build_manifest.json'),
        native_dynamic_fast_parent_source_sha256=parent['isolated_source_sha256'],
        native_dynamic_fast_parent_mppi_sha256=parent['mppi_library_sha256'],
        native_dynamic_fast_inputs_sha256=inputs, native_dynamic_fast_recipe=compiler,
        native_dynamic_fast_reused_binary_sha256=previous.reused_hashes(parent),
        native_dynamic_fast_switch='NATIVE_DYNAMIC_FAST',
        native_dynamic_fast_switch_default=False,
        native_dynamic_fast_only_mppi_recompiled=True,
        native_dynamic_fast_scope='native3D dynamic pruning/query reuse only; no search/geometry/execution policy change',
        production_modified=False, production_default=False,
        isolated_source_sha256=previous.previous.native_recipe.source_inventory(out / 'source'))
    verify_sources(out, manifest, base)
    (out / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    check(out)
    print('PREPARED NATIVE DYNAMIC FAST', out, flush=True)
    return out


def check(out):
    out = Path(out).resolve()
    path = out / 'build_manifest.json'
    if path.is_symlink() or not path.is_file():
        raise ValueError('Missing regular child manifest')
    manifest = json.loads(path.read_text())
    if not manifest.get('native_dynamic_fast_experiment'):
        raise ValueError('Not a dynamic-fast isolated build')
    base = Path(manifest['native_dynamic_fast_parent']).resolve()
    parent = validate(base)
    checked_hash(base / 'build_manifest.json', manifest['native_dynamic_fast_parent_manifest_sha256'])
    if manifest['native_dynamic_fast_parent_source_sha256'] != parent['isolated_source_sha256']:
        raise ValueError('Parent source provenance mismatch')
    if manifest['native_dynamic_fast_parent_mppi_sha256'] != parent['mppi_library_sha256']:
        raise ValueError('Parent MPPI provenance mismatch')
    inputs = manifest['native_dynamic_fast_inputs_sha256']
    if set(inputs) != set(BUILD_INPUTS):
        raise ValueError('Incomplete build input inventory')
    for name, digest in inputs.items():
        checked_hash(HERE / name, digest)
    if manifest['native_dynamic_fast_recipe'] != recipe(parent, base, out):
        raise ValueError('Frozen compiler recipe differs')
    verify_sources(out, manifest, base)
    reused = previous.reused_hashes(parent)
    if manifest['native_dynamic_fast_reused_binary_sha256'] != reused:
        raise ValueError('Native product reuse mismatch')
    for name, digest in reused.items():
        checked_hash(out / 'build' / name, digest)
    for key in ('native_binaries_sha256', 'native_collision_3d_archive_sha256'):
        if manifest[key] != parent[key]:
            raise ValueError('Native product metadata differs')
    if manifest['prepared_only'] != manifest['native_dynamic_fast_prepared_only']:
        raise ValueError('Prepared flags differ')
    expected = parent['mppi_library_sha256'] if manifest['prepared_only'] else manifest['mppi_library_sha256']
    checked_hash(out / 'build/libonline_mppi.so', expected)
    verify_production(out)
    if not manifest['prepared_only']:
        check_build(out)
    return manifest


def compile_prepared(out):
    out = Path(out).resolve()
    manifest = check(out)
    if not manifest['prepared_only']:
        raise ValueError('Fresh prepared child required')
    item = manifest['native_dynamic_fast_recipe']['mppi']
    subprocess.run(item['compile_command'], check=True)
    subprocess.run(item['actual_link_command'], cwd=item['cwd'], check=True)
    validate(Path(manifest['native_dynamic_fast_parent']))
    verify_sources(out, manifest, Path(manifest['native_dynamic_fast_parent']))
    previous.verify_recipe(manifest['native_dynamic_fast_recipe'])
    digest = sha(out / 'build/libonline_mppi.so')
    if digest == manifest['native_dynamic_fast_parent_mppi_sha256']:
        raise ValueError('Rebuilt MPPI unexpectedly unchanged')
    manifest.update(prepared_only=False, native_dynamic_fast_prepared_only=False, mppi_library_sha256=digest)
    (out / 'build_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    check(out)
    print('BUILT NATIVE DYNAMIC FAST', out, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, default=BASE)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--compile', action='store_true')
    action.add_argument('--check', action='store_true')
    args = parser.parse_args()
    if args.check:
        check(args.out)
    elif args.compile:
        compile_prepared(args.out)
    else:
        prepare(args.base, args.out)


if __name__ == '__main__':
    main()
