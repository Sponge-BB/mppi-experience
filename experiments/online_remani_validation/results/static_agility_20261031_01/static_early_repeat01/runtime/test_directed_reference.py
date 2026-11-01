#!/usr/bin/env python3
"""Independent cursor and real guide boundaries; no ROS runtime or control IO."""
import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import shlex
import shutil
import subprocess
from types import SimpleNamespace
import numpy as np

HERE = Path(__file__).resolve().parent
DP = ct.POINTER(ct.c_double)
CURSOR_FAILURES = {
    1: 'initial binding must remain phase0, including a measured cusp',
    2: 'far measurement changed cursor', 3: 'positive high speed crossed cusp',
    4: 'negative high speed crossed cusp', 5: 'outside position tolerance crossed cusp',
    6: 'candidate reads mutated cursor', 7: 'valid stopped cusp did not advance once',
    8: 'active phase indices not bounded', 9: 'same measurement leapt across next cusp',
    10: 'second valid cusp transition failed', 11: 'final phase advanced past end',
    12: 'same-gear fragments created a cusp', 13: 'leading zeros lost reverse gear',
    14: 'all-stationary reference mishandled', 15: 'new reference retained old phase',
    16: 'reset left cursor bound', 17: 'rebinding reset did not start phase0',
    18: 'cursor mutated/truncated complete reference',
}
CONTROLLER_FAILURES = {
    1: 'C API initialization failed', 2: 'new reference reset lost feedback or left cache',
    3: 'reset lost actuator feedback', 4: 'empty warm feedback guard failed',
    5: 'shared collision fixture binding failed', 6: 'native range lost full endpoint',
    7: 'projection/direction/goal crossed current phase',
    8: 'nonfinal cusp manufactured rotation from zero target vector',
    9: 'nonzero reference endpoint speed did not brake toward cusp',
    10: 'off-path cusp projection caused zero-speed reconnect deadlock',
    11: 'local connector copied native indices or lost global endpoint',
    12: 'local independent range/gear mapping failed', 13: 'local projection/goal crossed phase',
    14: 'connector joined a future native phase', 15: 'true final-goal capture was disabled',
    16: 'true final-yaw alignment lost', 17: 'actual C API ingress rejected fixture',
    18: 'forecast changed initially bound cursor', 19: 'hypothetical plan mutated cursor',
    20: 'high-speed actual ingress failed', 21: 'high-speed actual ingress changed cursor',
    22: 'valid cusp actual ingress failed', 23: 'valid cusp did not switch exactly once',
    24: 'phase switch lost actuator feedback', 25: 'reference reset left cursor/cache or lost feedback',
    26: 'reset left cursor or lost feedback',
}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def frozen_fixture(library, trial):
    """Boundary checks on saved native cusp geometry, not an online replay."""
    trial = Path(trial).resolve()
    status = json.loads((trial / 'run_status.json').read_text())
    if not status.get('result_written') or not (trial / 'result.json').is_file():
        raise ValueError('frozen fixture requires a completed trial')
    from native_reference import reference_polynomial, sampled_reference
    raw = json.loads((trial / 'trajectories.json').read_text())
    bundles = []; current = []
    for row in raw:
        if row['action'] != 1:
            continue
        if row['id'] == 1:
            if current:
                bundles.append(current)
            current = []
        current.append(row)
    if current:
        bundles.append(current)
    bundle = next((group for group in bundles if len(set(v['singul'] for v in group)) > 1), None)
    if bundle is None:
        raise ValueError('fixture has no actual native mixed-gear bundle')
    parts = []
    for row in bundle:
        msg = SimpleNamespace(singul=row['singul'], trajectory=[SimpleNamespace(**v) for v in row['polynomials']])
        parts.append(reference_polynomial(msg)[:, 1:5])
    reference = sampled_reference(parts, 0.)
    lib = ct.CDLL(str(library))
    lib.directed_cursor_create.restype = ct.c_void_p
    lib.directed_cursor_destroy.argtypes = [ct.c_void_p]
    lib.directed_cursor_update.argtypes = [ct.c_void_p, DP, ct.c_int, ct.c_double, ct.c_double, ct.c_double, DP]
    lib.directed_cursor_candidate_reads.argtypes = [ct.c_void_p, ct.c_int]
    ptr = lib.directed_cursor_create(); debug = np.zeros(12)
    def update(x, y, v):
        code = lib.directed_cursor_update(ptr, reference.ctypes.data_as(DP), len(reference), x, y, v, debug.ctypes.data_as(DP))
        if code:
            raise AssertionError('frozen cursor C API rejected native reference')
        return debug.copy()
    try:
        first = update(*reference[0, :2], reference[0, 3])
        assert first[1] == 0 and first[2] >= 2
        end = int(first[4]); cusp = reference[end, :2]
        assert update(*cusp, .026)[1] == 0  # Original native endpoint is .05, not stopped.
        assert update(*cusp, -.026)[1] == 0
        assert lib.directed_cursor_candidate_reads(ptr, 1000) == 0
        switched = update(*cusp, .024)
        assert switched[1] == 1 and switched[7] == 1
        return dict(passed=True, original_trial=str(trial), original_result=json.loads((trial / 'result.json').read_text())['reason'],
                    raw_trajectory_sha256=sha(trial / 'trajectories.json'), sampled_reference_sha256=sha_array(reference),
                    native_message_directions=[v['singul'] for v in bundle], sample_count=len(reference),
                    phase_count=int(first[2]), first_cusp=cusp.tolist(),
                    native_cusp_signed_speed=float(reference[end, 3]), global_endpoint=reference[-1, :3].tolist(),
                    scope='Saved native geometry with synthetic boundary measurements; not shadow rollout, online success, or safety validation.')
    finally:
        lib.directed_cursor_destroy(ptr)


def sha_array(array):
    return hashlib.sha256(np.asarray(array, dtype='<f8').tobytes()).hexdigest()


def test(build, out, fixture=None):
    build, out = Path(build).resolve(), Path(out).resolve()
    manifest = json.loads((build / 'build_manifest.json').read_text())
    source_names = ('directed_reference.hpp', 'online_kernel.hpp', 'online_api.cpp', 'mppi_v12.hpp', 'shared_collision.hpp')
    hashes = {name: sha(build / 'source' / name) for name in source_names}
    for name, value in hashes.items():
        if manifest.get('isolated_source_sha256', {}).get(name) != value:
            raise ValueError('completed directed source differs from manifest: ' + name)
    if sha(build / 'build/libonline_mppi.so') != manifest['mppi_library_sha256']:
        raise ValueError('completed library hash differs from manifest')
    out.mkdir(parents=True, exist_ok=False)
    probe = out / 'directed_reference_probe.cpp'
    shutil.copy2(HERE / probe.name, probe)
    parent = Path(manifest['parent_build'])
    cm = parent / 'build/CMakeFiles/online_mppi.dir'
    includes = next(row.split('=', 1)[1] for row in (cm / 'flags.make').read_text().splitlines() if row.startswith('CXX_INCLUDES ='))
    obj = out / 'directed_reference_probe.o'
    # O0 minimizes this gate's compile cost; it is a small test translation
    # unit, not a new experiment/controller build or performance benchmark.
    subprocess.run(['/usr/bin/c++', '-std=gnu++17', '-O0', '-fPIC', '-D_GLIBCXX_ASSERTIONS',
                    '-DDIRECTED_WITH_CONTROLLER', '-I' + str(build / 'source'), *shlex.split(includes),
                    '-c', str(probe), '-o', str(obj)], check=True)
    link = shlex.split((cm / 'link.txt').read_text())
    old = [arg for arg in link if arg.endswith('/online_api.cpp.o') or arg == 'online_api.cpp.o']
    if len(old) != 1:
        raise ValueError('ambiguous probe link recipe')
    library = out / 'libdirected_reference_probe.so'
    link = [str(obj) if arg == old[0] else arg for arg in link]
    link[link.index('-o') + 1] = str(library)
    subprocess.run(link, cwd=str(parent / 'build'), check=True)
    code = ('import ctypes,json,sys;lib=ctypes.CDLL(sys.argv[1]);'
            'a=lib.directed_cursor_regression();b=lib.directed_controller_regression() if a==0 else -1;'
            'print("DIRECTED_TEST_RESULT "+json.dumps(dict(cursor_code=a,controller_code=b)));'
            'sys.exit(0 if a==b==0 else 1)')
    child = subprocess.run(['python3', '-c', code, str(library)], capture_output=True, text=True)
    encoded = next((line.split(' ', 1)[1] for line in child.stdout.splitlines() if line.startswith('DIRECTED_TEST_RESULT ')), None)
    codes = json.loads(encoded) if encoded else {}
    report = dict(passed=child.returncode == 0, returncode=child.returncode, codes=codes,
                  cursor_failure=CURSOR_FAILURES.get(codes.get('cursor_code')),
                  controller_failure=CONTROLLER_FAILURES.get(codes.get('controller_code')),
                  bounds_assertions=True, source_sha256=hashes, probe_sha256=sha(probe),
                  checks=['initial actual binding remains phase0', 'near-cusp and measured low speed gate only',
                          'same gear and stationary sampling conventions', 'candidate forecasts cannot advance actual cursor',
                          'new reference/reset clear phase and cached guide while retaining actuator feedback',
                          'full native goal and future segments retained', 'local connector independent index mapping',
                          'nonfinal cusp has no artificial terminal rotation', 'nonzero native endpoint speeds brake',
                          'off-path cusp projection retains reconnect motion', 'true final-yaw capture remains active'],
                  scope='Independent cursor and actual controller source boundaries, not collision-cost tuning, online success, or physical safety.',
                  stdout=child.stdout, stderr=child.stderr)
    if report['passed'] and fixture is not None:
        report['frozen_fixture'] = frozen_fixture(library, fixture)
    report['sources_unchanged_during_test'] = all(sha(build / 'source' / name) == value for name, value in hashes.items())
    report['passed'] = report['passed'] and report['sources_unchanged_during_test']
    (out / 'tests.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: report[k] for k in ('passed', 'codes', 'cursor_failure', 'controller_failure')}, indent=2))
    if not report['passed']:
        raise AssertionError(report)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--fixture', type=Path)
    args = parser.parse_args()
    test(args.build, args.out, args.fixture)
