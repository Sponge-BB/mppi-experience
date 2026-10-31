#!/usr/bin/env python3
"""Compile tests against exact isolated KinoAstar functions, without ROS."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import subprocess

from build_native_v9 import HERE, sha
from patch_seed_parameterization import transform_search, project_off
from report_domain_recovery import assert_plot_quiet
from verify_delivery_seed_geometry import CLASS, extract_function, parameter_values, textsha


def extract(text, name):
    if name != 'evaluatePos':
        return extract_function(text, name)
    signature = 'Eigen::Vector3d KinoAstar::evaluatePos'
    if text.count(signature) != 1:
        raise ValueError('Ambiguous evaluatePos')
    surrogate = text.replace(signature, 'double KinoAstar::evaluatePos', 1)
    found = extract_function(surrogate, name)
    found['text'] = found['text'].replace('double KinoAstar::evaluatePos', signature, 1)
    found['sha256'] = textsha(found['text'])
    found.pop('start_byte'); found.pop('end_byte')
    return found


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, default=HERE/'builds/delivery_stability_20261024_02/source/kino_astar.cpp')
    p.add_argument('--previous-batch', type=Path, default=HERE/'results/delivery_stability_20261024_01')
    p.add_argument('--out', type=Path, required=True)
    a = p.parse_args()
    jobs = json.loads((a.previous_batch/'complete.json').read_text())['completed']
    assert_plot_quiet(dict(trials=[dict(run_status=json.loads(
        (a.previous_batch/j['trial']/'run_status.json').read_text())) for j in jobs]))
    original = a.source.read_text(); modified = transform_search(original)
    if project_off(modified, a.source) != original:
        raise ValueError('OFF projection not exact')
    names = ('getTrajsWithTime', 'evaluateCurvatureWeightedDistance', 'evaluateDuration',
             'evaluateLength', 'getFlatState', 'evaluatePos')
    functions = [extract(modified, name) for name in names]
    native_header = HERE.parent.parent/'src/mm_remani_planner/path_searching/include/path_searching/kino_astar.h'
    container = HERE.parent.parent/'src/mm_remani_planner/traj_utils/include/traj_utils/plan_container.hpp'
    macros = re.findall(r'^#define inf[^\n]*', native_header.read_text(), re.M)
    struct = re.search(r'  struct CarFlatTrajData\{.*?\n  \};', container.read_text(), re.S)
    if len(macros) != 1 or not struct:
        raise ValueError('Missing exact native macro/flat struct')
    parameter_path = a.previous_batch/'static_capture_seed11/parameters.yaml'
    parameters = parameter_values(parameter_path)
    a.out.mkdir(parents=True, exist_ok=False)
    klass = CLASS.replace(' void getTrajsWithTime();',
        ' Eigen::Vector3d evaluatePos(const double&);\n void getTrajsWithTime();')
    generated = ('using namespace Eigen;\nusing namespace std;\n'+macros[0]+'\n'+struct.group()+'\n'+klass+
        '\n\n'.join(f['text'] for f in functions)+'\n')
    (a.out/'native_functions.hpp').write_text(generated)
    fixture = 'void configure(KinoAstar& s){\n'+''.join('s.%s=%.17g;\n'%(k,v)
        for k,v in parameters.items())+'}\n'
    (a.out/'native_fixture.hpp').write_text(fixture)
    command = ['/usr/bin/c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-pedantic',
        '-DSEED_NATIVE_EXTRACTED', '-I/usr/include/eigen3', '-I'+str(a.out.resolve()),
        '-I'+str(HERE), str(HERE/'test_seed_parameterization.cpp'), '-o', str(a.out/'test_seed_parameterization')]
    with (a.out/'compile.log').open('w') as log:
        compiled = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    if compiled.returncode:
        raise RuntimeError('Compilation failed; log retained')
    with (a.out/'stdout.log').open('w') as out, (a.out/'stderr.log').open('w') as err:
        ran = subprocess.run([str((a.out/'test_seed_parameterization').resolve())], stdout=out, stderr=err, timeout=40)
    summary = dict(created=datetime.now(timezone.utc).isoformat(),
        source=str(a.source.resolve()), source_sha256=sha(a.source), exact_off_projection=True,
        functions=functions, native_macro=macros[0], parameters=parameters,
        parameter_source=str(parameter_path), parameter_source_sha256=sha(parameter_path),
        compile_returncode=compiled.returncode, test_returncode=ran.returncode,
        compiler_command=command, binary_sha256=sha(a.out/'test_seed_parameterization'),
        verifier_sha256=sha(Path(__file__)), harness_sha256=sha(HERE/'test_seed_parameterization.cpp'),
        helper_sha256=sha(HERE/'seed_parameterization.hpp'),
        generated_functions_sha256=sha(a.out/'native_functions.hpp'),
        generated_fixture_sha256=sha(a.out/'native_fixture.hpp'),
        ros_started=False, production_modified=False,
        scope='Six exact transformed native function texts; minimal class excludes search/map/ROS, not a complete planner test')
    (a.out/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    if ran.returncode:
        raise RuntimeError('Tests failed; all output retained')
    print('ACTUAL_NATIVE_SEED_TESTS_PASSED', a.out, flush=True)


if __name__ == '__main__':
    main()
