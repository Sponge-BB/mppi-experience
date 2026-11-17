#!/usr/bin/env python3
"""Offline active-arm protocol/gradient checks; no ROS graph or experiment run.

Uses only new temporary files and frozen parent's compiler include recipe.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile

import active_arm_coupling_transform as transform

HERE = Path(__file__).resolve().parent
DEFAULT_BASE = HERE / 'builds/short_prefix_rejoin_20261101_01'
TRANSFORMS = {
    'collab_factorial.hpp': transform.transform_collab,
    'coupling_bridge.hpp': transform.transform_bridge,
    'poly_traj_optimizer.cpp': transform.transform_optimizer_unfreeze,
    'remani_replan_fsm.cpp': transform.transform_joint_feedback,
    'native_collision_export.hpp': transform.transform_live_collision_export,
}


def native_fixture(text):
    text = transform.once(text, '    for(int d=2;d<8;++d){', '    for(int d=2;d<8;++d){')
    text = transform.once(text, 'x.head[b](d,0)=.01*d;x.tail[b](d,0)=.01*d;',
                          'x.head[b](d,0)=.01*d;x.tail[b](d,0)=.025*d;')
    text = transform.once(text, '      x.points[b].row(d).setConstant(.01*d);', '''      for(int k=0;k<x.points[b].cols();++k)x.points[b](d,k)=.01*d+.004*d*(k+1);
      x.head[b](d,1)=.003*d;x.tail[b](d,1)=.005*d;''')
    text = transform.once(text, '  return p;\n}', '''  p.arm_valid=true;p.arm_delivery.q.setConstant(.09);p.arm_delivery.dq.setConstant(.02);
  return p;
}''')
    text = transform.once(text, '  auto x=input();auto p=prediction();', '''  setenv("ACTIVE_ARM_ENABLED","1",1);setenv("ACTIVE_ARM_COUPLING","1",1);
  auto x=input();auto p=prediction();''')
    text = transform.once(text, '  requestBoundaries();', '''  // Active handoff must propagate all the way to native arm waypoints.
  weights(0.,2.3);evaluate(x,p,&g);
  require(g.points[0].bottomRows(6).norm()>1e-7,"joint handoff did not reach native P");
  auto absent=p;absent.arm_valid=false;
  require(evaluate(x,absent,&g)==0.,"missing arm target enabled active handoff");zero(g);
  // The disabled group retains original prediction behavior and contracts.
  setenv("ACTIVE_ARM_ENABLED","0",1);requestBoundaries();
  weights(1.7,2.3);gradients(x,p);''')
    return text.replace('NATIVE_PREDICTION_GRADIENTS_PASSED', 'ACTIVE_ARM_NATIVE_PREDICTION_GRADIENTS_PASSED')


def verify(base, out):
    base = base.resolve()
    out.mkdir(parents=True, exist_ok=True)
    src = out / 'source'
    src.mkdir()
    for path in (base / 'source').glob('*.hpp'):
        shutil.copy2(path, src / path.name)
    shutil.copy2(HERE / 'active_arm_coupling.hpp', src)
    for name, fn in TRANSFORMS.items():
        original = (base / 'source' / name).read_text()
        changed = fn(original)
        if changed == original:
            raise AssertionError('transform made no change: ' + name)
        (src / name).write_text(changed)
    # Every frozen source is exercised; transformed C++ headers are compiled
    # by the native test. Full FSM/optimizer compilation belongs to child build.
    test = src / 'test_active_arm_native_prediction.cpp'
    test.write_text(native_fixture((HERE / 'test_coupling_native_prediction.cpp').read_text()))
    results = []
    pure = out / 'test_active_arm_coupling'
    subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra',
                    '-Wno-misleading-indentation', '-I/usr/include/eigen3',
                    str(HERE / 'test_active_arm_coupling.cpp'), '-o', str(pure)], check=True)
    pure_run = subprocess.run([str(pure)], check=True, capture_output=True, text=True)
    (out / 'algebra.log').write_text(pure_run.stdout + pure_run.stderr)
    results.append(pure_run.stdout.strip())
    print(pure_run.stdout, end='', flush=True)
    manifest = json.loads((base / 'build_manifest.json').read_text())
    original_command = manifest['rejoin_recipe']['rejoin_test_command']
    # Keep only include options, preserving -isystem's separate argument.
    includes = []
    consume_next = False
    for arg in original_command:
        if consume_next:
            includes.append(arg)
            consume_next = False
        elif arg == '-isystem':
            includes.append(arg)
            consume_next = True
        elif arg.startswith('-I'):
            includes.append(arg)
    executable = out / 'test_active_arm_native_prediction'
    command = ['/usr/bin/c++', '-std=gnu++17', '-O2', '-I' + str(src), *includes,
               str(test), '-L/opt/ros/noetic/lib', '-Wl,-rpath,/opt/ros/noetic/lib',
               '-lroscpp', '-lrostime', '-lcpp_common', '-lrosconsole',
               '-lroscpp_serialization', '-pthread', '-o', str(executable)]
    subprocess.run(command, check=True)
    run = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
    (out / 'native.log').write_text(run.stdout + run.stderr)
    print(run.stdout, end='', flush=True)
    results.append(run.stdout.strip())
    result = dict(passed=True, no_ros_graph=True, base=str(base),
                  transformed=list(TRANSFORMS), results=results,
                  native_compile_command=command)
    (out / 'verification.json').write_text(json.dumps(result, indent=2) + '\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, default=DEFAULT_BASE)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    destination = args.out or Path(tempfile.mkdtemp(prefix='active_arm_coupling_verify_'))
    verify(args.base, destination.resolve())
    print('ACTIVE_ARM_OFFLINE_VERIFICATION', destination)
