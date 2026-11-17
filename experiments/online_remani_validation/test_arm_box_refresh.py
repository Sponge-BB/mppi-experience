#!/usr/bin/env python3
"""Compile/run the tiny pure policy tests; validate immutable source transforms."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import time

from arm_box_refresh_transform import transform_sources, SEED_CONTEXT, PREFIX_ROWS


HERE = Path(__file__).resolve().parent
PARENT = HERE/'builds/active_arm_20261104_01/source'
NAMES = ('remani_replan_fsm.cpp', 'collab_factorial.hpp', 'coupling_bridge.hpp', 'planner_manager.cpp')


def context_fixture():
    # Execute the actual child transform's scope/prefix code. Only data-carrier
    # types and the ROS clock are replaced; no validation/interpolation/policy
    # implementation is duplicated in this fixture.
    return r'''
#include <array>
#include <vector>
#include <string>
#include <algorithm>
#include <initializer_list>
namespace ros {
static double test_now=100.;
struct Time {double value;double toSec()const{return value;}static Time now(){return {test_now};}};
}
namespace Eigen {
struct VectorXd {
  std::vector<double> values;
  VectorXd(std::initializer_list<double> v):values(v){}
  double operator()(int i)const{return values.at(i);}
};
}
namespace active_arm_coupling {
inline bool armEnabled(){return true;}
struct JointState {std::array<double,6> q{},dq{};};
struct Row {std::array<double,8> base{};JointState arm;};
struct Seed {double joint_stamp=0.;std::vector<Row> rows;};
}
namespace collab {
struct Seed {
  double stamp=0.,revision=0.,generation=0.,goal=0.,odom=0.,cloud=0.,dynamic=0.;
  std::vector<std::array<double,8>> rows;active_arm_coupling::Seed arm;
};
inline Seed& seed(){static Seed value;return value;}
''' + SEED_CONTEXT + r'''
inline double angle(double value){return std::atan2(std::sin(value),std::cos(value));}
''' + PREFIX_ROWS + '\n} // namespace collab\n'


def run_tests():
    start = time.monotonic()
    with tempfile.TemporaryDirectory(prefix='arm_box_refresh_test_') as directory:
        binary = Path(directory)/'policy_test'
        fixture = Path(directory)/'scope_fixture.hpp'
        fixture.write_text(context_fixture())
        command = ['/usr/bin/c++', '-std=c++14', '-O0', '-g0', '-Wall', '-Wextra',
                   '-I'+str(HERE), '-I'+directory, '-DARM_BOX_CONTEXT_HEADER="scope_fixture.hpp"',
                   str(HERE/'test_arm_box_refresh.cpp'), '-o', str(binary)]
        compiled = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if compiled.returncode:
            raise RuntimeError('Policy test compile failed:\n'+compiled.stdout)
        checked = subprocess.run([str(binary)], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        if checked.returncode:
            raise RuntimeError('Policy mechanism test failed:\n'+checked.stdout)
        mechanism = json.loads(checked.stdout)
    # A child must never mutate input strings or silently apply itself twice.
    original = {name:(PARENT/name).read_text() for name in NAMES}
    before = {name:hashlib.sha256(text.encode()).hexdigest() for name,text in original.items()}
    modified = transform_sources(original)
    after = {name:hashlib.sha256(text.encode()).hexdigest() for name,text in original.items()}
    if before != after or set(modified) != set(original):
        raise AssertionError('Transform mutated parent input or changed the source inventory')
    if not all(modified[name] != original[name] for name in NAMES):
        raise AssertionError('Expected four declared changed translation inputs')
    rejected = False
    try:
        transform_sources(modified)
    except ValueError:
        rejected = True
    if not rejected:
        raise AssertionError('Transform silently accepted already transformed source')
    return dict(passed=True, mechanism=mechanism, immutable_transform_passed=True,
                repeat_transform_rejected=True, compiler_command=command,
                compiler_messages=compiled.stdout, elapsed_s=time.monotonic()-start,
                ros_started=False, parent_modified=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    result = run_tests()
    if args.out:
        args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))
