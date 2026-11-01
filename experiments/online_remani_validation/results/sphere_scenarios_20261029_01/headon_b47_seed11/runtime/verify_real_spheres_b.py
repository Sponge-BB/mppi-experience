#!/usr/bin/env python3
"""Check launch resolution and installed geometry; never run robot drivers."""
import argparse
import json
import os
from pathlib import Path
import shlex
import socket
import subprocess
import time
import xmlrpc.client

import yaml

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'backups/real_urdf_spheres_b_20261029_01'
PORT = 11396


def launch_parameters():
    resolved = {}
    for enabled in (True, False):
        command = ['roslaunch', '--dump-params', 'mm_remani', 'exp0_real.launch',
                   'use_urdf_collision_b:='+str(enabled).lower(), 'enable_model_vis:=true']
        process = subprocess.run(command, cwd=ROOT, text=True, stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE, check=True)
        params = yaml.safe_load(process.stdout)
        for namespace in ('remani_planner_node', 'mm_controller_node', 'model_vis'):
            key = '/'+namespace+'/mm/fixed_collision_spheres_enable'
            if params[key] is not enabled:
                raise ValueError('Launch did not resolve explicit model switch: '+key)
            spheres = params.get('/'+namespace+'/mm/fixed_collision_spheres')
            if enabled and (spheres is None or len(spheres) != 47):
                raise ValueError('New sphere rows missing from '+namespace)
            if not enabled and spheres is not None:
                raise ValueError('OFF unexpectedly loads new sphere rows')
        label = 'on' if enabled else 'off'
        (OUT/('launch_params_'+label+'.yaml')).write_text(process.stdout)
        resolved[enabled] = params
    def without_geometry(params):
        return {key: value for key, value in params.items()
                if '/mm/fixed_collision_spheres' not in key}
    if without_geometry(resolved[True]) != without_geometry(resolved[False]):
        raise ValueError('ON/OFF unexpectedly changed other ROS parameters')
    result = dict(passed=True, namespaces=3, enabled_spheres_per_namespace=47,
                  off_has_no_new_rows=True, other_parameters_identical=True,
                  scope='roslaunch parameter resolution only; no nodes started')
    (OUT/'launch_check.json').write_text(json.dumps(result, indent=2)+'\n')
    return resolved[True]


def compile_test():
    flag_file = ROOT/'build/mm_remani_planner/mm_config/CMakeFiles/mm_config.dir/flags.make'
    flags = {}
    for line in flag_file.read_text().splitlines():
        if line.startswith(('CXX_INCLUDES =', 'CXX_FLAGS =', 'CXX_DEFINES =')):
            key, value = line.split('=', 1)
            flags[key.strip()] = shlex.split(value)
    obj, binary = OUT/'installed_model_test.o', OUT/'installed_model_test'
    source = ROOT/'experiments/online_remani_validation/test_urdf_spheres_b.cpp'
    command = ['/usr/bin/c++', *flags['CXX_FLAGS'], *flags['CXX_DEFINES'],
               *flags['CXX_INCLUDES'], '-c', str(source), '-o', str(obj)]
    subprocess.run(command, cwd=ROOT/'build', check=True)
    link_file = ROOT/'build/mm_remani_planner/plan_manage/CMakeFiles/remani_planner_node.dir/link.txt'
    link = shlex.split(link_file.read_text())
    first = next(i for i, value in enumerate(link) if value.endswith('.o'))
    link = link[:first]+[str(obj)]+[value for value in link[first:] if not value.endswith('.o')]
    link[link.index('-o')+1] = str(binary)
    subprocess.run(link, cwd=ROOT/'build', check=True)
    dependencies = subprocess.check_output(['ldd', str(binary)], text=True)
    (OUT/'installed_model_ldd.txt').write_text(dependencies)
    if str(ROOT/'devel/lib/libmm_config.so') not in dependencies or 'not found' in dependencies:
        raise RuntimeError('Test did not link the installed production model')
    return binary


def model_test(params):
    binary = compile_test()
    private = {}
    prefix = '/remani_planner_node/'
    for key, value in params.items():
        if not key.startswith(prefix):
            continue
        fields = key[len(prefix):].split('/')
        cursor = private
        for field in fields[:-1]:
            cursor = cursor.setdefault(field, {})
        cursor[fields[-1]] = value
    # Only the private test namespace uses the example startup pose. No live
    # robot joint limits or joints are touched by this master-only test.
    private['mm']['manipulator_min_pos'] = [0., .520979, -1.048914, 0., .947297, 0.]
    private['test_explicit_sphere_switch'] = True
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d'%PORT,
               ROS_IP='127.0.0.1', ROS_HOSTNAME='localhost',
               ROS_HOME=str(OUT/'test_ros_home'), ROS_LOG_DIR=str(OUT/'test_roslogs'))
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', PORT)) == 0:
            raise RuntimeError('Private test port already in use')
    with (OUT/'test_master.log').open('w') as log:
        master = subprocess.Popen(['roscore', '-p', str(PORT)], env=env,
                                  stdout=log, stderr=subprocess.STDOUT)
        try:
            proxy = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
            for _ in range(100):
                if master.poll() is not None:
                    raise RuntimeError('Test master exited')
                try:
                    if proxy.getPid('/verify_real_spheres_b')[0] == 1:
                        break
                except OSError:
                    pass
                time.sleep(.1)
            else:
                raise TimeoutError('Test master startup timed out')
            if proxy.setParam('/verify_real_spheres_b', '/urdf_spheres_b_model_test', private)[0] != 1:
                raise RuntimeError('Test parameter upload failed')
            process = subprocess.run([str(binary)], env=env, text=True, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, timeout=30.)
            (OUT/'installed_model_test.log').write_text(process.stdout)
            if process.returncode:
                raise RuntimeError('Installed C++ model test failed; inspect installed_model_test.log')
            marker = 'URDF_SPHERES_B_RESULT '
            row, = [line[len(marker):] for line in process.stdout.splitlines() if line.startswith(marker)]
            result = json.loads(row)
            result.update(installed_library=str(ROOT/'devel/lib/libmm_config.so'),
                          off_with_stale_rows_tested=True, enabled_missing_rows_rejected=True,
                          test_pose=private['mm']['manipulator_min_pos'],
                          scope='installed production library, example startup posture, private master only; not live pose or motion validation')
            (OUT/'installed_model_check.json').write_text(json.dumps(result, indent=2)+'\n')
            if result['base_arm_hard_collision'] or result['base_arm_safe_collision']:
                raise RuntimeError('Example startup posture fails fixed-structure self check')
            return result
        finally:
            master.terminate()
            try:
                master.wait(timeout=10.)
            except subprocess.TimeoutExpired:
                master.kill()
                master.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch-only', action='store_true')
    args = parser.parse_args()
    params = launch_parameters()
    if not args.launch_only:
        print(json.dumps(model_test(params), ensure_ascii=False))
    else:
        print('LAUNCH_PARAMETERS_OK: ON/OFF and three private namespaces')
