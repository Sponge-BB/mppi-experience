#!/usr/bin/env python3
"""Compile/run the actual C++ model test on a private, master-only ROS graph."""
import argparse
import json
import os
from pathlib import Path
import socket
import subprocess
import time
import xmlrpc.client

import yaml

from build_urdf_spheres_b import DEFAULT_OUT as BUILD, check, recipes
from run_urdf_spheres_b import MODEL, sphere_rows

HERE = Path(__file__).resolve().parent
PARAMETERS = HERE/'results/watchdog_off_20261029_01/mid_cross_seed11/parameters.yaml'
DEFAULT_OUT = HERE/'results/urdf_spheres_b_validation_20261029_01'
PORT = 11396


def private_parameters():
    root = {}
    for key, value in yaml.safe_load(PARAMETERS.read_text()).items():
        prefix = '/remani_planner_node/'
        if not key.startswith(prefix):
            continue
        fields = key[len(prefix):].split('/')
        node = root
        for field in fields[:-1]:
            node = node.setdefault(field, {})
        node[fields[-1]] = value
    root['mm']['fixed_collision_spheres'] = sphere_rows(json.loads(MODEL.read_text()))
    return root


def validate(out=DEFAULT_OUT):
    check(BUILD)
    out = Path(out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out/'model_test.json').exists():
        raise FileExistsError('A validation result already exists; select a new output directory')
    recipe = recipes(BUILD)
    source, obj, binary = HERE/'test_urdf_spheres_b.cpp', out/'model_test.o', out/'model_test'
    command = list(recipe['compile_commands'][0]['command'])
    command[command.index('-c')+1] = str(source)
    command[command.index('-o')+1] = str(obj)
    subprocess.run(command, check=True)
    link = list(recipe['link_commands']['remani_planner_node'])
    first = next(i for i, value in enumerate(link) if value.endswith('.o'))
    link = link[:first]+[str(obj)]+[v for v in link[first:] if not v.endswith('.o')]
    link[link.index('-o')+1] = str(binary)
    subprocess.run(link, cwd=recipe['cwd'], check=True)
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', PORT)) == 0:
            raise RuntimeError('Private test master port is already in use')
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d'%PORT,
               ROS_IP='127.0.0.1', ROS_HOSTNAME='localhost',
               ROS_HOME=str(out/'ros_home'), ROS_LOG_DIR=str(out/'roslogs'))
    with (out/'master.log').open('w') as log:
        master = subprocess.Popen(['roscore', '-p', str(PORT)], env=env,
                                  stdout=log, stderr=subprocess.STDOUT)
        try:
            proxy = xmlrpc.client.ServerProxy(env['ROS_MASTER_URI'])
            for _ in range(100):
                if master.poll() is not None:
                    raise RuntimeError('Private ROS master exited; inspect master.log')
                try:
                    if proxy.getPid('/urdf_spheres_b_validation')[0] == 1:
                        break
                except OSError:
                    pass
                time.sleep(.1)
            else:
                raise TimeoutError('Private master did not become ready')
            result = proxy.setParam('/urdf_spheres_b_validation',
                                    '/urdf_spheres_b_model_test', private_parameters())
            if result[0] != 1:
                raise RuntimeError('Test parameter upload failed')
            process = subprocess.run([str(binary)], env=env, text=True,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30.)
            (out/'model_test.log').write_text(process.stdout)
            if process.returncode:
                raise RuntimeError('C++ model test failed; inspect model_test.log')
            prefix = 'URDF_SPHERES_B_RESULT '
            row, = [line[len(prefix):] for line in process.stdout.splitlines() if line.startswith(prefix)]
            result = json.loads(row)
            result.update(scope='actual C++ model, private master only; no planner, simulator, actuator or hardware node',
                          source_parameters=str(PARAMETERS), compile_command=command)
            (out/'model_test.json').write_text(json.dumps(result, indent=2)+'\n')
            if result['base_arm_hard_collision'] or result['base_arm_safe_collision']:
                raise RuntimeError('Fixed arm collides with candidate fixed structure; result retained')
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
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    print(json.dumps(validate(args.out), ensure_ascii=False))
