#!/usr/bin/env python3
"""Prepare complete saved-input replay and its native build recipe.

This helper never starts a ROS master. `run` requires a private, preloaded
parameter server and executes only the standalone native replay program.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess
from urllib.parse import urlparse

HERE = Path(__file__).resolve().parent
TRIAL = HERE / 'results/candidate_exit_audit_20261101_01/headon_capture'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    return json.loads(Path(path).read_text(), parse_int=lambda token: -0.0 if token == '-0' else int(token))


def records(path):
    return [json.loads(line, parse_int=lambda token: -0.0 if token == '-0' else int(token))
            for line in Path(path).read_text().splitlines() if line.strip()]


def vector_hash(values):
    value = 1469598103934665603
    for x in values:
        for byte in struct.pack('<d', x):
            value = ((value ^ byte) * 1099511628211) & ((1 << 64) - 1)
    return str(value)


def prepare(trial, output):
    capture_path = trial / 'exit_audit/capture.jsonl'
    rows = records(capture_path)
    if len(rows) != 1:
        raise ValueError('Expected exactly one predetermined exit audit')
    captured = rows[0]
    if captured['stable'] is not True or not captured['accepted_available']:
        raise ValueError('Saved native input was not stable or accepted_x unavailable')
    if vector_hash(captured['accepted_evaluation']['x']) != captured['accepted_x_hash']:
        raise ValueError('Accepted x hash mismatch, including signed zero')
    if vector_hash(captured['accepted_evaluation']['gradient']) != captured['accepted_evaluation']['gradient_hash']:
        raise ValueError('Captured gradient hash mismatch, including signed zero')
    temporal_path = trial / 'temporal_collision.jsonl'
    temporal = records(temporal_path)
    native = [r for r in temporal if r.get('request_id') == captured['request_id']
              and r['stage'] == 'exit_accepted_candidate']
    planning = [r for r in temporal if r.get('request_id') == captured['request_id'] and r['stage'] == 'planning']
    if len(native) != 1 or len(planning) != 1:
        raise ValueError('Missing unique same-request native polynomial/gears')
    parts = native[0]['candidate']['parts']
    pieces = [len(part['pieces']) for part in parts]
    gears = [part['gear'] for part in parts]
    if gears != planning[0]['gears']:
        raise ValueError('Saved candidate and planning gears differ')
    expected_variables = sum(8 * (n - 1) + n for n in pieces) + 9 * (len(pieces) - 1)
    if expected_variables != len(captured['accepted_evaluation']['x']):
        raise ValueError('Native part metadata and accepted_x dimension differ')
    map_path = trial / 'exit_audit/map.json'
    metadata = load(map_path)
    parameters = load(trial / 'coupling_parameters.json')
    paths = [capture_path, temporal_path, map_path, trial / 'parameters.yaml', trial / 'effective_real_params.json',
             trial / 'coupling_parameters.json', Path(metadata['esdf_path']), Path(metadata['overlay_path'])]
    value = dict(schema='window_continuity_saved_input_v1', capture=captured, pieces=pieces, gears=gears,
        map=metadata, map_path=str(map_path.resolve()), original_parameter_yaml=str((trial / 'parameters.yaml').resolve()),
        execution_weight=parameters['execution_weight'], delivery_weight=parameters['handover_weight'],
        source_sha256={str(p.resolve()): sha(p) for p in paths}, original_coordinate_scale_available=False,
        continuation_scale='one cheap proxy estimate at saved accepted_x shared by old and new objective',
        budget_s=captured['original_remaining_budget_s'], hardware=False,
        purpose='Exact original objective check, then equal-input/equal-budget continuity comparison; no online performance inference')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')
    return value


def recipe(build, output_directory):
    """Derive compile/link from the immutable child's actual native recipe."""
    manifest = load(build / 'build_manifest.json')
    candidates = [v for v in manifest.values() if isinstance(v, dict)
                  and 'compile_commands' in v and 'link_commands' in v]
    applicable = [r for r in candidates if any(str(build / 'source') in str(c['source'])
                  for c in r['compile_commands'])]
    if not applicable:
        raise ValueError('No child-specific native recipe found')
    native = applicable[-1]
    command = next(c['command'].copy() for c in native['compile_commands']
                   if Path(c['source']).name == 'poly_traj_optimizer.cpp')
    source = HERE / 'window_continuity_replay.cpp'
    obj = output_directory / 'window_continuity_replay.o'
    target = output_directory / 'window_continuity_replay'
    command[command.index('-c') + 1] = str(source)
    command[command.index('-o') + 1] = str(obj)
    link = [x for x in native['link_commands']['remani_planner_node'] if not x.endswith('.cpp.o')]
    link.insert(1, str(obj));link[link.index('-o') + 1] = str(target)
    if '-lz' not in link:
        link.append('-lz')
    return dict(cwd=native['cwd'], compile=command, link=link, binary=str(target),
                source_sha256=sha(source), native_manifest_sha256=sha(build / 'build_manifest.json'),
                required_child_header_changes=['candidate_audit.hpp: import rootcause_map_loader.METHOD after public:',
                    'candidate_audit.hpp: include boost/property_tree/ptree.hpp and boost/property_tree/json_parser.hpp',
                    'window_continuity.hpp and transformed native optimizer from the same child'])


def run(binary, input_path, output, master_uri, mode):
    uri = urlparse(master_uri)
    if uri.hostname not in ('localhost', '127.0.0.1') or uri.port in (None, 11311):
        raise ValueError('Standalone replay requires an explicitly private loopback ROS master')
    data = load(input_path)
    for path, digest in data['source_sha256'].items():
        if sha(path) != digest:
            raise ValueError('Saved input changed: ' + path)
    env = os.environ.copy()
    for key in list(env):
        if key.startswith(('FAILURE_', 'RESOURCE_', 'COLLAB_', 'COUPLING_', 'STATE_CONTRACT_', 'DELIVERY_',
                           'NATIVE_', 'CANDIDATE_', 'WHEEL_CAUSE_', 'ENDPOINT_TIME_', 'BUDGET_',
                           'SEED_PARAMETERIZATION_', 'FEASIBILITY_', 'DYNAMIC_WINDOW_', 'TIME_INDEXED_', 'COVERAGE_')):
            env.pop(key)
    env.update(ROS_MASTER_URI=master_uri, ROS_IP='127.0.0.1', ROS_HOSTNAME='localhost',
        NATIVE_COLLISION_3D='1', DELIVERY_FAST_QUERY='1', DELIVERY_ACCEPT_CANDIDATE='1',
        FAILURE_PROFILE_DISABLED='1', COUPLING_ENGINEERING='1',
        COUPLING_EXEC_WEIGHT=str(data['execution_weight']), COUPLING_HAND_WEIGHT=str(data['delivery_weight']),
        TIME_INDEXED_COLLISION='1', COVERAGE_ALIGNMENT='1', COUPLING_DOMAIN_RECOVERY='1')
    output.parent.mkdir(parents=True, exist_ok=True)
    # SetParam reads the captured parameters; no sensing/control node is run.
    subprocess.run(['/opt/ros/noetic/bin/rosparam', 'load', data['original_parameter_yaml']], env=env, check=True)
    subprocess.run([str(binary), str(input_path), data['map_path'], str(output), mode], env=env, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='action', required=True)
    p = subs.add_parser('prepare');p.add_argument('--trial', type=Path, default=TRIAL);p.add_argument('--output', type=Path, required=True)
    p = subs.add_parser('recipe');p.add_argument('--build', type=Path, required=True);p.add_argument('--output-directory', type=Path, required=True)
    p = subs.add_parser('run');p.add_argument('--binary', type=Path, required=True);p.add_argument('--input', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True);p.add_argument('--master-uri', required=True)
    p.add_argument('--mode', choices=('check', 'old', 'continuous', 'both'), default='both')
    args = parser.parse_args()
    if args.action == 'prepare':
        value = prepare(args.trial.resolve(), args.output.resolve())
        print(json.dumps(dict(input=str(args.output.resolve()), pieces=value['pieces'], gears=value['gears'], budget_s=value['budget_s'])))
    elif args.action == 'recipe':
        print(json.dumps(recipe(args.build.resolve(), args.output_directory.resolve()), indent=2))
    else:
        run(args.binary.resolve(), args.input.resolve(), args.output.resolve(), args.master_uri, args.mode)


if __name__ == '__main__':
    main()
