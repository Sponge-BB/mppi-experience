#!/usr/bin/python3
"""One artifact-only startup trial; defaults to offline launch preparation."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import sys
import time
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import yaml

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path.insert(0, str(ROOT/'src/mm_remani/scripts'))
import run_standby_sim as runner

original_prepare = runner.prepare
command_stream = None


def prepare(args):
    global command_stream
    out = original_prepare(args)
    source = ROOT/'experiments/online_remani_validation/results/hardware_startup_check_20261107/runtime_feedback.json'
    record = json.loads(source.read_text())
    target = record['startup_parameters']['/remani_planner_node/fsm/init_state']
    overrides = {'/'+name+'/fsm/init_state': target for name in ('remani_planner_node', 'mm_controller_node')}
    overrides['/whole_body_mppi/startup_to_standby'] = True
    params = yaml.safe_load((out/'parameters.yaml').read_text()); params.update(overrides)
    (out/'parameters.yaml').write_text(yaml.safe_dump(params))
    (out/'planner_params.yaml').write_text(yaml.safe_dump(
        {key: value for key, value in params.items() if key.startswith('/remani_planner_node/')}))
    existing = json.loads((out/'simulation_overrides.json').read_text()); existing.update(overrides)
    (out/'simulation_overrides.json').write_text(json.dumps(existing, indent=2)+'\n')
    launch = ET.parse(str(out/'trial.launch'))
    simulator = launch.find("./node[@name='standby_simulator']")
    simulator.set('launch-prefix', '/usr/bin/python3 '+shlex.quote(str(OUT/'startup_simulator_wrapper.py')))
    launch.write(str(out/'trial.launch'), encoding='unicode')
    meta = json.loads((out/'experiment.json').read_text())
    meta['startup_override'] = dict(parameters=overrides,
        initial_q_rad=record['topics']['/joint_states_single']['position_rad'],
        source=str(source.relative_to(ROOT)), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        simulator_override='Only module Q_START changes; original 11466/use_sim_time entry guards retained',
        production_files_modified=False, free_base=True,
        hardware_equivalence=False, driver_watchdog_note='Plant brakes after .30 s but does not latch; report command gaps explicitly',
        runner_memory_guards=dict(minimum_available_mib=1200, maximum_graph_rss_mib=1600),
        snapshot_note='Real compiled mapper/exporter; original complete fusion timestamps retained, no synthetic freshness')
    (out/'experiment.json').write_text(json.dumps(meta, indent=2)+'\n')
    return out


def main():
    global command_stream
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true', help='Run only after the build/tests and review are complete')
    parser.add_argument('--out', type=Path)
    parser.add_argument('--duration', type=float, default=60.)
    options = parser.parse_args()
    if not 0. < options.duration <= 120.:
        parser.error('--duration must be between 0 and 120 seconds')
    out = options.out or OUT/('ros_startup_integration_s11' if options.run else 'ros_startup_prepared')
    args = SimpleNamespace(group='mppi_arm', out=out, seed=11, scene='empty', sensor_model='mid360',
        goal_x=None, duration=options.duration, realtime_factor=1., uncertainty_scale=1., arm_execution_model='velocity_legacy')
    runner.prepare = prepare
    if not options.run:
        print(prepare(args)/'trial.launch')
        return
    original_subscriber = runner.rospy.Subscriber

    def subscribe(topic, message_type, callback, *rest, **kwargs):
        if topic == '/sim/joint_ctrl_single':
            counted = callback

            def callback(message):
                command_stream.write(json.dumps(dict(ros_time=runner.rospy.get_time(),
                    source_stamp=message.header.stamp.to_sec(), wall_monotonic=time.monotonic(),
                    position=list(message.position)))+'\n')
                counted(message)
        return original_subscriber(topic, message_type, callback, *rest, **kwargs)

    def prepare_and_record(args):
        global command_stream
        prepared = prepare(args)
        command_stream = (prepared/'arm_command_times.jsonl').open('w', buffering=1)
        return prepared

    runner.prepare = prepare_and_record
    runner.rospy.Subscriber = subscribe
    sys.argv = [sys.argv[0], '--group', args.group, '--out', str(args.out), '--seed', str(args.seed),
        '--scene', args.scene, '--sensor-model', args.sensor_model, '--duration', str(args.duration),
        '--realtime-factor', str(args.realtime_factor), '--uncertainty-scale', str(args.uncertainty_scale),
        '--arm-execution-model', args.arm_execution_model]
    try:
        runner.main()
    finally:
        runner.rospy.Subscriber = original_subscriber
        if command_stream is not None:
            command_stream.close()


if __name__ == '__main__':
    main()
