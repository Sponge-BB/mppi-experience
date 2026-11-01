#!/usr/bin/env python3
"""Repeat the long-goal crossing SIL with the per-cycle stack watchdog OFF.

Retain the same algorithm, binary, scene, limits, command lease and deadline.
Passive timing records remain, with an exit-only flush so they are not lost.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
BASELINE = HERE / 'results/long_goal_crossing_20261029_01/mid_cross_seed11'
DEFAULT_OUT = HERE / 'results/watchdog_off_20261029_01/mid_cross_seed11'


def disable_watchdog(out):
    from native_collision_runtime import replace_once
    out = Path(out)
    runtime = out / 'runtime'
    path = runtime / 'diagnose_execution_timing.py'
    source = path.read_text()
    for old, new in (
        ('import faulthandler\n', ''),
        ("        self.stacks = (self.out / 'loop_stacks.log').open('w')\n", ''),
        ('        faulthandler.dump_traceback_later(.3, repeat=True, file=self.stacks)\n',
         '        # No watchdog creation, cancellation or thread join in the control loop.\n'),
        ('        faulthandler.cancel_dump_traceback_later()\n        self.stacks.close()\n',
         "        if getattr(self, 'closed', False):\n            return\n        self.closed = True\n"),
        ('clocks=self.clocks, stack_threshold_s=.3,',
         'clocks=self.clocks, stack_threshold_s=None, watchdog_enabled=False,'),
        ("scope='diagnostic rerun only; instrumentation changes timing, not planning or safety logic'",
         "scope='passive timestamps only; watchdog disabled; exit-only flush, no planning/safety changes'"),
    ):
        source = replace_once(source, old, new)
    ast.parse(source)
    path.write_text(source)
    path = runtime / 'local_execution.py'
    source = replace_once(path.read_text(),
        '    node.timing_probe=LoopProbe(node.out,rospy.get_time)\n',
        '    node.timing_probe=LoopProbe(node.out,rospy.get_time)\n'
        '    rospy.on_shutdown(node.timing_probe.close) # exit-only flush, never a loop watchdog\n')
    ast.parse(source)
    path.write_text(source)
    shutil.copy2(Path(__file__), runtime / Path(__file__).name)
    path = out / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['watchdog_off_experiment'] = dict(
        baseline=str(BASELINE), per_cycle_stack_watchdog=False,
        passive_stage_timestamps=True, passive_clock_receipts=True,
        shutdown_flush_only=True, shutdown_flush_is_not_in_timed_control=True,
        unchanged='same native REMANI/MPPI binaries, seed11, 6 m goal, midpoint crossing, 4.10 rad/s wheel limit, 3 s forecast, 200/250 ms timing, geometry/cost/validation',
        scope='one same-settings full online ROS numerical SIL repeat, not Gazebo or hardware')
    manifest['execution_lag_diagnosis'].update(
        stack_threshold_s=None, watchdog_enabled=False,
        stages_in_memory=True, exit_only_flush=True,
        scope='passive timing only; the per-cycle faulthandler watchdog has been removed')
    for name in ('diagnose_execution_timing.py', 'local_execution.py', Path(__file__).name):
        manifest['experiment_sources_sha256'][name] = hashlib.sha256((runtime / name).read_bytes()).hexdigest()
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')


def check_same_settings(out, baseline=BASELINE):
    """Compare actual prepared inputs before ROS starts, excluding path names."""
    import xml.etree.ElementTree as ET
    import yaml
    out, baseline = Path(out), Path(baseline)
    checked = []
    for name in ('limits.json', 'shared_collision_config.json', 'scenario.json', 'geometry.json'):
        if json.loads((out / name).read_text()) != json.loads((baseline / name).read_text()):
            raise ValueError('Settings unexpectedly changed: ' + name)
        checked.append(name)
    if yaml.safe_load((out / 'parameters.yaml').read_text()) != yaml.safe_load((baseline / 'parameters.yaml').read_text()):
        raise ValueError('ROS parameters unexpectedly changed')
    checked.append('parameters.yaml')
    def launch(path, root):
        text = path.read_text().replace(str(root), '$TRIAL')
        return ET.tostring(ET.fromstring(text))
    if launch(out / 'experiment.launch', out) != launch(baseline / 'experiment.launch', baseline):
        raise ValueError('Launch topology/parameters unexpectedly changed')
    checked.append('experiment.launch (only output paths normalized)')
    current = json.loads((out / 'manifest.json').read_text())
    original = json.loads((baseline / 'manifest.json').read_text())
    for key in ('native_binaries_sha256', 'mppi_library_sha256'):
        if current[key] != original[key]:
            raise ValueError('Binary unexpectedly changed: ' + key)
    # Only the passive probe helper and its exit hook may change among the
    # modules actually imported by the prior control/sensing/plant runtime.
    excluded = {'diagnose_execution_timing.py', 'local_execution.py'}
    for name in ('simulator.py', 'mppi_node.py', 'native_reference.py', 'progress.py',
                 'classified_frame.py', 'dynamic_scenarios.py', 'shared_collision_node.py',
                 'native_collision_runtime.py', 'collab_seed_export.py', 'search_recording.py'):
        if name not in excluded and (out / 'runtime' / name).read_bytes() != (baseline / 'runtime' / name).read_bytes():
            raise ValueError('Runtime unexpectedly changed: ' + name)
        checked.append('runtime/' + name)
    return dict(passed=True, checked=checked, binary_hashes_identical=True,
        changed='watchdog removed; timestamps retained; passive buffer flushed at shutdown',
        scheduling_determinism_guaranteed=False)


def patch_trial(out, settings):
    from run_long_goal_crossing import patch_trial as parent_patch
    parent_patch(out, settings)
    disable_watchdog(out)
    evidence = check_same_settings(out)
    (Path(out) / 'same_settings_check.json').write_text(json.dumps(evidence, indent=2) + '\n')


def main():
    from run_long_goal_crossing import register_scene, SCENE
    register_scene()
    from run_execution_timing_pair import BUILD
    from run_coupled_objective import check_build, run_trial, sha
    from run_native_collision_3d import trial_kwargs
    from run_native_dynamic_fast import clean_environment
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    manifest, _ = check_build(BUILD)
    args.out.resolve().parent.mkdir(parents=True, exist_ok=True)
    with clean_environment():
        valid = run_trial(args.out.resolve(), BUILD, SCENE, 'engineering',
            11, 65., 0., 0., True, sha(BUILD / 'build_manifest.json'),
            prepare_hook=patch_trial, **trial_kwargs(manifest))
    print('WATCHDOG_OFF_REPEAT', 'valid_interface=' + str(valid), args.out, flush=True)
    if not valid:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
