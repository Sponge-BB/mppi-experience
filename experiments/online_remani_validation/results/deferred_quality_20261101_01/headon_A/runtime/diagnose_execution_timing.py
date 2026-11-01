#!/usr/bin/env python3
"""One unchanged 200/250 ms SIL loop with in-memory stage stamps and stacks.

This is diagnostic instrumentation, not a controller fix or performance trial.
Never imports ROS or opens a socket unless main() is explicitly requested.
"""
import ast
import hashlib
import json
from pathlib import Path
import shutil
import time

HERE = Path(__file__).resolve().parent
DEFAULT_OUT = HERE / 'results/execution_timing_diagnosis_20261028_01/online_probe'


class LoopProbe:
    """Only append small timing records online; write them after the loop."""
    def __init__(self, out, ros_now):
        self.out = Path(out)
        self.ros_now = ros_now
        self.rows = []
        self.clocks = []
        self.cycle = 0

    def clock(self, message):
        self.clocks.append([time.monotonic(), message.clock.to_sec()])

    def begin(self, cycle):
        self.cycle = cycle
        # No watchdog creation, cancellation or thread join in the control loop.
        self.mark('cycle_start')

    def mark(self, stage):
        self.rows.append([self.cycle, stage, time.monotonic(), time.thread_time(), self.ros_now()])

    def close(self):
        if getattr(self, 'closed', False):
            return
        self.closed = True
        result = dict(columns=['cycle', 'stage', 'mono', 'thread_cpu_s', 'ros_time'],
                      rows=self.rows, clock_columns=['receipt_mono', 'ros_time'],
                      clocks=self.clocks, stack_threshold_s=None, watchdog_enabled=False,
                      scope='passive timestamps only; watchdog disabled; exit-only flush, no planning/safety changes')
        (self.out / 'loop_stages.json').write_text(json.dumps(result) + '\n')


def instrument_source(source):
    from native_collision_runtime import replace_once
    changes = (
        ('def configure(node):\n    lib=node.lib',
         "def configure(node):\n    from diagnose_execution_timing import LoopProbe\n"
         "    from rosgraph_msgs.msg import Clock\n"
         "    node.timing_probe=LoopProbe(node.out,rospy.get_time)\n"
         "    node.timing_clock_sub=rospy.Subscriber('/clock',Clock,node.timing_probe.clock,queue_size=1)\n"
         '    lib=node.lib'),
        ('        cycle_id+=1\n        wall=',
         '        cycle_id+=1\n        node.timing_probe.begin(cycle_id)\n        wall='),
        ('        if node.native_collision_3d and field is None:field=node.native_proposal_field\n        solve_obstacles=',
         "        node.timing_probe.mark('solve_snapshot')\n"
         '        if node.native_collision_3d and field is None:field=node.native_proposal_field\n        solve_obstacles='),
        ('        # Validate on the LATEST same-frame collision snapshot and measured',
         "        node.timing_probe.mark('planning_done')\n"
         '        # Validate on the LATEST same-frame collision snapshot and measured'),
        ('            computation_ms=(time.monotonic()-wall)*1000',
         "            node.timing_probe.mark('validation_done')\n"
         '            computation_ms=(time.monotonic()-wall)*1000'),
        ('            issued=TwistStamped();',
         "            node.timing_probe.mark('command_published')\n            issued=TwistStamped();"),
        ('        # Telemetry follows command publication. It does not authorize motion.',
         "        node.timing_probe.mark('rollout_published')\n"
         '        # Telemetry follows command publication. It does not authorize motion.'),
        ('        # Read-only identity/state linkage runs only AFTER publication and the',
         "        node.timing_probe.mark('seed_export')\n"
         '        # Read-only identity/state linkage runs only AFTER publication and the'),
        ('        if reference is not None and last_reference!=',
         "        node.timing_probe.mark('state_capture')\n        if reference is not None and last_reference!="),
        ('        if node.progress_pub is not None and reference is not None and state is not None:',
         "        node.timing_probe.mark('reference_export')\n"
         '        if node.progress_pub is not None and reference is not None and state is not None:'),
        ('        record_cycle(node,published-1000,reason)',
         "        node.timing_probe.mark('progress')\n        record_cycle(node,published-1000,reason)\n"
         "        node.timing_probe.mark('collision_record')"),
        ('        node.diag.publish(String(json.dumps(row)))',
         "        node.timing_probe.mark('feedback_and_diagnostic_build')\n"
         "        node.diag.publish(String(json.dumps(row)))\n        node.timing_probe.mark('diagnostics_published')"),
        ('        # Read-only export AFTER the command and safety decisions. It cannot',
         "        node.timing_probe.mark('execution_log')\n"
         '        # Read-only export AFTER the command and safety decisions. It cannot'),
        ('        try:rate.sleep()\n        except rospy.ROSInterruptException:break',
         "        node.timing_probe.mark('search_capture')\n        node.timing_probe.mark('sleep_enter')\n"
         "        try:rate.sleep()\n        except rospy.ROSInterruptException:break\n"
         "        node.timing_probe.mark('sleep_exit')"),
        ('    if node.search_recorder is not None:node.search_recorder.close()',
         '    node.timing_probe.close()\n    if node.search_recorder is not None:node.search_recorder.close()'),
    )
    for old, new in changes:
        source = replace_once(source, old, new)
    ast.parse(source)
    return source


def patch_trial(out, settings):
    from execution_timing_runtime import patch_trial as timing_patch
    timing_patch(out, 200., .250)
    out = Path(out)
    path = out / 'runtime/local_execution.py'
    path.write_text(instrument_source(path.read_text()))
    shutil.copy2(Path(__file__), out / 'runtime' / Path(__file__).name)
    path = out / 'manifest.json'
    manifest = json.loads(path.read_text())
    for name in ('local_execution.py', Path(__file__).name):
        manifest['experiment_sources_sha256'][name] = hashlib.sha256((out / 'runtime' / name).read_bytes()).hexdigest()
    manifest['execution_lag_diagnosis'] = dict(stages_in_memory=True, stack_threshold_s=.3,
        parameters_changed=False, recorded_clock_receipts=True,
        scope='one same-seed numerical SIL diagnostic rerun; NOT a success-rate or speed comparison')
    path.write_text(json.dumps(manifest, indent=2) + '\n')


def main():
    import argparse
    from run_execution_timing_pair import BUILD
    from run_coupled_objective import run_trial, check_build, sha
    from run_native_collision_3d import trial_kwargs
    from run_native_dynamic_fast import clean_environment
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    manifest, _ = check_build(BUILD)
    args.out.resolve().parent.mkdir(parents=True, exist_ok=True)
    with clean_environment():
        valid = run_trial(args.out.resolve(), BUILD, 'dynamic_head_on_center', 'engineering',
            11, 55., 0., 0., True, sha(BUILD / 'build_manifest.json'),
            prepare_hook=patch_trial, **trial_kwargs(manifest))
    print('DIAGNOSTIC_RUN', 'valid_interface=' + str(valid), args.out, flush=True)
    if not valid:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
