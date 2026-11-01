"""Small, identical timing records for NEW sphere-comparison trial copies.

Never patch shared runtime sources, compiled code, or a past result. The native
3-D inputs and truth monitor remain live. Only full input-replay artifacts and
per-odometry state JSON are omitted; MPPI keeps only compact cycle timings.
Signed low-speed processing
must remain enabled; its planner startup requires STATE_CONTRACT_CAPTURE=1.
"""
import ast
import hashlib
import json
from pathlib import Path
import shutil
import threading
import xml.etree.ElementTree as ET


CYCLE_KEYS = (
    'schema_version', 'event', 'cycle_id', 'solve_ros_time',
    'validation_snapshot_ros_time', 'validation_ros_time', 'command_ros_time',
    'online_step_entry_ros_time', 'online_step_entry_monotonic',
    'command_publish_before_ros_time', 'command_publish_before_monotonic',
    'command_publish_after_ros_time', 'command_publish_after_monotonic',
    'command_header_ros_time', 'cycle_start_monotonic',
    'online_step_invoked', 'online_step_succeeded', 'kernel_code',
    'goal_generation', 'guide_epoch', 'guide_revision', 'reference_array_sha256',
    'validation_goal_generation', 'validation_revision', 'validation_invoked',
    'validation_code', 'validation_attempts', 'validated_command_published',
    'command_v', 'command_w', 'reason', 'first_kernel_cycle_for_reference',
    'first_validated_command_cycle_for_reference',
)


class LightCycleCapture:
    """Small in-memory cycle records; one exit flush, no per-cycle file I/O.

    This sink does not require odometry hashing to be enabled on the node.
    Existing publication timestamps and loop-cycle IDs remain available for
    reports; raw states, identities and full reference messages are not kept.
    """
    def __init__(self, path):
        self.path = Path(path)
        self.enabled = True
        self.rows = []
        self.closed = False
        self.lock = threading.Lock()

    def write(self, record):
        from state_contract_records import record_with_validity
        row = {key: record[key] for key in CYCLE_KEYS if key in record}
        row['scope'] = 'compact cycle timing/publication only; not exact-state replay'
        row = record_with_validity(row)
        with self.lock:
            if self.closed:
                return False
            self.rows.append(row)
        return True

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            with self.path.open('w') as stream:
                for row in self.rows:
                    stream.write(json.dumps(row, separators=(',', ':'), allow_nan=False)+'\n')


class BodyOnlySnapshotRecorder:
    """Keep actual exported body evidence, but do not save large ESDF maps."""
    def __init__(self, out):
        self.out = Path(out)
        self.root = self.out / 'native3d_snapshots'
        self.root.mkdir(exist_ok=False)
        self.closed = False
        self.stats = dict(frames=0, parity=0, errors=[], body_counts=[],
                          maps_recorded=False, complete_input_replay_available=False)
        self.body_digest = None

    def submit(self, raw, payload=None, parity=False):
        if self.closed:
            return
        if parity:
            self.stats['parity'] += 1
            return
        self.stats['frames'] += 1
        try:
            nb = int(payload[12])
            if nb not in self.stats['body_counts']:
                self.stats['body_counts'].append(nb)
            # Fixed arm and fixed collision model are immutable in these runs.
            # No per-frame field conversion/hash/serialization is necessary.
            if self.body_digest is not None:
                return
            body = payload[32:32+7*nb].reshape((-1, 7))
            self.body_digest = hashlib.sha256(body.tobytes()).hexdigest()
            path = self.out / 'native_collision_3d_body.json'
            path.write_text(json.dumps(dict(schema_version=1,
                source='/sim/native_collision_3d', header=payload[:32].tolist(),
                body=body.tolist(), body_sha256=self.body_digest,
                scope='actual native exported MM spheres; full map recording disabled only'),
                indent=2)+'\n')
        except Exception as exc:
            self.stats['errors'].append(type(exc).__name__ + ': ' + str(exc))

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stats['scope'] = 'timing-only capture; live snapshot input and truth monitor unchanged'
        (self.root / 'recording.json').write_text(json.dumps(self.stats, indent=2)+'\n')


def _replace(source, old, new):
    if source.count(old) != 1:
        raise ValueError('timing capture patch anchor missing or ambiguous: ' + old[:100])
    return source.replace(old, new, 1)


def patch_sources(sources):
    """Return only telemetry changes; testable without importing ROS."""
    changed = dict(sources)
    changed['native_collision_runtime.py'] = _replace(
        sources['native_collision_runtime.py'],
        '    sim.native_snapshot_recorder = SnapshotRecorder(sim.out)\n',
        '    from timing_capture_runtime import BodyOnlySnapshotRecorder\n'
        '    sim.native_snapshot_recorder = BodyOnlySnapshotRecorder(sim.out)\n')
    changed['shared_collision_node.py'] = _replace(
        sources['shared_collision_node.py'],
        "        (self.out/'collision_snapshots').mkdir(exist_ok=False)\n", '')
    changed['shared_collision_node.py'] = _replace(
        changed['shared_collision_node.py'],
        '            # Save the exact transport bytes once. Consumers record sequence IDs.\n'
        "            target=self.out/'collision_snapshots'/('%05d.npz'%self.sequence)\n"
        '            np.savez_compressed(target,payload=payload)\n',
        '            # Timing trial: publish the identical payload, omit input-replay dumps.\n')
    changed['shared_collision_node.py'] = _replace(
        changed['shared_collision_node.py'],
        'payload_sha256=hashlib.sha256(payload.tobytes()).hexdigest(),healthy=healthy,**stats)',
        'healthy=healthy,**stats)')
    changed['search_recording.py'] = _replace(
        sources['search_recording.py'], '        self.next_t = t + .195\n',
        '        self.next_t = t + .995 # telemetry cadence only; kernel sampling unchanged\n')
    changed['search_recording.py'] = _replace(
        changed['search_recording.py'], '        self.stats.update(hz=5, columns=',
        '        self.stats.update(hz=1, columns=')
    changed['local_execution.py'] = _replace(
        sources['local_execution.py'],
        "    node.state_contract_cycles=JsonlCapture(node.out/'state_mppi_cycles.jsonl',\n"
        "        enabled=getattr(node,'state_contract_enabled',capture_enabled()))\n",
        '    from timing_capture_runtime import LightCycleCapture\n'
        "    node.state_contract_cycles=LightCycleCapture(node.out/'state_mppi_cycles.jsonl')\n")
    changed['local_execution.py'] = _replace(
        changed['local_execution.py'],
        '                    validation_code=node.lib.online_validate_execution(',
        '                    validation_wall_start=time.monotonic()\n'
        '                    validation_cpu_start=time.thread_time()\n'
        '                    validation_code=node.lib.online_validate_execution(')
    changed['local_execution.py'] = _replace(
        changed['local_execution.py'],
        "                    validation_attempts.append({'candidate':rank,'code':validation_code})",
        "                    validation_attempts.append({'candidate':rank,'code':validation_code,\n"
        "                        'wall_ms':1000*(time.monotonic()-validation_wall_start),\n"
        "                        'thread_cpu_ms':1000*(time.thread_time()-validation_cpu_start)})")
    for source in changed.values():
        ast.parse(source)
    return changed


def patch_trial(out):
    """Call last in the prepare hook; apply identically to both geometries."""
    out = Path(out).resolve()
    runtime = out / 'runtime'
    names = ('native_collision_runtime.py', 'shared_collision_node.py',
             'search_recording.py', 'local_execution.py')
    sources = {name: (runtime / name).read_text() for name in names}
    for name, source in patch_sources(sources).items():
        (runtime / name).write_text(source)
    shutil.copy2(Path(__file__), runtime / Path(__file__).name)

    tree = ET.parse(out / 'experiment.launch')
    root = tree.getroot()
    nodes = {node.get('name'): node for node in root.findall('node')}
    overrides = {
        'remani_planner_node': {'COUPLING_DIAGNOSTIC_DIR': '',
            'STATE_CONTRACT_CAPTURE': '1', 'STATE_CONTRACT_SIGNED_LOW_SPEED': '1'},
        'base_mppi': {'STATE_CONTRACT_CAPTURE': '0'},
        'dual_timescale_observer': {'STATE_CONTRACT_CAPTURE': '0'},
    }
    for name, values in overrides.items():
        node = nodes[name]
        for key, value in values.items():
            existing = node.findall("env[@name='%s']" % key)
            if len(existing) > 1:
                raise ValueError('ambiguous node environment: ' + name + '/' + key)
            if existing:
                existing[0].set('value', value)
            else:
                ET.SubElement(node, 'env', name=key, value=value)
    tree.write(out / 'experiment.launch', encoding='unicode')

    path = out / 'manifest.json'
    manifest = json.loads(path.read_text())
    manifest['timing_only_capture'] = dict(enabled=True,
        common_to_both_collision_models=True, native_snapshot_transport_unchanged=True,
        actual_body_export_retained=True, independent_truth_monitor_retained=True,
        complete_input_replay_available=False, native_map_disk_recording=False,
        shared_2d_input_disk_recording=False, per_mppi_state_json=False,
        compact_mppi_cycle_timings=True, compact_mppi_cycle_flush='exit only',
        compact_mppi_cycle_keys=list(CYCLE_KEYS), mppi_raw_odom_identity_capture=False,
        per_odom_state_json=False, planner_state_contract_light_trace=True,
        planner_signed_low_speed_unchanged=True, native_diagnostic_full_snapshots=False,
        search_telemetry_hz=1, kernel_candidate_sampling_unchanged=True,
        validation_attempt_wall_and_thread_cpu_ms=True,
        retained=['state.csv', 'solver_trace.jsonl', 'dual_timeline.jsonl',
                  'resource monitor', 'mppi_diagnostics.json', 'execution_rollouts.jsonl',
                  'state_mppi_cycles.jsonl (compact timings, no raw states/identities)',
                  'actual references', 'actual search traces at 1 Hz',
                  'native_collision_3d_body.json', 'passive loop_stages.json'],
        scope='new isolated timing comparison; not full-state replay or hardware')
    for name in (*names, Path(__file__).name):
        manifest['experiment_sources_sha256'][name] = hashlib.sha256(
            (runtime / name).read_bytes()).hexdigest()
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')

