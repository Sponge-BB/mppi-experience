"""Configure execution timing only in new paired trial snapshots."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from native_collision_runtime import replace_once
from run_native_dynamic_fast import patch_trial as patch_fast_trial


def patch_trial(out, deadline_ms, command_lease_s):
    """Use identical runtime sources; vary only private launch parameters."""
    out = Path(out)
    patch_fast_trial(out, True)
    runtime = out / 'runtime'
    path = runtime / 'local_execution.py'
    text = path.read_text()
    text = replace_once(text, '    lib=node.lib', '''    lib=node.lib
    node.execution_deadline_ms=float(rospy.get_param('~execution_deadline_ms',50.))
    node.command_lease_s=float(rospy.get_param('~command_lease_s',.075))
    lib.online_set_execution_lease.argtypes=[ct.c_void_p,ct.c_double]
    lib.online_set_execution_lease.restype=None
    lib.online_set_execution_lease(node.handle,node.command_lease_s)''')
    text = replace_once(text,
        '                    # Leave room for publication and the final 50ms deadline.',
        '                    # Leave 5ms for publication before the configured deadline.')
    text = replace_once(text,
        '                    if rank>0 and time.monotonic()-wall>.045:break',
        '                    if rank>0 and (time.monotonic()-wall)*1000>node.execution_deadline_ms-5.:break')
    text = replace_once(text, "            if computation_ms>50:reason='deadline_miss'",
        "            validated_proposal=result[:2].tolist() if reason=='ok' else None\n"
        "            if computation_ms>node.execution_deadline_ms:reason='deadline_miss'")
    text = replace_once(text,
        '        # unchanged 50ms deadline decision, outside the safety critical lock.',
        '        # configured deadline decision, outside the safety critical lock.')
    text = replace_once(text, "            'applied_v':result[0],'applied_w':result[1],'nominal_v':result[2],'nominal_w':result[3],",
        "            'validated_proposal':validated_proposal,\n"
        "            'applied_v':result[0],'applied_w':result[1],'nominal_v':result[2],'nominal_w':result[3],")
    text = replace_once(text, "            'lease_s':.075,'command':result[:2].tolist(),'validation':validation.tolist(),",
        "            'lease_s':node.command_lease_s,'command':result[:2].tolist(),'validation':validation.tolist(),")
    path.write_text(text)
    path = runtime / 'simulator.py'
    text = replace_once(path.read_text(),
        '        self.command_lease=.075 if self.local_execution_owner else .3',
        "        self.command_lease=float(rospy.get_param('~command_lease_s',.075 if self.local_execution_owner else .3))")
    path.write_text(text)

    tree = ET.parse(out / 'experiment.launch')
    nodes = {node.get('name'): node for node in tree.getroot().findall('node')}
    for name in ('base_mppi', 'algorithm_simulator'):
        ET.SubElement(nodes[name], 'param', name='command_lease_s', value=str(float(command_lease_s)), type='double')
    ET.SubElement(nodes['base_mppi'], 'param', name='execution_deadline_ms', value=str(float(deadline_ms)), type='double')
    tree.write(out / 'experiment.launch', encoding='unicode')

    path = out / 'manifest.json'
    manifest = json.loads(path.read_text())
    for name in ('local_execution.py', 'simulator.py'):
        manifest['experiment_sources_sha256'][name] = hashlib.sha256((runtime / name).read_bytes()).hexdigest()
    unchanged = float(deadline_ms) == 50. and float(command_lease_s) == .075
    manifest['native_dynamic_fast']['native_and_execution_budgets_unchanged'] = unchanged
    manifest['execution_timing_pair'] = dict(group='current' if unchanged else 'matched',
        callback_deadline_s=float(deadline_ms) / 1000., deadline_ms=float(deadline_ms),
        command_lease_s=float(command_lease_s), validation_reserve_ms=5.,
        native_solver_budgets_unchanged=True, same_runtime_sources=True,
        cpp_execution_lease_matches_simulator=True,
        unchanged='native geometry, search, prediction, speed, latest-state validation and braking checks',
        scope='paired head-on numerical ROS closed loop; not Gazebo or hardware')
    path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n')
