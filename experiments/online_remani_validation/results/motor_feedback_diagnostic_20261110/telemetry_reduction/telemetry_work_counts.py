#!/usr/bin/env python3
"""Count production telemetry work with AST-only methods and fake SDK storage."""
import ast
import hashlib
import json
import math
from collections import Counter
from pathlib import Path
from types import SimpleNamespace as NS

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p/'src/piper_msgs').is_dir())
SDK_SOURCE = Path('/home/l/.local/lib/python3.8/site-packages/piper_sdk/interface/piper_interface.py')
DRIVERS = {'before': HERE/'before/piper_ctrl_single_node.py',
           'after': ROOT/'src/piper_ros/src/piper/scripts/piper_ctrl_single_node.py'}

def methods(path, class_name, selected, namespace):
    original = next(n for n in ast.parse(path.read_text()).body
                    if isinstance(n, ast.ClassDef) and n.name == class_name)
    definition = ast.ClassDef(name=class_name, bases=[], keywords=[],
        body=[n for n in original.body if isinstance(n, ast.FunctionDef) and n.name in selected],
        decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[definition], type_ignores=[]))
    exec(compile(module, str(path), 'exec'), namespace)
    return namespace[class_name]

def count_work(driver, connections):
    counts = Counter()
    class Lock:
        def __init__(self, name): self.name = name
        def __enter__(self): counts[self.name] += 1
        def __exit__(self, *args): pass
    class FPS:
        def get_fps(self, name):
            with Lock('fps_locks'): return 0.0
        def cal_average(self, *values): return sum(values)/len(values)
    class Publisher:
        def __init__(self, name, active): self.name, self.active = name, active
        def get_num_connections(self): return self.active
        def publish(self, msg): counts[self.name+'_published'] += 1
    def message(name):
        counts[name+'_constructed'] += 1
        return NS(header=NS(), pose=NS(position=NS(), orientation=NS()))
    def quaternion(*args):
        counts['quaternion_conversions'] += 1
        return (0., 0., 0., 1.)
    sdk_class = methods(SDK_SOURCE, 'C_PiperInterface',
        {'GetArmEndPoseMsgs', 'GetArmLowSpdInfoMsgs'}, {})
    sdk = sdk_class()
    sdk._C_PiperInterface__fps_counter = FPS()
    sdk._C_PiperInterface__arm_end_pose_mtx = Lock('end_pose_locks')
    sdk._C_PiperInterface__arm_end_pose = NS(end_pose=NS(
        X_axis=120000, Y_axis=-250000, Z_axis=670000,
        RX_axis=90000, RY_axis=-45000, RZ_axis=180000))
    sdk._C_PiperInterface__arm_motor_info_low_spd_mtx = Lock('motor_locks')
    sdk._C_PiperInterface__arm_motor_info_low_spd = NS(**{
        'motor_%d'%i:NS(foc_status=NS(driver_enable_status=True)) for i in range(1, 7)})
    namespace = {'math':math, 'rospy':NS(Time=NS(now=lambda:1000.)),
        'PoseStamped':lambda:message('pose'), 'PiperEulerPose':lambda:message('euler'),
        'UInt8MultiArray':lambda:message('motor'), 'quaternion_from_euler':quaternion}
    node_class = methods(driver, 'C_PiperRosNode',
        {'GetMotorEnableStates', 'PublishMotorEnableStates', 'PublishArmEndPose'}, namespace)
    node = node_class()
    node.piper = sdk
    node.end_pose_pub = Publisher('pose', connections[0])
    node.end_pose_euler_pub = Publisher('euler', connections[1])
    node.motor_enable_pub = Publisher('motor', connections[2])
    for _ in range(200):
        node.PublishMotorEnableStates()
        node.PublishArmEndPose()
    return dict(counts)

report = {'scope':'200 fake telemetry cycles; actual driver and SDK getter bodies loaded by AST; '
                  'no SDK import, CAN, ROS, or timing claim; not evidence that hardware latency is fixed',
          'source_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in list(DRIVERS.values())+[SDK_SOURCE]}, 'scenarios':{}}
for name, connections in {'none':(0,0,0), 'pose_only':(1,0,0),
                          'euler_only':(0,1,0), 'all':(1,1,1)}.items():
    report['scenarios'][name] = {label:count_work(path, connections)
                                for label, path in DRIVERS.items()}
output = HERE/'telemetry_work_counts.json'
output.write_text(json.dumps(report, indent=2)+'\n')
print(output)
