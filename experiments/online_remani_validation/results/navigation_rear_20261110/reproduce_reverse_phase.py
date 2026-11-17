"""Offline actual adapter/kernel, analytic clear geometry. No ROS or device I/O."""
from pathlib import Path
import json
import sys
from types import SimpleNamespace as NS
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'src/mm_remani/scripts/mppi'))
sys.path.insert(0, str(ROOT/'src/mm_remani/test'))
if '--before' in sys.argv:
    sys.path.insert(0, str(Path(__file__).resolve().parent/'before'))
from active_arm_mppi import WholeBodyMPPI
from native_reference_adapter import NativeReferenceAdapter
from test_mppi_native_phase import goal, header, progress
from test_mppi_navigation_priority import TwoSphereRobot

class Clear:
    def query(self, spheres, times):
        return np.full(len(spheres), 10.), np.zeros(len(spheres))

def trajectory(gear):
    coefficients = np.zeros((8, 8))
    coefficients[0, -2] = .2*gear
    piece = NS(num_dim=8, num_order=7, duration=2., data=coefficients.ravel(order='F').tolist())
    msg = NS(header=header(10.1), action=1, trajectory_id=1, singul=gear, trajectory=[piece])
    msg.serialize = lambda stream: stream.write(coefficients.tobytes()+bytes([gear+1]))
    return msg

def run(gear, scale):
    config = yaml.safe_load((ROOT/'src/mm_remani/config/mppi.yaml').read_text())
    kernel = WholeBodyMPPI(TwoSphereRobot(), config['base_limits'],
        (np.full(6,-1.), np.ones(6), np.array(config['arm_velocity']), np.array(config['arm_acceleration'])),
        [], horizon=config['horizon'], step=.1, random_samples=config['random_samples'], seed=11)
    adapter = NativeReferenceAdapter(kernel, kernel, clock=lambda:20.)
    target = goal(); target.pose.position.x = .4*gear
    assert adapter.feed_goal(target,10.01) == 'accepted'
    assert adapter.feed_trajectory(trajectory(gear),10.11) == 'replace'
    assert adapter.feed_progress(progress(20., phase=.4, scale=scale),20.) == 'accepted'
    state = np.zeros(17); state[0] = .04*gear
    path = adapter.snapshot_path(0.,20.)
    answer, info = adapter.plan(state,None,None,Clear(),20.)
    assert answer is not None, info
    controls, states = answer
    return dict(native_gear=info['native_gear'], scale=scale,
        scaled_reference_speed=float(path[0,3]), inferred_speed_threshold_gear=int(-1 if path[0,3]<-.01 else 1),
        first_command_vw=controls[0,:2].tolist(), mode=info['mode'],
        first_reference_xy=path[0,:2].tolist(), endpoint_xy=path[-1,:2].tolist(),
        predicted_terminal_xyyaw=states[-1,:3].tolist(), reference_status=info['reference_status'])

if __name__=='__main__':
    result=dict(description=__doc__, before='--before' in sys.argv,
        config=yaml.safe_load((ROOT/'src/mm_remani/config/mppi.yaml').read_text()),
        cases=[run(gear,scale) for gear in (-1,1) for scale in (1.,.1,.05,.02,0.)])
    print(json.dumps(result,indent=2,sort_keys=True))
