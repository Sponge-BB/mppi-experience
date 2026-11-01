#!/usr/bin/env python3
"""Prepare a private, state-only low-speed native-planning SIL probe.

This module neither starts ROS nor changes the workspace simulator/planner.
The generated private runtime deliberately reports a constant pose with a
nonzero reverse twist.  It is a directed input fixture, NOT a kinematically
consistent moving plant, an executed command, or a navigation-performance test.
The real ROS odometry/sensor/goal interfaces and native search/MINCO/backend are
used; no planner member variable or extra velocity publisher is introduced.
"""
import argparse
import ast
import hashlib
import json
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent
FIXTURE_STATE = (0., 0., 0., -.04, 0.)
FIXTURE_GOAL = (-.8, 0., 0.)
MINIMUM_RUNTIME_S = 14.5  # Existing run_trial audits the graph after 13 s.
POST_REFERENCE_S = .5
FIXTURE_TIMEOUT_S = 18.
SCHEMA = 'directed_low_speed_state_probe_v1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise ValueError('Missing/ambiguous private simulator anchor: ' + before[:100])
    return text.replace(before, after, 1)


def transform_simulator(original):
    """Only transform a NEW trial's copied simulator; fail on source drift."""
    text = replace_once(original, '        self.s=np.zeros(5);',
        '        self.s=np.array([0.,0.,0.,-.04,0.],dtype=np.float64);')
    text = replace_once(text,
        '        self.lock=threading.Lock();self.events=[];self.raw_trajs=[];self.diagnostics=[];self.rows=[];self.obstacle_rows=[]',
        '''        self.lock=threading.Lock();self.events=[];self.raw_trajs=[];self.diagnostics=[];self.rows=[];self.obstacle_rows=[]
        self.directed_first_reference_time=None
        self.events.append({'t':0.,'kind':'directed_state_fixture',
            'schema':'directed_low_speed_state_probe_v1','state':[0.,0.,0.,-.04,0.],
            'goal':[-.8,0.,0.],'commands_integrated':False,
            'semantics':'constant-pose nonzero-twist directed ROS input; NOT physical motion or closed-loop performance'})''')
    text = replace_once(text,
        '        if m.action==1 and m.trajectory_id==1:self.traj_count+=1',
        '''        if m.action==1 and m.trajectory_id==1:
            self.traj_count+=1
            if self.directed_first_reference_time is None:
                self.directed_first_reference_time=self.t
                self.events.append({'t':self.t,'kind':'directed_first_native_reference',
                    'semantics':'a native reference was observed, NOT an executed trajectory'})''')
    first = text.find('    def integrate(self,dt):\n')
    last = text.find('    def publish_state(self):\n')
    if first < 0 or last <= first or text.count('    def integrate(self,dt):\n') != 1 or text.count('    def publish_state(self):\n') != 1:
        raise ValueError('Missing/ambiguous private plant method boundaries')
    text = text[:first] + '''    def integrate(self,dt):
        # STATE-ONLY DIRECTED INPUT: commands are recorded, never integrated.
        # Keeping pose/twist fixed makes the planning boundary deterministic;
        # the inconsistent pose derivative is explicit, never physical evidence.
        self.s[:]=[0.,0.,0.,-.04,0.]

'''+text[last:]
    text = replace_once(text, 'goal.pose.position.x=3.;goal.pose.orientation.w=1.;',
        'goal.pose.position.x=-.8;goal.pose.orientation.w=1.;')
    text = replace_once(text, '            self.record()\n', '''            self.record()
            if self.t>=14.5 and self.directed_first_reference_time is not None and self.t>=self.directed_first_reference_time+.5:
                reason='probe_complete';break
            if self.t>=18.:
                reason='probe_timeout';break
''')
    old_goal = '            at_goal=np.linalg.norm(self.s[:2]-[3.,0.])<self.limits[\'goal_pos\'] and abs(self.s[2])<self.limits[\'goal_yaw\'] and abs(self.s[3])<self.limits[\'goal_speed\']'
    text = replace_once(text, old_goal,
        '            at_goal=False # This state-only probe never claims physical goal reach.')
    text = replace_once(text,
        "        (self.out/'result.json').write_text(json.dumps(result,indent=2)+'\\n')",
        '''        result.update(scope='state-only directed ROS input probe; NOT physical motion, closed-loop navigation or performance',
            directed_low_speed={'schema':'directed_low_speed_state_probe_v1',
                'state':[0.,0.,0.,-.04,0.],'goal':[-.8,0.,0.],
                'commands_integrated':False,'first_native_reference_time':self.directed_first_reference_time,
                'minimum_runtime_s':14.5,'post_reference_s':.5,'timeout_s':18.,
                'native_map_replay_frozen':False},closed_loop_performance=False)
        (self.out/'result.json').write_text(json.dumps(result,indent=2)+'\\n')''')
    # Syntactic verification only: no imports or node/model execution.
    ast.parse(text, filename='private_runtime/simulator.py')
    if text.count("rospy.Publisher('/sim/cmd_vel'") != original.count("rospy.Publisher('/sim/cmd_vel'"):
        raise ValueError('A private fixture must not add velocity publishers')
    if text.count("rospy.Publisher('/odom'") != original.count("rospy.Publisher('/odom'"):
        raise ValueError('The original plant must remain the sole odometry source')
    return text


def fixture_geometry(body, margin):
    """Independent fixed-pose disk-union clearance to declared empty-scene walls."""
    if not isinstance(margin, (int, float)) or not math.isfinite(margin) or abs(margin-.1)>1e-12:
        raise ValueError('The directed probe retains the recorded 0.10 m margin')
    if not body or any(len(row)!=3 or not all(math.isfinite(v) for v in row) or row[2]<=0 for row in body):
        raise ValueError('Invalid projected body geometry')
    def clearance(x, y):
        return min(min(2.-abs(y+by)-radius, x+bx+1.5-radius,
                       4.5-x-bx-radius) for bx,by,radius in body)
    initial=clearance(*FIXTURE_STATE[:2]);goal=clearance(*FIXTURE_GOAL[:2])
    # This is a preparation sanity bound, NOT a replay of the native 3D map.
    # Leave more than two 5cm cell diagonals before the unchanged hard margin.
    reserve=2.*math.sqrt(2)*.05
    if min(initial,goal)<=margin+reserve:
        raise ValueError('Fixture start/goal too close to declared walls')
    return dict(initial_world_physical_m=initial,goal_world_physical_m=goal,
        conservative_grid_sanity_reserve_m=reserve,margin_m=margin,
        online_frozen_field_endpoint_validation_required=True,
        native_3d_replay_or_safety_certificate=False)


def patch_prepared_trial(out):
    """Patch only an existing freshly prepared isolated trial, once."""
    out=Path(out).resolve();runtime=out/'runtime';manifest_path=out/'manifest.json'
    if out in (HERE,HERE.parents[1]) or runtime.resolve().parent!=out:
        raise ValueError('Expected a private trial directory')
    manifest=json.loads(manifest_path.read_text())
    if manifest.get('directed_low_speed_probe'):
        raise ValueError('Directed fixture already applied; refusing a second rewrite')
    if manifest.get('scene')!='empty' or manifest.get('mode')!='mppi' or not manifest.get('local_execution_owner'):
        raise ValueError('Directed fixture requires the empty MPPI-owned isolated scene')
    path=runtime/'simulator.py'
    expected=manifest.get('experiment_sources_sha256',{}).get('simulator.py')
    if not expected or path.is_symlink() or sha(path)!=expected:
        raise ValueError('Private runtime simulator is missing, changed, or not a regular snapshot')
    original=path.read_text();transformed=transform_simulator(original)
    body=[]
    for line in (out/'geometry.csv').read_text().splitlines():
        if line.strip():body.append([float(value) for value in line.split(',')])
    limits=json.loads((out/'limits.json').read_text())
    geometry=fixture_geometry(body,limits['margin'])
    fixture=dict(schema=SCHEMA,state=list(FIXTURE_STATE),goal=list(FIXTURE_GOAL),
        minimum_runtime_s=MINIMUM_RUNTIME_S,post_reference_s=POST_REFERENCE_S,
        timeout_s=FIXTURE_TIMEOUT_S,
        commands_integrated=False,additional_velocity_publishers=False,
        input_source='existing simulator /odom, TF, fixed-arm joints and synthetic point clouds',
        semantics='deliberately constant pose/nonzero twist; state-only directed input, not a consistent physical plant',
        paired_same_complete_native_input=False,native_map_replay_frozen=False,
        projected_geometry_sanity=geometry,preparer_sha256=sha(Path(__file__)),
        original_private_simulator_sha256=expected,
        modified_private_simulator_sha256=hashlib.sha256(transformed.encode()).hexdigest())
    # All validation above precedes the one private generated-runtime write.
    path.write_text(transformed)
    manifest['experiment_sources_sha256']['simulator.py']=sha(path)
    manifest['directed_low_speed_probe']=fixture
    settings=manifest.get('coupling_experiment',{})
    settings.update(genuine_online_closed_loop=False,directed_low_speed_probe=True,
        experiment_scope='native ROS state-only low-speed input probe; NOT physical motion or performance',
        commanded_motion_integrated=False,final_goal_unchanged=False,
        fixture_state=list(FIXTURE_STATE),fixture_goal=list(FIXTURE_GOAL))
    manifest['coupling_experiment']=settings
    params=out/'coupling_parameters.json';params.write_text(json.dumps(settings,indent=2)+'\n')
    manifest['parameters_sha256']=sha(params)
    manifest_path.write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
    (out/'directed_low_speed_fixture.json').write_text(json.dumps(fixture,indent=2,ensure_ascii=False)+'\n')
    return settings


def apply_fixture(out, settings, config=None):
    """Root runner preparation hook; mutate its returned settings coherently.

    Call only after the regular isolated prepare_trial has created a NEW trial.
    The root runner subsequently records signed OFF/ON and capture environment
    flags. This hook deliberately contains no signed switch or policy change.
    """
    if not isinstance(settings,dict):
        raise ValueError('Expected the root runner settings dictionary')
    if config not in (None,{}):
        raise ValueError('This single-factor probe uses only the declared rear-goal fixture')
    prepared=patch_prepared_trial(out)
    settings.clear();settings.update(prepared)
    return settings


def prepare_trial(out, build, scene, group, seed, duration, execution_weight=1.,
                  handover_weight=1., search=False):
    """Drop-in preparer for a NEW root-owned runner; does not launch anything.

    The normal experiment runner should use apply_fixture as its optional
    preparation hook. This separate preparation-only entry point requires an
    explicitly injected original preparer; no persistent module is edited.
    """
    if scene!='empty' or group!='coupled' or not 8.<=duration<=25.:
        raise ValueError('Directed probe requires empty/coupled and duration in [8,25] s')
    manifest=json.loads((Path(build)/'build_manifest.json').read_text())
    if not manifest.get('state_contract_experiment') or manifest.get('state_contract_prepared_only'):
        raise ValueError('Requires the already compiled isolated state-contract build')
    if ORIGINAL_PREPARER is None:
        raise ValueError('Bind the existing original preparer explicitly before running the directed fixture')
    settings=ORIGINAL_PREPARER(out,build,scene,group,seed,duration,execution_weight,handover_weight,search)
    return apply_fixture(out,settings)


# Explicit dependency injection avoids ROS imports during helper/unit imports
# and accidental recursive preparation when a root-owned runner binds the hook.
ORIGINAL_PREPARER = None


def set_original_preparer(function):
    global ORIGINAL_PREPARER
    if not callable(function) or function is prepare_trial:
        raise ValueError('Expected the original coupling trial preparer')
    ORIGINAL_PREPARER=function


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=18.)
    args=parser.parse_args()
    from run_coupled_objective import prepare_trial as original
    set_original_preparer(original)
    prepare_trial(args.out,args.build,'empty','coupled',11,args.duration)
    print('PREPARED_DIRECTED_STATE_ONLY_PROBE',args.out.resolve())
