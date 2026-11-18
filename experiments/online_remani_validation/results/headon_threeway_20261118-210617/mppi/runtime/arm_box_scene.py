"""One identical, time-driven approaching box; truth is used only by sensors/audit."""
from copy import deepcopy
import numpy as np
from arm_box_geometry import box_gaps, obstacle_points, validate_scene
from active_arm_scene import Q_START, Q_RETRACT


def scene_definition():
    return dict(schema='arm_box_scene_v1', scene='whole_body_headon_box',
        name='迎面移动长方体三组对照', time_origin='measured motion anchor',
        appearance_s=2., persistent=True, goal=[3.4, 0.], arena=[-1.5, 4.5, -2., 2.],
        q_start=Q_START.tolist(), q_retract=Q_RETRACT.tolist(),
        box=dict(id=1, center=[3.2, 0., .8], size=[.4, .4, 1.45]),
        velocity=[-.18, 0., 0.], front_gap_m=1.,
        front_gap_role='legacy schema positive field only; actual gap measured at appearance',
        placement='fixed world x=3.2m, y=0; same path/speed in all groups',
        input_contract='Only synthetic measured clouds enter planner/controller; no actor truth commands',
        scope='previous whole-body numerical ROS comparison framework; not real-vehicle control')


def place_box(model, q, base_pose, scene=None):
    scene=scene_definition() if scene is None else scene
    body=model.body_spheres(q, base_pose);box=deepcopy(scene['box'])
    return dict(box=box, velocity=scene['velocity'], base_pose=np.asarray(base_pose).tolist(),
                q=np.asarray(q).tolist(), front_gap_m=float(box['center'][0]-.5*box['size'][0]-np.max(body[:,0]+body[:,3])))


def actor_at(runtime, now):
    box=deepcopy(runtime['box'])
    elapsed=max(0., float(now)-runtime['appearance_time'])
    box['center']=(np.asarray(box['center'])+elapsed*np.asarray(runtime['velocity'])).tolist()
    return box


def straight_path_precheck(model, scene=None, runtime=None, **unused):
    scene=scene_definition() if scene is None else scene
    x=np.linspace(0.,3.4,101);q=np.asarray(scene['q_start'])
    box=scene['box'] if runtime is None else runtime['box'];values={}
    for name,y in [('straight',0.),('left',.9),('right',-.9)]:
        poses=np.c_[x,np.full(len(x),y),np.zeros(len(x))]
        bodies=model.body_spheres(np.tile(q,(len(x),1)),poses)
        gaps=np.stack([box_gaps(body,box) for body in bodies])
        values[name]=dict(fixed_min_gap_m=float(gaps[:,:47].min()),arm_min_gap_m=float(gaps[:,47:].min()))
    passed=values['straight']['fixed_min_gap_m']<0 and all(min(values[n].values())>.05 for n in ('left','right'))
    return dict(passed=bool(passed),lanes=values,scope='sampled clearance of three geometric lanes at actor appearance; not dynamic reachability proof or an online target')
