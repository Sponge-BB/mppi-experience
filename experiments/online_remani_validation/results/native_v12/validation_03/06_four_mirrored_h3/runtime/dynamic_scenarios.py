"""Declared V8-style actors for native REMANI ROS numerical experiments.

Ground truth drives the synthetic cloud and the independent collision audit.
It is never provided directly to MPPI or the native detector.
"""
import numpy as np

# x, y at appearance, vx, vy, physical radius. Same actors as V8.
A=(1.55,-1.65,0.,.26,.23)
B=(2.40,1.40,0.,-.20,.23)
C=(3.00,-2.30,0.,.17,.20)
D=(4.30,.30,-.18,0.,.20)
CASES={
    'dynamic_single':('01_single','单障碍穿越',(A,)),
    'dynamic_two':('02_two_crossing','双障碍交叉',(A,B)),
    'dynamic_three':('03_three_sequential','三障碍连续交会',(A,B,C)),
    'dynamic_four':('04_four_dense','四障碍混合交会',(A,B,C,D)),
    'dynamic_head_on':('05_head_on_only','单独迎面交会',(D,)),
    'dynamic_four_mirrored':('06_four_mirrored','四障碍镜像交会',
                             tuple((x,-y,vx,-vy,r) for x,y,vx,vy,r in (A,B,C,D))),
    'dynamic_four_late':('07_four_late','四障碍延后迎面',(A,B,C,(D[0]+.3,*D[1:]))),
    'dynamic_head_on_center':('08_head_on_center','中心线正迎面',((D[0],0.,*D[2:]),)),
}
SCENES=('empty','static','crossing',*CASES)
APPEARANCE=2.


def scene_definition(scene):
    if scene not in SCENES:raise ValueError('unknown scene: '+scene)
    if scene not in CASES:return dict(scene=scene,legacy=True)
    case,name,actors=CASES[scene]
    return dict(scene=scene,case=case,name=name,appearance_s=APPEARANCE,
                persistent=True,actors=[dict(id=i+1,x=a[0],y=a[1],vx=a[2],vy=a[3],r=a[4])
                                        for i,a in enumerate(actors)],
                source='V8 declared actor motion; center-line head-on is an additional stress case',
                time_origin='measured motion anchor',
                motion='constant velocity, prescribed trajectories; actors may enter/leave the bounded sensor arena')


def actors_at(scene, elapsed):
    """Rows are x,y,r,id,vx,vy; first 3 match legacy obstacle tuples."""
    if scene not in SCENES:raise ValueError('unknown scene: '+scene)
    if scene=='static':return [(1.8,0.,.23,1,0.,0.)] if elapsed>=4 else []
    if scene=='crossing':
        return [(1.65,-1.6+.26*(elapsed-1),.23,1,0.,.26)] if 1<=elapsed<=14 else []
    if scene not in CASES or elapsed<APPEARANCE:return []
    return [(x+(elapsed-APPEARANCE)*vx,y+(elapsed-APPEARANCE)*vy,r,i+1,vx,vy)
            for i,(x,y,vx,vy,r) in enumerate(CASES[scene][2])]


def body_clearance(state, body, actors):
    """Physical projected disk union against walls and every actor (no margin)."""
    c,s=np.cos(state[2]),np.sin(state[2])
    centers=body[:,:2]@np.array([[c,s],[-s,c]])+state[:2]
    gap=float(min(np.min(2-abs(centers[:,1])-body[:,2]),
                  np.min(np.minimum(centers[:,0]+1.5,4.5-centers[:,0])-body[:,2])))
    for x,y,r,*_ in actors:
        gap=min(gap,float(np.min(np.linalg.norm(centers-[x,y],axis=1)-body[:,2]-r)))
    return gap
