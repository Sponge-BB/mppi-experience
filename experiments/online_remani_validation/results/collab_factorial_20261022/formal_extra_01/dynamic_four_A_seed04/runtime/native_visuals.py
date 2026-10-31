"""Shared recorded native-experiment geometry and search drawing; no ROS."""
import json
import math
import numpy as np


def geometry(out):
    body = np.atleast_2d(np.loadtxt(out / 'geometry.csv', delimiter=','))
    if body.shape[1] != 3 or not np.isfinite(body).all() or np.any(body[:, 2] <= 0):
        raise ValueError('invalid recorded body; never substitute a center point')
    meta = json.loads((out / 'geometry.json').read_text())
    if not np.allclose(body, meta['body_disks_xy_r'], atol=1e-12, rtol=0):
        raise ValueError('geometry CSV/JSON mismatch')
    return body, meta


def world_disks(body, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    result = body.copy()
    result[:, :2] = body[:, :2] @ np.array([[c, s], [-s, c]]) + [x, y]
    return result


def footprint(draw, xy, scale, body, pose, meta=None, ghost=False):
    disks = world_disks(body, pose)
    # Draw the UNION fill first, then its component outlines. These are the
    # very same fixed-arm disk radii fed to MPPI, not the extra safety margin.
    boxes = []
    for x, y, radius in disks:
        px, py = xy(x, y); r = radius * scale
        boxes.append((px-r, py-r, px+r, py+r))
    if not ghost:
        for box in boxes:draw.ellipse(box, fill='#e4edf5')
    for box in boxes:draw.ellipse(box, outline='#b3c4cf' if ghost else '#7995b0', width=1)
    if ghost:return
    x, y, yaw = pose
    if meta is not None:
        c, s = math.cos(yaw), math.sin(yaw)
        def transform(a, b):return xy(x+c*a-s*b, y+s*a+c*b)
        l, w = meta['base_length']/2, meta['base_width']/2
        draw.line([transform(a,b) for a,b in [(-l,-w),(l,-w),(l,w),(-l,w),(-l,-w)]],fill='#2874a6',width=2)
        for a, b, _ in meta['arm_collision_points_xyz']:
            px,py=transform(a,b);draw.ellipse((px-2,py-2,px+2,py+2),fill='#536679')
    draw.line([xy(x,y),xy(x+.30*math.cos(yaw),y+.30*math.sin(yaw))], fill='#164a73',width=3)


class SearchReplay:
    def __init__(self, out):
        self.entries = []
        for path in sorted((out / 'search').glob('*.npz')):
            with np.load(path, allow_pickle=False) as z:
                self.entries.append((float(z['published']), path))
        self.times = [t for t, _ in self.entries]
        self.cached = None

    def at(self, t, terminal=False):
        if terminal:return None
        i = np.searchsorted(self.times, t, side='right')-1
        if i < 0 or t-self.times[i] > .25:return None
        path = self.entries[i][1]
        if self.cached is None or self.cached[0] != path:
            with np.load(path, allow_pickle=False) as z:
                self.cached = path, {k:z[k].copy() for k in z.files}
        return self.cached[1]


def draw_search(draw, xy, snapshot):
    if snapshot is None:return 0, 0
    rows = snapshot['rows']
    counts = [0, 0]
    # Infeasible curves show rejected proposals, NOT executable paths. Draw
    # kernel selection last, but keep it distinct from publication approval.
    groups = [rows[rows[:,0] == ident] for ident in np.unique(rows[:,0])]
    for group in sorted(groups, key=lambda g: bool(g[0,7])):
        feasible, selected = bool(group[0,6]), bool(group[0,7])
        counts[int(feasible)] += 1
        color = '#8761a8' if selected else '#b9cdda' if feasible else '#edd5d5'
        if len(group)>1:draw.line([xy(x,y) for x,y in group[:,2:4]],fill=color,width=2 if selected else 1)
    return counts[1], counts[0]


def draw_predictions(draw, xy, snapshot, validity):
    """Use recorded detector estimates, never simulated actor velocities."""
    if snapshot is None or 'obstacles' not in snapshot:return 0
    age=float(snapshot['obstacle_age'])
    if not np.isfinite(age) or age<0 or age>=validity-.01:return 0
    ahead=min(1.5,validity-age-.01)
    count=0
    for x,y,vx,vy,r,moving in snapshot['obstacles']:
        if not moving:continue
        start=xy(x+age*vx,y+age*vy);end=xy(x+(age+ahead)*vx,y+(age+ahead)*vy)
        draw.line([start,end],fill='#7c7c7c',width=2)
        if np.hypot(end[0]-start[0],end[1]-start[1])>4:
            angle=math.atan2(end[1]-start[1],end[0]-start[0])
            draw.line([(end[0]-7*math.cos(angle-.5),end[1]-7*math.sin(angle-.5)),end,
                       (end[0]-7*math.cos(angle+.5),end[1]-7*math.sin(angle+.5))],fill='#7c7c7c',width=2)
        count+=1
    return count
