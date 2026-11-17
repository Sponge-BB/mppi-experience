#!/usr/bin/env python3
"""Offline fixed local-plane complete-triangle split; no production edits."""
import itertools
import json
import struct
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'src/mm_remani/scripts/mppi'))
from active_arm_model import PiperArmModel, _origin

asset = json.loads((ROOT/'src/mm_remani/config/mppi_shoulder_wrist_proxy.json').read_text())
previous = json.loads((HERE/'mesh_components_audit.json').read_text())
q = np.array(previous['q_rad'])
model = PiperArmModel.from_parameters(json.loads((HERE.parent/'hardware_startup_20261108_183157/planner_parameters_at_attempt.json').read_text()))
tf = model.urdf_link_transforms(q)
urdf = ET.parse(model.urdf_path).getroot()
axes = np.array([asset['certificate']['baseline']['axis_local_link5'],
                 [-.9954197235233002, .042160942448799234, -.0858022665937532],
                 [.9944288043049222, -.04444383511520801, .09558294141181611]])
axes /= np.linalg.norm(axes, axis=1)[:, None]
world_axes = axes @ tf['link5'][:3, :3].T

def world(points, name):
    return points @ tf[name][:3, :3].T + tf[name][:3, 3]

def triangles(name):
    data = (ROOT/asset['proxies'][name]['source_mesh']).read_bytes()
    count = struct.unpack('<I', data[80:84])[0]
    assert len(data) == 84+50*count
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    pts = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].astype(float)
    origin = _origin(urdf.find("link[@name='%s']/collision/origin" % name))
    return pts @ origin[:3, :3].T + origin[:3, 3]

tri = triangles('link1')
projected = world(tri, 'link1') @ world_axes.T
wrist = np.concatenate([world(np.asarray(asset['proxies'][n]['vertices']), n)
                        for n in ('link5', 'link6')]) @ world_axes.T
wlo, whi = wrist.min(0), wrist.max(0)
results, winner = [], None
for dims in [(0,), (1,), (2,), (0, 1), (0, 2), (1, 2)]:
    parts, details = [], []
    for signs in itertools.product((-1, 1), repeat=len(dims)):
        mask = np.ones(len(tri), dtype=bool)
        for dim, sign in zip(dims, signs):
            mask &= ((tri[:, :, dim].min(1) <= 0.) if sign < 0
                     else (tri[:, :, dim].max(1) >= 0.))
        if not mask.any():
            continue
        p = projected[mask].reshape(-1, 3)
        gaps = np.maximum(p.min(0)-whi, wlo-p.max(0))-.0001
        idx = int(np.argmax(gaps))
        details.append(dict(signs=list(signs), triangle_count=int(mask.sum()),
                            best_axis=int(idx), padded_gap_m=float(gaps[idx]),
                            all_axis_padded_gaps_m=gaps.tolist()))
        parts.append(mask)
    assert np.all(np.logical_or.reduce(parts))
    result = dict(split_dimensions=list(dims), split_values=[0.]*len(dims),
                  all_parts_separated=all(d['padded_gap_m'] > 0. for d in details),
                  parts=details)
    results.append(result)
    if result['all_parts_separated'] and winner is None:
        winner = dict(result)
        winner['complete_triangle_coverage'] = int(np.logical_or.reduce(parts).sum())
        winner['original_triangle_count'] = int(len(tri))
        winner['duplicated_triangle_inclusions'] = int(sum(m.sum() for m in parts)-len(tri))
        winner['hulls'] = []
        for mask, detail in zip(parts, details):
            unique = np.unique(tri[mask].reshape(-1, 3), axis=0)
            vertices = unique[ConvexHull(unique).vertices]
            winner['hulls'].append(dict(vertices=vertices.tolist(), hull_vertex_count=len(vertices),
                                        axis_local_link5=axes[detail['best_axis']].tolist(),
                                        **detail))
        break
out = dict(q_rad=q.tolist(), scope='Diagnostic candidate only. Fixed local zero planes, each crossing triangle included whole in every touched partition.',
           original_mesh_sha256=asset['proxies']['link1']['source_mesh_sha256'],
           link1_local_bounds=[tri.min((0, 1)).tolist(), tri.max((0, 1)).tolist()],
           axes_local_link5=axes.tolist(), attempts=results, candidate=winner)
(HERE/'link1_parts_candidate.json').write_text(json.dumps(out, indent=2)+'\n')
print(json.dumps({'candidate_found': winner is not None, 'attempts': results}))
