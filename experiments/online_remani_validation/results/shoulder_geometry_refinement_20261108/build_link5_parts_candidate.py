#!/usr/bin/env python3
"""Offline link5 complete-triangle partitions; diagnostic directions explicit."""
import contextlib
import io
import json
import runpy
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull

HERE = Path(__file__).resolve().parent
with contextlib.redirect_stdout(io.StringIO()):
    ns = runpy.run_path(str(HERE/'build_link1_parts_candidate.py'))
tri = ns['triangles']('link5')
world, tf, asset = ns['world'], ns['tf'], ns['asset']
principal_link1 = np.eye(3) @ tf['link1'][:3, :3].T @ tf['link5'][:3, :3]
axes = np.concatenate([ns['axes'], np.eye(3), principal_link1,
                       [[.995597728, .001585644, .093715792]]])
axes /= np.linalg.norm(axes, axis=1)[:, None]
labels = ['existing_%d' % i for i in range(3)] + ['link5_'+x for x in 'xyz'] + ['link1_'+x for x in 'xyz'] + ['today_FCL_nearest_DIAGNOSTIC_ONLY']
world_axes = axes @ tf['link5'][:3, :3].T
projected = world(tri, 'link5') @ world_axes.T
shoulder = np.concatenate([world(np.asarray(asset['proxies'][n]['vertices']), n)
                           for n in ('arm_base', 'link1')]) @ world_axes.T
alo, ahi = shoulder.min(0), shoulder.max(0)
results, winner = [], None
for value in [.03, .04, .05]:
    masks = [tri[:, :, 1].min(1) <= value, tri[:, :, 1].max(1) >= value]
    assert np.all(masks[0] | masks[1])
    details = []
    for name, mask in zip(['low', 'high'], masks):
        p = projected[mask].reshape(-1, len(axes))
        gaps = np.maximum(alo-p.max(0), p.min(0)-ahi)-.0001
        idx = int(np.argmax(gaps[:-1]))
        details.append(dict(name=name, triangle_count=int(mask.sum()),
                            best_geometry_axis=labels[idx],
                            best_geometry_padded_gap_m=float(gaps[idx]),
                            all_axis_padded_gaps_m=dict(zip(labels, gaps.tolist()))))
    result = dict(split_frame='link5', split_axis=1, split_value_m=value, parts=details,
                  geometry_axes_all_separated=all(d['best_geometry_padded_gap_m'] > 0. for d in details),
                  diagnostic_axis_all_separated=all(d['all_axis_padded_gaps_m'][labels[-1]] > 0. for d in details))
    results.append(result)
    if result['geometry_axes_all_separated'] and winner is None:
        winner = dict(result, original_triangle_count=len(tri),
                      covered_triangle_count=int((masks[0] | masks[1]).sum()),
                      duplicated_crossing_triangles=int((masks[0] & masks[1]).sum()),
                      crossing_triangles='each complete crossing triangle included in both pieces', hulls=[])
        for mask, detail in zip(masks, details):
            v = np.unique(tri[mask].reshape(-1, 3), axis=0)
            v = v[ConvexHull(v).vertices]
            winner['hulls'].append(dict(coordinate_frame='link5', vertices=v.tolist(),
                                        hull_vertex_count=len(v), **detail))
out = dict(scope='Offline candidate only; final FCL-derived axis is diagnostic, not a proposed deployed pose-specific direction.',
           q_rad=ns['q'].tolist(), source_mesh=asset['proxies']['link5']['source_mesh'],
           source_mesh_sha256=asset['proxies']['link5']['source_mesh_sha256'],
           axes_at_this_q_local_link5=dict(zip(labels, axes.tolist())),
           attempts=results, candidate=winner)
(HERE/'link5_parts_geometry_axes_candidate.json').write_text(json.dumps(out, indent=2)+'\n')
print(json.dumps({'candidate_found': winner is not None, 'attempts': results}))

# Keep the simplest fixed geometry independently of today's direction success.
value = .03
masks = [tri[:, :, 1].min(1) <= value, tri[:, :, 1].max(1) >= value]
parts = {}
for name, mask in zip(['low', 'high'], masks):
    unique = np.unique(tri[mask].reshape(-1, 3), axis=0)
    hull = ConvexHull(unique)
    vertices = unique[hull.vertices]
    parts[name] = dict(coordinate_frame='link5', vertices=vertices.tolist(),
                       hull_vertex_count=len(vertices), triangle_count=int(mask.sum()),
                       face_normals=hull.equations[:, :3].tolist())
item = asset['proxies']['link5']
geometry = dict(status='geometry_candidate_only_needs_generic_direction_fallback',
                source_mesh=item['source_mesh'], source_mesh_sha256=item['source_mesh_sha256'],
                collision_origin=item['collision_origin'], mesh_scale=item['mesh_scale'],
                collision_origin_and_scene_transforms_applied=True,
                split=dict(frame='link5', axis=1, value=value,
                           crossing_triangles='complete crossing triangle included in both pieces'),
                original_triangle_count=len(tri), covered_triangle_count=int((masks[0] | masks[1]).sum()),
                duplicated_crossing_triangles=int((masks[0] & masks[1]).sum()), parts=parts)
(HERE/'wrist_geometry_candidate.json').write_text(json.dumps(geometry, indent=2)+'\n')
shape_axes = np.array(parts['high']['face_normals'])
shape_world = shape_axes @ tf['link5'][:3, :3].T
shoulder_world = np.concatenate([world(np.asarray(asset['proxies'][n]['vertices']), n)
                                 for n in ('arm_base', 'link1')])
high_world = world(np.array(parts['high']['vertices']), 'link5')
best = (-float('inf'), None)
for start in range(0, len(shape_axes), 64):
    a = shoulder_world @ shape_world[start:start+64].T
    b = high_world @ shape_world[start:start+64].T
    gaps = np.maximum(a.min(0)-b.max(0), b.min(0)-a.max(0))-.0001
    i = int(np.argmax(gaps))
    if gaps[i] > best[0]:
        best = (float(gaps[i]), start+i)
report = dict(high_part_best_shape_face_padded_gap_m=best[0], face_index=best[1],
              shape_axis_local_link5=shape_axes[best[1]].tolist(),
              note='Axis is a face normal of an unchanged complete-triangle hull; testing all such normals is generic but runtime cost not assessed here.')
(HERE/'wrist_shape_face_direction.json').write_text(json.dumps(report, indent=2)+'\n')
