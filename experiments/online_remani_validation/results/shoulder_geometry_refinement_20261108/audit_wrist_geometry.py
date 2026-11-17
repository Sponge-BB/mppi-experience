#!/usr/bin/env python3
"""Independent half-space coverage check of every original triangle."""
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

from build_wrist_geometry import source_triangles, build

HERE = Path(__file__).resolve().parent
candidate = json.loads((HERE/'wrist_geometry_candidate.json').read_text())
item, triangles = source_triangles()
fresh = build()
assert candidate == fresh, 'Stored candidate differs from posture-free reconstruction'
assert candidate['source_mesh_sha256'] == item['source_mesh_sha256']
masks = {'low': triangles[:, :, 1].min(1) <= .03,
         'high': triangles[:, :, 1].max(1) >= .03}
covered = np.zeros(len(triangles), dtype=bool)
report = dict(scope='Offline shape coverage only; no physical calibration, ROS, or execution.',
              source_hash_verified=True, generation_without_q_exact_match=True,
              source_mesh_sha256=item['source_mesh_sha256'],
              original_triangle_count=len(triangles), parts={})
for name in ('low', 'high'):
    part = candidate['parts'][name]
    vertices = np.asarray(part['vertices'])
    hull = ConvexHull(vertices)
    normals, offsets = hull.equations[:, :3], hull.equations[:, 3]
    indices = np.flatnonzero(masks[name])
    violations = []
    for start in range(0, len(indices), 128):
        chunk = triangles[indices[start:start+128]]
        residual = chunk.reshape(-1, 3) @ normals.T + offsets
        violations.extend(residual.max(1).reshape(-1, 3).max(1).tolist())
    violations = np.array(violations)
    contained = violations <= 1e-10
    covered[indices[contained]] = True
    assert contained.all(), 'Complete original triangle is outside its assigned hull'
    assert len(indices) == part['triangle_count']
    report['parts'][name] = dict(triangles_checked=len(indices), hull_vertex_count=len(vertices),
                                max_vertex_face_residual_m=float(violations.max()),
                                assigned_complete_triangles_contained=bool(contained.all()),
                                assigned_triangle_ids_sha256=hashlib.sha256(indices.astype('<i8').tobytes()).hexdigest())
    if name == 'high':
        axis = np.asarray(part['face_normals'][402])
        matches = np.linalg.norm(normals-axis, axis=1)
        idx = int(matches.argmin())
        assert matches[idx] < 1e-12
        face = hull.simplices[idx]
        coplanar_residual = vertices[face] @ axis + offsets[idx]
        assert np.max(np.abs(coplanar_residual)) < 1e-12
        report['high_axis'] = dict(source_face_index=402, axis_local_link5=axis.tolist(),
                                   regenerated_hull_matching_face=idx,
                                   normal_difference=float(matches[idx]),
                                   face_vertex_plane_max_abs_residual_m=float(np.max(np.abs(coplanar_residual))),
                                   face_normal_verified=True)
crossing = (triangles[:, :, 1].min(1) <= .03) & (triangles[:, :, 1].max(1) >= .03)
assert np.all(masks['low'][crossing] & masks['high'][crossing])
assert covered.all()
report.update(covered_triangle_count=int(covered.sum()), missing_triangle_count=int((~covered).sum()),
              duplicated_complete_crossing_triangles=int(crossing.sum()),
              every_crossing_triangle_whole_in_both_parts=True,
              containment_tolerance_m=1e-10, audit_passed=True)
(HERE/'wrist_geometry_coverage_audit.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report))
