#!/usr/bin/env python3
"""Generate fixed link5 partitions from original mesh; no posture input."""
import hashlib
import json
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'src/mm_remani/scripts/mppi'))
from active_arm_model import _origin


def source_triangles():
    asset = json.loads((ROOT/'src/mm_remani/config/mppi_shoulder_wrist_proxy.json').read_text())
    item = asset['proxies']['link5']
    path = ROOT/item['source_mesh']
    data = path.read_bytes()
    assert hashlib.sha256(data).hexdigest() == item['source_mesh_sha256']
    count = struct.unpack('<I', data[80:84])[0]
    assert len(data) == 84+50*count
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
    triangles = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].astype(float)
    urdf = ET.parse(ROOT/'src/my_description/robot/mm.urdf').getroot()
    origin = urdf.find("link[@name='link5']/collision/origin")
    assert origin.attrib == item['collision_origin']
    transform = _origin(origin)
    triangles *= np.asarray(item['mesh_scale'])
    triangles = triangles @ transform[:3, :3].T + transform[:3, 3]
    return item, triangles


def build():
    item, triangles = source_triangles()
    masks = [triangles[:, :, 1].min(1) <= .03,
             triangles[:, :, 1].max(1) >= .03]
    parts = {}
    for name, mask in zip(('low', 'high'), masks):
        unique = np.unique(triangles[mask].reshape(-1, 3), axis=0)
        hull = ConvexHull(unique)
        vertices = unique[hull.vertices]
        parts[name] = dict(coordinate_frame='link5', vertices=vertices.tolist(),
                           hull_vertex_count=len(vertices), triangle_count=int(mask.sum()),
                           face_normals=hull.equations[:, :3].tolist())
    return dict(status='geometry_candidate_only_needs_generic_direction_fallback',
                source_mesh=item['source_mesh'], source_mesh_sha256=item['source_mesh_sha256'],
                collision_origin=item['collision_origin'], mesh_scale=item['mesh_scale'],
                collision_origin_and_scene_transforms_applied=True,
                split=dict(frame='link5', axis=1, value=.03,
                           crossing_triangles='complete crossing triangle included in both pieces'),
                original_triangle_count=len(triangles),
                covered_triangle_count=int((masks[0] | masks[1]).sum()),
                duplicated_crossing_triangles=int((masks[0] & masks[1]).sum()), parts=parts)


if __name__ == '__main__':
    output = build()
    (HERE/'wrist_geometry_candidate.json').write_text(json.dumps(output, indent=2)+'\n')
    print(json.dumps({name: (part['triangle_count'], part['hull_vertex_count'])
                      for name, part in output['parts'].items()}))
