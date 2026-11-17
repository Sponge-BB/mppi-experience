#!/usr/bin/env python3
"""Offline convex support certificate candidate from unchanged original meshes.

Run with OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONNOUSERSITE=1 and a
512 MiB address-space limit. No ROS, hardware, package installs, or mesh edits.
"""
import hashlib
import json
import os
from pathlib import Path
import struct
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True

import numpy as np
from scipy.spatial import ConvexHull
import pyassimp


ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
URDF = ROOT / 'src/my_description/robot/mm.urdf'
MODEL = ROOT / 'experiments/hardware_standby_mppi/releases/standby_20261106_alignment_05/core'
sys.path.insert(0, str(MODEL))
from active_arm_model import PiperArmModel, _origin  # noqa: E402


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def transform(points, matrix):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def read_mesh(path):
    if path.suffix.lower() == '.stl':
        data = path.read_bytes()
        count = struct.unpack('<I', data[80:84])[0]
        if len(data) != 84 + 50 * count:
            raise ValueError('Expected binary STL: ' + str(path))
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
        vertices = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3).astype(float)
        return vertices, {'triangles': count, 'scene_nodes_applied': False}
    scene = pyassimp.load(str(path))
    vertices = []
    triangle_count = 0
    node_count = 0
    def visit(node, parent):
        nonlocal triangle_count, node_count
        matrix = parent @ np.asarray(node.transformation, dtype=float)
        node_count += 1
        for mesh in node.meshes:
            vertices.append(transform(np.asarray(mesh.vertices, dtype=float), matrix))
            triangle_count += len(mesh.faces)
        for child in node.children:
            visit(child, matrix)
    try:
        visit(scene.rootnode, np.eye(4))
        result = np.concatenate(vertices)
    finally:
        pyassimp.release(scene)
    return result, {'triangles': triangle_count, 'scene_nodes_applied': True, 'scene_node_count': node_count}


def main():
    root = ET.parse(URDF).getroot()
    proxies = {}
    hulls = {}
    points = {}
    for name in ('arm_base', 'link1', 'link5', 'link6'):
        collision = root.find("link[@name='%s']/collision" % name)
        mesh = collision.find('geometry/mesh')
        uri = mesh.get('filename')
        assert uri.startswith('package://my_description/')
        path = ROOT / 'src/my_description' / uri[len('package://my_description/'):]
        fallback = None
        if not path.exists():
            if name != 'arm_base' or uri != 'package://my_description/meshes/arm/dae/base_link.stl':
                raise FileNotFoundError(str(path))
            specified_path = path
            path = ROOT / 'src/my_description/meshes/arm/base_link.stl'
            matching_original = ROOT / 'src/piper_description/meshes/base_link.STL'
            if digest(path) != digest(matching_original):
                raise ValueError('arm_base fallback no longer matches original Piper mesh')
            fallback = {'reason': 'Original URDF mesh URI points to a broken /home/hai symlink.',
                        'specified_path': str(specified_path.relative_to(ROOT)),
                        'specified_symlink_target': os.readlink(specified_path),
                        'matching_original_mesh': str(matching_original.relative_to(ROOT)),
                        'matching_original_mesh_sha256': digest(matching_original),
                        'original_link_modified': False}
        raw, details = read_mesh(path)
        scale = np.asarray([float(x) for x in mesh.get('scale', '1 1 1').split()])
        local = transform(raw * scale, _origin(collision.find('origin')))
        unique = np.unique(local, axis=0)
        hull = ConvexHull(unique)
        hulls[name] = hull
        points[name] = unique[hull.vertices]
        proxies[name] = {
            'coordinate_frame': name,
            'source_mesh': str(path.relative_to(ROOT)),
            'source_mesh_sha256': digest(path),
            'source_uri': uri,
            'explicit_fallback': fallback,
            'collision_origin': dict(collision.find('origin').attrib),
            'mesh_scale': scale.tolist(),
            'collision_origin_and_scene_transforms_applied': True,
            'input_vertex_count': int(len(raw)),
            'hull_vertex_count': int(len(points[name])),
            'vertices': points[name].tolist(),
            'local_bounds': [local.min(0).tolist(), local.max(0).tolist()],
            **details,
        }
    joints = []
    for i in range(1, 7):
        joint = root.find("joint[@name='joint%d']" % i)
        joints.append({'name': joint.get('name'), 'parent': joint.find('parent').get('link'),
                       'child': joint.find('child').get('link'),
                       'origin': dict(joint.find('origin').attrib),
                       'axis': [float(x) for x in joint.find('axis').get('xyz').split()]})

    snapshot = ROOT / 'experiments/hardware_standby_mppi/raw_baseline_20261106.npz'
    q = np.load(snapshot)['joints']
    model = PiperArmModel([[0., 0., 0., .01]], urdf_path=URDF)
    transforms = model.urdf_link_transforms(q)
    align = model.mount @ np.linalg.inv(transforms['arm_base'])
    transforms = {name: align @ transforms[name] for name in points}
    world = {name: transform(vertices, transforms[name]) for name, vertices in points.items()}
    shoulder = np.concatenate([world['arm_base'], world['link1']])
    wrist = np.concatenate([world['link5'], world['link6']])
    normals = hulls['link5'].equations[:, :3]
    best = None
    for local_axis in normals:
        axis = transforms['link5'][:3, :3] @ local_axis
        for sign in (1., -1.):
            oriented = axis * sign
            gap = float(np.min(shoulder @ oriented) - np.max(wrist @ oriented))
            if best is None or gap > best['gap_m']:
                best = {'gap_m': gap, 'axis_local_link5': (local_axis * sign).tolist(),
                        'axis_world': oriented.tolist(),
                        'shoulder_min_projection_m': float(np.min(shoulder @ oriented)),
                        'wrist_union_max_projection_m': float(np.max(wrist @ oriented)),
                        'link5_only_gap_m': float(np.min(shoulder @ oriented) - np.max(world['link5'] @ oriented)),
                        'link6_only_gap_m': float(np.min(shoulder @ oriented) - np.max(world['link6'] @ oriented))}
    axis = np.asarray(best['axis_world'])
    best['component_gaps_m'] = {
        left + '--' + right: float(np.min(world[left] @ axis) - np.max(world[right] @ axis))
        for left in ('arm_base', 'link1') for right in ('link5', 'link6')}
    # The upper arm has a fork near the shoulder. One whole-link hull fills
    # that gap. Split at its original local z=0 plane; retain every complete
    # crossing triangle in BOTH pieces, so this cannot discard mesh geometry.
    collision = root.find("link[@name='link2']/collision")
    mesh = collision.find('geometry/mesh')
    path2 = ROOT/'src/my_description'/mesh.get('filename')[len('package://my_description/'):]
    raw2, details2 = read_mesh(path2)
    scale2 = np.asarray([float(x) for x in mesh.get('scale', '1 1 1').split()])
    triangles = transform(raw2*scale2, _origin(collision.find('origin'))).reshape(-1, 3, 3)
    masks = {'low': triangles[:, :, 2].min(axis=1) <= 0.,
             'high': triangles[:, :, 2].max(axis=1) >= 0.}
    assert np.all(masks['low'] | masks['high'])
    axes = {'low': [-.8467244546722408, .114130134529918, -.5196460432372976],
            'high': [-.9169874207629914, -.004804760638082535, .3988871825938344]}
    t2 = align @ model.urdf_link_transforms(q)['link2']
    parts = {}
    for name, mask in masks.items():
        unique = np.unique(triangles[mask].reshape(-1, 3), axis=0)
        vertices = unique[ConvexHull(unique).vertices]
        direction = transforms['link5'][:3, :3] @ np.asarray(axes[name])
        a, b = transform(vertices, t2) @ direction, wrist @ direction
        gap = float(max(a.min()-b.max(), b.min()-a.max()))
        assert gap > .0001
        parts[name] = {'vertices': vertices.tolist(), 'coordinate_frame': 'link2',
                       'hull_vertex_count': len(vertices), 'triangle_count': int(mask.sum()),
                       'axis_local_link5': axes[name], 'baseline_gap_m': gap}
    upper_arm = {'source_mesh': str(path2.relative_to(ROOT)), 'source_mesh_sha256': digest(path2),
                 'collision_origin': dict(collision.find('origin').attrib),
                 'mesh_scale': scale2.tolist(), 'collision_origin_and_scene_transforms_applied': True,
                 'split': {'frame': 'link2', 'axis': 2, 'value': 0.,
                           'crossing_triangles': 'complete triangle included in both pieces'},
                 'original_triangle_count': len(triangles),
                 'covered_triangle_count': int(np.count_nonzero(masks['low'] | masks['high'])),
                 'duplicated_crossing_triangles': int(np.count_nonzero(masks['low'] & masks['high'])),
                 'parts': parts, 'wrist_union': ['link5', 'link6'],
                 'acceptance': 'Both pieces require a positive separating support gap at each candidate q.'}
    output = {
        'schema': 'offline_shoulder_wrist_convex_support_candidate_v1',
        'status': 'candidate_only_not_hardware_validation',
        'units': 'metres_and_radians',
        'source_urdf': str(URDF.relative_to(ROOT)),
        'source_urdf_sha256': digest(URDF),
        'source_fk_module_sha256': digest(MODEL / 'active_arm_model.py'),
        'joint_chain_arm_base_to_link6': joints,
        'proxies': proxies,
        'upper_arm_certificate': upper_arm,
        'certificate': {
            'description': 'min((arm_base union link1) dot n) - max((link5 union link6) dot n); positive certifies separating plane between complete original collision mesh convex hulls only',
            'direction_frame': 'link5',
            'projection_convention': 'Each proxy stores vertices in its URDF link frame after collision origin and scene transforms. Compute T_arm_base_link_i from the stored joint1..6 origin then Rodrigues(axis,q). Set n_arm_base = R_arm_base_link5 @ axis_local_link5. Project transformed arm_base/link1 vertices and link5/link6 vertices onto n_arm_base; global rigid mounting cancels from the gap.',
            'searched_axes': int(len(normals) * 2),
            'fixed_axis_chosen_from_link5_hull_face_normals': True,
            'snapshot': str(snapshot.relative_to(ROOT)),
            'snapshot_sha256': digest(snapshot),
            'q_radians': q.tolist(),
            'baseline': best,
            'limitations': ['No physical calibration or motion authorization.',
                            'No all-configuration guarantee; recompute support gap for each candidate state.',
                            'Nonpositive support gap is inconclusive and must remain rejected.',
                            'Only complete arm_base/link1 against complete link5/link6 meshes; all other collision checks remain necessary.'],
        },
    }
    target = HERE / 'shoulder_wrist_proxy_candidate.json'
    target.write_text(json.dumps(output, indent=2) + '\n')
    print(json.dumps({'output': str(target), 'hull_vertices': {name: len(v) for name, v in points.items()}, 'baseline': best}))


if __name__ == '__main__':
    main()
