#!/usr/bin/env python3
"""Offline original-triangle diagnostic; no ROS or control interfaces."""
import hashlib
import json
import resource
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT / 'src/mm_remani/scripts/mppi'))
from active_arm_model import PiperArmModel, _origin

Q = np.array([-.042708206796301246, -.03331833542057175,
              .01645845484630653, .10458012877950022,
              .45671775866187614, -.10260790672474664])
asset_path = ROOT / 'src/mm_remani/config/mppi_shoulder_wrist_proxy.json'
asset = json.loads(asset_path.read_text())
params_path = HERE.parent / 'hardware_startup_20261108_183157/planner_parameters_at_attempt.json'
model = PiperArmModel.from_parameters(json.loads(params_path.read_text()))
transforms = model.urdf_link_transforms(Q)
root = ET.parse(model.urdf_path).getroot()
pairs = [('link1', 'link5'), ('arm_base', 'link5'),
         ('arm_base', 'link6'), ('link1', 'link6')]
out = dict(q_rad=Q.tolist(), scope='Original unchanged URDF collision meshes, all triangles; no physical calibration or execution.',
           limitation='Surface separation alone does not exclude solid containment.',
           sphere_check=model.check(Q), fixed_proxy_gaps_m=model.shoulder_wrist_proxy.pair_gaps(Q[None]).tolist(),
           original_mesh_sha256={}, results={})

def limit():
    resource.setrlimit(resource.RLIMIT_AS, (1024**3, 1024**3))

for a, b in pairs:
    files, poses = [], []
    for name in (a, b):
        item = asset['proxies'][name]
        assert item['mesh_scale'] == [1., 1., 1.]
        path = ROOT / item['source_mesh']
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        assert digest == item['source_mesh_sha256']
        out['original_mesh_sha256'][name] = digest
        collision = root.find("link[@name='%s']/collision" % name)
        poses.append((transforms[name] @ _origin(collision.find('origin')))[:3].ravel())
        files.append(str(path))
    row = ' '.join('%.17g' % x for x in np.concatenate(poses)) + '\n'
    run = subprocess.run([str(HERE/'mesh_pair_distance')] + files, input=row,
                         text=True, capture_output=True, check=True,
                         preexec_fn=limit, timeout=20)
    result = json.loads(run.stdout)
    result['input_transforms_3x4'] = [x.tolist() for x in poses]
    out['results'][a+'__'+b] = result
    print(a, b, result['surface_intersection'], result['surface_distance_m'], flush=True)
(HERE/'mesh_components_audit.json').write_text(json.dumps(out, indent=2)+'\n')
previous = HERE/'full_hull_separation_preliminary.json'
if previous.exists():
    p = json.loads(previous.read_text())
    p['valid_separation_evidence'] = False
    p['correction'] = ('Optimizer converged to a near-zero normal and zero margin. '
                       'Normalizing that degenerate normal produced an arbitrary direction. '
                       'The reported raw gap is NOT an optimal separation or intersection certificate.')
    previous.write_text(json.dumps(p, indent=2)+'\n')
