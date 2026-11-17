#!/usr/bin/python3
"""Offline geometry audit of a captured real snapshot. Never opens ROS or CAN."""
from pathlib import Path
import hashlib
import json
import sys

import numpy as np
from scipy.spatial import cKDTree


ROOT = Path(__file__).resolve().parent
RELEASE = ROOT / 'releases/standby_20261105_feedback_02'
sys.path.insert(0, str(RELEASE / 'core'))
from active_arm_model import PiperArmModel


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    model = PiperArmModel.from_parameters(RELEASE / 'model/parameters.yaml',
                                         RELEASE / 'model/mm.urdf')
    capture = np.load(ROOT / 'actual_geometry_snapshot.npz')
    q = capture['q_rad']
    native = model.native_transforms(q)
    urdf = model.urdf_link_transforms(q)
    native_arm = np.linalg.inv(model.mount) @ native
    urdf_arm = {k: np.linalg.inv(urdf['arm_base']) @ v for k, v in urdf.items()}

    # Load only the installed SDK's pure numerical FK source. Do not import its
    # package (__init__ can import hardware dependencies); defer its optional
    # typing_extensions annotation to avoid changing Python installations.
    sdk_path = Path('/home/l/.local/lib/python3.8/site-packages/piper_sdk/kinematics/piper_fk.py')
    sdk_source = sdk_path.read_text()
    optional_import = 'from typing_extensions import (\n    Literal,\n)'
    if sdk_source.count(optional_import) != 1:
        raise RuntimeError('Unexpected local pure FK source; review before loading')
    namespace = {}
    exec(compile('from __future__ import annotations\n' +
                 sdk_source.replace(optional_import, ''), str(sdk_path), 'exec'), namespace)
    sdk = namespace['C_PiperForwardKinematics'](dh_is_offset=1)

    def comparisons(angles):
        n = np.linalg.inv(model.mount) @ model.native_transforms(angles)
        u = model.urdf_link_transforms(angles)
        u = {k: np.linalg.inv(u['arm_base']) @ v for k, v in u.items()}
        f = np.asarray(sdk.CalFK(angles))[:, :3] / 1000.
        flange = n[5, :3, 3] - .120 * n[5, :3, 2]
        return {
            'dh_vs_sdk_shoulder': float(np.linalg.norm(n[0, :3, 3] - f[0])),
            'dh_vs_sdk_elbow': float(np.linalg.norm(n[1, :3, 3] - f[2])),
            'dh_vs_sdk_wrist': float(np.linalg.norm(n[3, :3, 3] - f[3])),
            'dh_vs_sdk_flange_after_removing_120mm_tool': float(np.linalg.norm(flange - f[5])),
            'dh_vs_urdf_elbow': float(np.linalg.norm(n[1, :3, 3] - u['link3'][:3, 3])),
            'dh_vs_urdf_wrist': float(np.linalg.norm(n[3, :3, 3] - u['link4'][:3, 3])),
            'dh_vs_urdf_flange_after_removing_120mm_tool': float(np.linalg.norm(flange - u['link6'][:3, 3])),
        }

    rng = np.random.RandomState(20261105)
    angles = np.vstack([q, np.zeros(6), model.q_nominal,
                        rng.uniform(model.lower, model.upper, size=(64, 6))])
    numerical = [comparisons(angles_i) for angles_i in angles]
    spheres = model.arm_spheres(q)
    pair = np.array([4, 13])
    raw = capture['raw_sensor_xyz']
    odom = capture['raw_odom_xyz']
    base = capture['base_xyyaw']
    c, s = np.cos(base[2]), np.sin(base[2])
    rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    base_points = (odom - np.r_[base[:2], 0.]) @ rotation
    sensor_origin = np.mean(base_points - raw, axis=0)
    residual = np.linalg.norm(base_points - raw - sensor_origin, axis=1)
    valid = np.isfinite(raw).all(axis=1) & (np.linalg.norm(raw, axis=1) > .05)
    distances, neighbors = cKDTree(base_points[valid]).query(spheres[:, :3])
    arm_sensor = spheres[:, :3] - sensor_origin
    arm_elevation = np.rad2deg(np.arctan2(arm_sensor[:, 2],
                                         np.linalg.norm(arm_sensor[:, :2], axis=1)))
    observed_elevation = np.rad2deg(np.arctan2(raw[valid, 2],
                                              np.linalg.norm(raw[valid, :2], axis=1)))

    physical_points = {}
    for label, native_point, urdf_link in (
            ('shoulder', native[0, :3, 3], 'link1'),
            ('elbow', native[1, :3, 3], 'link3'),
            ('wrist', native[3, :3, 3], 'link4'),
            ('flange', native[5, :3, 3] - .120 * native[5, :3, 2], 'link6')):
        physical_points[label] = {
            'native_ground_xyz_m': native_point.tolist(),
            'urdf_plus_original_base_z_0_115_xyz_m':
                (urdf[urdf_link][:3, 3] + [0., 0., .115]).tolist(),
        }

    report = {
        'scope': 'offline geometry audit of real feedback/cloud snapshot; no motor commands or ROS/CAN access',
        'source': 'real_feedback_and_raw_lidar_no_simulation',
        'q_deg': np.rad2deg(q).tolist(),
        'model_verdict': model.check(q),
        'findings': [
            'The isolated native DH constants and 20 arm sample points reproduce current MMConfig, not an independently calibrated real-arm collision model.',
            'Semantic shoulder/elbow/wrist points agree with installed pure SDK FK and original URDF after removing mount conventions; no large joint-angle sign/offset error is indicated.',
            'Current MMConfig disables Piper self-collision unconditionally; therefore original-project operation does not validate the new sphere self-collision policy.',
            'The failing sphere pair is on the folded upper-arm and forearm. Equal 34mm sphere radii overlap although the centre lines are about 37mm apart. This is evidence of approximation conservatism, not proof that real links are collision free.',
            'Native DH frame indexing includes an elbow connector frame; native link indices 1 and 3 correspond to the two physically adjoining long links. Do not blindly translate abstract adjacency into a validated exclusion policy.',
            'Most current arm sphere centres lie above this snapshot\'s observed LiDAR elevation. Lack of raw returns near the arm must not be treated as known free space.',
        ],
        'native_and_sdk_geometry': {
            'native_standard_DH_offsets_deg': [0., -172.22, -102.78, 0., 0., 0.],
            'sdk_modified_DH_offset_mode': 1,
            'native_final_d_m': .211,
            'sdk_final_d_m': .091,
            'native_tool_extension_from_flange_m': .120,
            'comparison_current_metres': numerical[0],
            'comparison_sample_count': len(numerical),
            'comparison_max_over_samples_metres': {k: max(n[k] for n in numerical) for k in numerical[0]},
            'sample_scope': 'offline FK arithmetic only; sampled configurations are not certified collision free and were never commanded',
        },
        'mount_conventions': {
            'native_ground_mount_xyz_m': model.mount[:3, 3].tolist(),
            'urdf_arm_base_relative_base_link_xyz_m': urdf['arm_base'][:3, 3].tolist(),
            'original_odometry_base_z_m': .115,
            'urdf_ground_mount_with_original_base_z_xyz_m': (urdf['arm_base'][:3, 3] + [0., 0., .115]).tolist(),
            'height_difference_m': float(urdf['arm_base'][2, 3] + .115 - model.mount[2, 3]),
            'interpretation': '8mm mount-height disagreement requires measurement; a common rigid mount shift cannot resolve an arm-arm self gap',
        },
        'failing_sphere_pair': {
            'zero_based_indices': pair.tolist(),
            'native_link_ids_zero_based': model.link_ids[pair].tolist(),
            'centres_ground_xyz_m': spheres[pair, :3].tolist(),
            'radii_m': spheres[pair, 3].tolist(),
            'centre_distance_m': float(np.linalg.norm(spheres[pair[0], :3] - spheres[pair[1], :3])),
            'surface_gap_m': float(np.linalg.norm(spheres[pair[0], :3] - spheres[pair[1], :3]) - sum(spheres[pair, 3])),
            'policy_change_made': False,
            'required_resolution': 'Calibrate link occupancy/adjacency against actual arm or validated collision meshes; retain current rejection until reviewed. No deleted pair, smaller radius, disabled self check or relaxed joint bound was applied.',
        },
        'raw_cloud_visibility': {
            'total_points': len(raw),
            'zero_points': int(np.sum(np.linalg.norm(raw, axis=1) < 1e-8)),
            'finite_range_over_50mm_points': int(np.sum(valid)),
            'inferred_sensor_origin_base_ground_xyz_m': sensor_origin.tolist(),
            'inferred_transform_max_scatter_m': float(max(residual)),
            'observed_raw_elevation_min_max_deg': [float(min(observed_elevation)), float(max(observed_elevation))],
            'arm_centre_elevation_min_max_deg': [float(min(arm_elevation)), float(max(arm_elevation))],
            'arm_spheres_nearest_valid_raw_point_distance_m': distances.tolist(),
            'arm_spheres_with_valid_raw_return_within_100mm': int(np.sum(distances < .100)),
            'interpretation': 'One snapshot only; these nearest points are unlabelled scene returns. No arm-surface match, free-space certificate or hardware-FOV specification is inferred. Cloud ground placement also depends on unvalidated extrinsics; FAST-LIO global drift must be fixed independently.',
        },
        'minimal_stationary_site_checks': [
            'Keep this captured pose stationary and compare shoulder, elbow, wrist and flange centres with the listed ground-relative coordinates; a side photo plus a ruler/known board is enough to identify gross sign or mount discrepancies.',
            'Measure ground-to-arm-base mounting height and LiDAR origin height; resolve native .680 versus URDF+.115=.688, and confirm LiDAR .28 forward / .117 high.',
            'Inspect and measure the physical upper-arm/forearm separation at the failing sphere region and compare with actual collision geometry; do not infer physical collision from 34mm centre-line spheres alone.',
            'Confirm firmware version from an existing label or prior record before choosing the pre/post S-V1.6-3 joint-2/joint-3 zero convention. No firmware-query frame was sent.',
            'Use a noncontact target in the intended arm obstacle volume while stationary and record raw returns across height and azimuth; establish observed/unknown space before any avoidance motion.',
            'Check actual gripper, camera and fingers against the .120m tool extension; the URDF joint7 origin is .13503m from flange and is not the same tip definition.',
        ],
        'stationary_reference_points': physical_points,
        'source_evidence': {
            'native_dh': 'src/mm_remani_planner/mm_config/src/mm_config.cpp:359',
            'native_self_collision_bypass': 'src/mm_remani_planner/mm_config/src/mm_config.cpp:1042',
            'native_arm_samples': 'src/mm_remani_planner/mm_config/src/mm_config.cpp:1421',
            'original_urdf': 'src/my_description/robot/mm.urdf',
            'original_odometry_base_z': 'src/mm_remani/scripts/fastlio_to_odom.py:23',
            'firmware_zero_convention': 'src/piper_ros/README(EN).md:25',
            'sdk_fk': str(sdk_path),
        },
        'source_hashes': {str(p): sha(p) for p in [
            ROOT / 'actual_geometry_snapshot.npz',
            RELEASE / 'core/active_arm_model.py',
            RELEASE / 'model/parameters.yaml',
            RELEASE / 'model/mm.urdf', sdk_path]},
        'hardware_commands_sent_by_audit': False,
        'source_runtime_or_collision_policy_modified': False,
    }
    destination = ROOT / 'geometry_alignment_audit.json'
    destination.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'report': str(destination),
                      'current_fk_errors_m': numerical[0],
                      'max_fk_errors_m': report['native_and_sdk_geometry']['comparison_max_over_samples_metres'],
                      'near_arm_raw_returns_100mm': int(np.sum(distances < .100))}, indent=2))


if __name__ == '__main__':
    main()
