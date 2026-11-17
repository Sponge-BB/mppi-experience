"""Small offline checks for stationary-only original-filter cloud inputs."""
from dataclasses import replace
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

RELEASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RELEASE / 'runtime'))
from hardware_cloud_snapshot import DynamicObservation, HardwareCloudSnapshot, SurfaceObservation
from pointcloud_numpy import read_points_numpy
import standby_integration_node as integration


class StationaryCloudInputsTests(unittest.TestCase):
    def test_accepted_dynamic_frame_notifies_planning_loop_but_duplicate_does_not(self):
        node = integration.HardwareShadowNode.__new__(integration.HardwareShadowNode)
        node.rospy = SimpleNamespace(get_time=lambda: 100.1, Duration=lambda value: value)
        node.world, node.max_age, node.config = 'odom', .25, {}
        node.lock = threading.Lock()
        node.cloud_ready = threading.Event()
        for key in ('clouds','errors','receipt','latencies','cloud_profiles'):
            setattr(node, key, {})
        names = ('x','y','z','vx','vy','vz','confidence','track_id')
        msg = SimpleNamespace(width=0,height=1,point_step=32,row_step=0,is_bigendian=False,
            data=b'', header=SimpleNamespace(frame_id='odom',stamp=SimpleNamespace(to_sec=lambda:100.)),
            fields=[SimpleNamespace(name=name,offset=i*4,count=1,datatype=6 if i==7 else 7)
                    for i,name in enumerate(names)])
        node.cloud(msg, 'dynamic_cloud')
        self.assertTrue(node.cloud_ready.is_set())
        self.assertFalse(node.errors)
        node.cloud_ready.clear()
        node.cloud(msg, 'dynamic_cloud')
        self.assertFalse(node.cloud_ready.is_set())
        self.assertEqual(node.errors['dynamic_cloud'], 'nonmonotonic_cloud_stamp')

    def test_sensor_zero_returns_are_removed_before_translation_not_nearby_returns(self):
        node = integration.HardwareShadowNode.__new__(integration.HardwareShadowNode)
        node.rospy = SimpleNamespace(get_time=lambda: 100.1, Duration=lambda value: value)
        node.world, node.max_age, node.config = 'odom', .25, {}
        node.lock = threading.Lock()
        node.cloud_ready = threading.Event()
        for key in ('clouds','errors','receipt','latencies','cloud_profiles'):
            setattr(node, key, {})
        transform = SimpleNamespace(translation=SimpleNamespace(x=.28,y=0.,z=.117),
                                    rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
        node.tf = SimpleNamespace(lookup_transform=lambda *args: SimpleNamespace(transform=transform))
        points = np.asarray([[0.,0.,0.],[.001,0.,0.],[0.,.1,0.]], dtype='<f4')
        msg = SimpleNamespace(width=3,height=1,point_step=12,row_step=36,is_bigendian=False,
              data=points.tobytes(), header=SimpleNamespace(frame_id='lidar',stamp=SimpleNamespace(to_sec=lambda:100.)),
              fields=[SimpleNamespace(name=name,offset=i*4,count=1,datatype=7)
                      for i,name in enumerate(('x','y','z'))])
        node.cloud(msg, 'raw_cloud')
        self.assertFalse(node.errors)
        np.testing.assert_allclose(node.clouds['raw_cloud'].points, points[1:]+[.28,0.,.117])
        self.assertEqual(node.cloud_profiles['raw_cloud']['invalid_zero_returns_removed'], 1)

    def snapshot(self, dynamic=None):
        def surface(x, source):
            return SurfaceObservation.create([[x, 0., 0.]], 100., 'odom', 'odom', source)
        if dynamic is None:
            dynamic = DynamicObservation.create([], 100., 'odom', 'odom')
        return HardwareCloudSnapshot(surface(0., 'raw'), surface(.8, 'filtered'),
                                     surface(1., 'static'), dynamic, 100., True)

    def test_cohort_waits_for_matching_live_clouds_and_state_without_restamping(self):
        snapshot = self.snapshot()
        check = integration.HardwareShadowNode.cohort_aligned
        self.assertTrue(check((np.zeros(17), snapshot, 100., 100.02, {})))
        old_raw = replace(snapshot, raw=replace(snapshot.raw, stamp=99.9))
        self.assertFalse(check((np.zeros(17), old_raw, 100., 100.02, {})))
        self.assertFalse(check((np.zeros(17), snapshot, 99.8, 100.02, {})))
        self.assertFalse(check((np.zeros(17), snapshot, 100., 100.02, {'raw_cloud':'tf_unavailable'})))
        self.assertEqual(old_raw.raw.stamp, 99.9)

    def test_schema_valid_empty_frame_is_a_no_tracks_observation(self):
        names = ('x', 'y', 'z', 'vx', 'vy', 'vz', 'confidence', 'track_id')
        message = SimpleNamespace(width=0, height=1, point_step=32, row_step=0,
            is_bigendian=False, data=b'', fields=[SimpleNamespace(name=name,
                offset=4*i, count=1, datatype=6 if name == 'track_id' else 7)
                for i, name in enumerate(names)])
        rows = read_points_numpy(message, names)
        observation = DynamicObservation.create(rows, 100., 'odom', 'odom')
        self.assertEqual(observation.groups, ())
        self.assertEqual(observation.point_count, 0)
        self.assertTrue(self.snapshot(observation).fresh(100.1))
        message.fields.pop()
        with self.assertRaisesRegex(ValueError, 'requested_field_missing'):
            read_points_numpy(message, names)
        for stamp, frame in ((0., 'odom'), (100., 'sensor')):
            with self.assertRaises(ValueError):
                DynamicObservation.create([], stamp, frame, 'odom')

    def test_nonempty_rejected_rows_never_become_a_valid_empty_frame(self):
        rows = [[0., 0., 0., 0., 0., 0., .1, 1.]]
        with self.assertRaisesRegex(ValueError, 'no_confident_dynamic_points'):
            DynamicObservation.create(rows, 100., 'odom', 'odom')
        for column, invalid in ((0, np.nan), (3, 3.), (6, -1.), (7, 1.5)):
            row = [0., 0., 0., 0., 0., 0., .9, 1.]
            row[column] = invalid
            with self.subTest(column=column), self.assertRaises(ValueError):
                DynamicObservation.create([row], 100., 'odom', 'odom')

    def test_filtered_static_and_dynamic_affect_gap_but_raw_is_diagnostic(self):
        snapshot = self.snapshot()
        sphere = np.array([[0., 0., 0., .1]])
        self.assertTrue(snapshot.original_filter_shadow_only)
        self.assertAlmostEqual(float(snapshot.query(sphere, 100.)[0]), .13)
        self.assertLess(float(snapshot.raw.tree.query(sphere[:, :3])[0][0]-.1), 0.)
        near_static = SurfaceObservation.create([[.25, 0., 0.]], 1., 'odom', 'odom', 'static')
        self.assertAlmostEqual(float(replace(snapshot, static=near_static).query(sphere, 100.)[0]), .07)
        dynamic = DynamicObservation.create([[.2, 0., 0., 0., 0., 0., .9, 1.]],
                                             100., 'odom', 'odom')
        self.assertAlmostEqual(float(replace(snapshot, dynamic=dynamic).query(sphere, 100.)[0]), .02)
        self.assertEqual(float(snapshot.query(sphere, 103.)[0]), -1000.)

    def test_local_search_preserves_collision_and_soft_cost_with_conservative_far_gap(self):
        rng = np.random.RandomState(707)
        points = rng.uniform(-3.,3.,(800,3))
        points = np.vstack([points, [[.15,0.,0.],[.25,.3,0.],[.31,-.2,0.]]])
        surface = SurfaceObservation.create(points,100.,'odom','odom','filtered')
        snapshot = replace(self.snapshot(),filtered=surface,static=surface)
        spheres = np.concatenate([rng.uniform(-.3,.3,(12,4,3)),
                                  np.full((12,4,1),.1)],axis=2)
        exact = surface.tree.query(spheres[...,:3].reshape(-1,3))[0].reshape(12,4)-.08-.1
        gap, cost = snapshot.query(spheres,np.full(12,100.))
        np.testing.assert_array_equal(gap>=0.,exact.min(axis=1)>=0.)
        self.assertTrue(np.all(gap<=exact.min(axis=1)+1e-12))
        np.testing.assert_allclose(cost,180.*np.sum(np.maximum(0.,.08-exact)**3,axis=1),atol=1e-12)

    def test_required_inputs_freshness_and_health_stay_strict_with_no_tracks(self):
        snapshot = self.snapshot()
        for name in ('raw', 'filtered', 'static', 'dynamic'):
            with self.subTest(missing=name):
                self.assertEqual(replace(snapshot, **{name: None}).rejection_reason(100.1),
                                 name+'_not_received')
        for name in ('raw', 'filtered', 'dynamic'):
            with self.subTest(stale=name):
                stale = replace(getattr(snapshot, name), stamp=99.)
                self.assertEqual(replace(snapshot, **{name: stale}).rejection_reason(100.1),
                                 name+'_stale')
        for change in ({'healthy': False}, {'health_stamp': 99.}):
            self.assertEqual(replace(snapshot, **change).rejection_reason(100.1),
                             'perception_health_unavailable')
        self.assertTrue(replace(snapshot, static=replace(snapshot.static, stamp=1.)).fresh(100.1))

    def test_execute_is_rejected_even_if_readiness_and_config_are_changed(self):
        def fake_shadow_init(node, release, namespace):
            node.rospy = SimpleNamespace(get_param=lambda *args: True)
            node.config = {'original_filter_shadow_only': False}

        with patch.object(integration.HardwareShadowNode, '__init__', fake_shadow_init), \
                patch.object(integration, 'validate_readiness', return_value=(True, 'ready')) as readiness, \
                patch.object(integration, 'RosCommandSink') as sink:
            with self.assertRaisesRegex(RuntimeError, 'original_filter_shadow_only'):
                integration.HardwareIntegrationNode()
            readiness.assert_not_called()
            sink.assert_not_called()


if __name__ == '__main__':
    unittest.main()
