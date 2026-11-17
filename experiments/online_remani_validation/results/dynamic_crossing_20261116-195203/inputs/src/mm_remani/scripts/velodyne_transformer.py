#!/usr/bin/python3
import math
from pathlib import Path
import sys
import threading
import time
from collections import deque

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, JointState, PointField
from sensor_msgs import point_cloud2
import tf2_sensor_msgs.tf2_sensor_msgs


def outside_self_spheres(points, spheres, padding=.01, chunk_size=2048):
    """Keep finite points outside physical spheres, with bounded workspace."""
    points = np.asarray(points, dtype=float)
    spheres = np.asarray(spheres, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or spheres.shape != (67, 4) or
            not np.isfinite(spheres).all() or np.any(spheres[:, 3] <= 0.) or
            not math.isfinite(padding) or padding < 0. or
            not isinstance(chunk_size, int) or not 1 <= chunk_size <= 2048):
        raise ValueError('invalid physical self-filter geometry or chunk size')
    keep = np.isfinite(points).all(axis=1)
    radii = spheres[:, 3]+padding
    radius2 = radii**2
    # Outside this union-enclosing box no sphere can contain the point. Round
    # its faces outward so the broad phase never clips a floating-point edge.
    lo = np.nextafter(np.min(spheres[:, :3]-radii[:, None], axis=0), -np.inf)
    hi = np.nextafter(np.max(spheres[:, :3]+radii[:, None], axis=0), np.inf)
    for start in range(0, len(points), chunk_size):
        ids = np.flatnonzero(keep[start:start+chunk_size])+start
        batch = points[ids]
        candidate = np.all((batch >= lo) & (batch <= hi), axis=1)
        ids = ids[candidate]
        if not len(ids):
            continue
        batch = points[ids]
        delta = batch[:, None, :]-spheres[None, :, :3]
        inside = np.any(np.einsum('nsi,nsi->ns', delta, delta) <= radius2, axis=1)
        keep[ids[inside]] = False
    return keep


def rigid_transform(transform):
    """Validate an exact TF and return its rotation/translation without fallback."""
    t, q = transform.transform.translation, transform.transform.rotation
    values = np.array([t.x, t.y, t.z, q.x, q.y, q.z, q.w])
    if not np.isfinite(values).all() or abs(np.linalg.norm(values[3:])-1.) > .001:
        raise ValueError('invalid self-filter TF')
    x, y, z, w = values[3:]/np.linalg.norm(values[3:])
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    return rotation, values[:3]


def mapping_voxel_indices(world, sensor_origin, resolution, map_origin,
                          map_size, max_ray_length, projection_mask=None):
    """Retain the native first endpoint per voxel, without altering clipped rays."""
    world = np.asarray(world, dtype=float)
    sensor_origin = np.asarray(sensor_origin, dtype=float)
    map_origin = np.asarray(map_origin, dtype=float)
    map_size = np.asarray(map_size, dtype=float)
    if (world.ndim != 2 or world.shape[1] != 3 or
            any(value.shape != (3,) for value in (sensor_origin, map_origin, map_size)) or
            not all(np.isfinite(value).all() for value in
                    (world, sensor_origin, map_origin, map_size)) or
            np.any(map_size <= 0.) or not math.isfinite(resolution) or resolution <= 0. or
            not math.isfinite(max_ray_length) or max_ray_length <= 0.):
        raise ValueError('invalid mapping voxel geometry')
    if float(np.prod(map_size/resolution+1.)) >= 2.**62:
        raise ValueError('mapping voxel address overflow')
    dimensions = np.ceil(map_size/resolution).astype(np.int64)
    # The native map clips out-of-map and long beams before checking rayend.
    # Leave those beams untouched; grouping their original endpoints could
    # otherwise remove a distinct clipped free-space ray.
    eligible = (np.all((world >= map_origin+1e-4) &
                       (world <= map_origin+map_size-1e-4), axis=1) &
                (np.sum((world-sensor_origin)**2, axis=1) <= max_ray_length**2))
    if projection_mask is not None:
        projection_mask = np.asarray(projection_mask)
        if projection_mask.shape != (len(world),) or projection_mask.dtype != np.bool_:
            raise ValueError('invalid native mapping projection mask')
        # An out-of-window first point must not replace a later valid hit in
        # the same world voxel: projection happens before native rayend.
        eligible &= projection_mask
    ids = np.flatnonzero(eligible)
    keys = np.floor((world[ids]-map_origin)*(1./resolution)).astype(np.int64)
    addresses = (keys[:, 0]*dimensions[1]+keys[:, 1])*dimensions[2]+keys[:, 2]
    _, first = np.unique(addresses, return_index=True)
    keep = ~eligible
    keep[ids[first]] = True
    return np.flatnonzero(keep)  # Original order and the original measured point.


def cloud_xyz(msg):
    """Read real float32 XYZ without unpacking unrelated fields point by point."""
    selected = [field for field in msg.fields if field.name in ('x', 'y', 'z')]
    if (len(selected) != 3 or len({field.name for field in selected}) != 3 or
            any(field.datatype != PointField.FLOAT32 or field.count != 1 for field in selected)):
        raise ValueError('mapping output requires scalar float32 XYZ')
    fields = {field.name: field for field in selected}
    dtype = np.dtype({'names': ['x', 'y', 'z'],
                      'formats': [('>' if msg.is_bigendian else '<')+'f4']*3,
                      'offsets': [fields[name].offset for name in ('x', 'y', 'z')],
                      'itemsize': msg.point_step})
    rows = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      strides=(msg.row_step, msg.point_step))
    return np.column_stack([rows[name].reshape(-1) for name in ('x', 'y', 'z')])


def mapping_xyz_cloud(msg, indices):
    """Only the mapping branch drops unused fields; source frame/time stay exact."""
    xyz = cloud_xyz(msg)[indices]
    return PointCloud2(header=msg.header, height=1, width=len(xyz),
                       fields=[PointField(name, 4*i, PointField.FLOAT32, 1)
                               for i, name in enumerate(('x', 'y', 'z'))],
                       is_bigendian=False, point_step=12, row_step=12*len(xyz),
                       data=np.asarray(xyz, dtype='<f4').tobytes(),
                       is_dense=bool(np.isfinite(xyz).all()))


class CloudTransformer:
    def __init__(self):
        rospy.init_node('velodyne_transformer')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.filter_self = rospy.get_param('~filter_self', False)
        if type(self.filter_self) is not bool:
            raise ValueError('filter_self must be a boolean')
        mapping_sparse = rospy.get_param('~mapping_sparse', False)
        if type(mapping_sparse) is not bool or (mapping_sparse and not self.filter_self):
            raise ValueError('mapping_sparse requires boolean and physical self filtering')
        self.mapping_pub = None
        if mapping_sparse:
            grid = '/remani_planner_node/grid_map/'
            self.mapping_resolution = float(rospy.get_param(grid+'resolution'))
            self.mapping_size = np.asarray([rospy.get_param(grid+'map_size_'+axis)
                                            for axis in 'xyz'], dtype=float)
            self.mapping_origin = np.array([-self.mapping_size[0]/2.,
                                            -self.mapping_size[1]/2.,
                                            float(rospy.get_param(grid+'ground_height'))])
            self.mapping_max_ray_length = float(rospy.get_param(grid+'max_ray_length'))
            self.mapping_local_range = np.asarray([rospy.get_param(grid+'local_update_range_'+axis)
                                                   for axis in 'xyz'], dtype=float)
            if (not np.isfinite(self.mapping_local_range).all() or
                    np.any(self.mapping_local_range <= 0.)):
                raise ValueError('invalid native mapping projection range')
            mapping_voxel_indices(np.empty((0, 3)), np.zeros(3), self.mapping_resolution,
                                  self.mapping_origin, self.mapping_size,
                                  self.mapping_max_ray_length)
            self.mapping_pub = rospy.Publisher('~mapping_output', PointCloud2, queue_size=1)
        self.target_frame = rospy.get_param('~target_frame', 'livox_frame' if self.filter_self else 'world')
        if self.filter_self:
            sys.path.insert(0, str(Path(__file__).resolve().parent/'mppi'))
            from active_arm_model import PiperArmModel
            self.model = PiperArmModel.from_parameters(rospy.get_param('/remani_planner_node'))
            if len(self.model.fixed_spheres) != 47:
                raise ValueError('self-filter requires current 47 fixed + 20 arm spheres')
            self.joint_lock = threading.Lock()
            self.joint_condition = threading.Condition(self.joint_lock)
            self.q = None; self.joint_stamp = 0.
            self.joint_history = deque(maxlen=64)
            self.joint_sub = rospy.Subscriber('/mm/joint_states', JointState, self.joints_cb,
                                               queue_size=1, tcp_nodelay=True)
        self.pub = rospy.Publisher('~output', PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(
            '~input', PointCloud2, self.cloud_cb, queue_size=1,
            buff_size=1024**2, tcp_nodelay=True)
        rospy.loginfo("Transformer node started, target_frame: %s, filter_self: %s",
                      self.target_frame, self.filter_self)

    def joints_cb(self, msg):
        stamp = math.nan
        try:
            stamp = msg.header.stamp.to_sec()
            if math.isfinite(stamp) and stamp > 0.:
                with self.joint_lock:
                    if stamp <= self.joint_stamp:
                        return
            ids = [msg.name.index('joint'+str(i)) for i in range(1, 7)]
            if len(msg.name) != len(set(msg.name)):
                raise ValueError('duplicate self-filter joint names')
            q = np.asarray([msg.position[i] for i in ids], dtype=float)
            if (not np.isfinite(q).all() or not math.isfinite(stamp) or stamp <= 0. or
                    not -.005 <= rospy.get_time()-stamp <= .15 or
                    np.any(q < self.model.lower) or np.any(q > self.model.upper)):
                raise ValueError('invalid or stale self-filter joints')
            with self.joint_lock:
                if stamp > self.joint_stamp:
                    self.q = q; self.joint_stamp = stamp
                    self.joint_history.append((stamp, q.copy()))
                    self.joint_condition.notify_all()
        except (ValueError, IndexError) as exc:
            with self.joint_lock:
                # Another callback may have accepted a newer source while
                # this message was being checked. It cannot revoke that q.
                if math.isfinite(stamp) and 0. < stamp <= self.joint_stamp:
                    return
                self.q = None
                self.joint_history.clear()
                self.joint_condition.notify_all()
            rospy.logwarn_throttle(2., 'Robot self-filter joint feedback rejected: %s', exc)

    def joints_at_scan(self, scan_stamp, now):
        """Wait briefly for real source samples, then retain the same time gates."""
        began = time.monotonic()
        with self.joint_condition:
            while True:
                elapsed = time.monotonic()-began
                current_now = now+elapsed
                history = list(self.joint_history)
                if (self.q is None or not history or
                        not -.005 <= current_now-self.joint_stamp <= .15):
                    raise ValueError('self-filter joint feedback unavailable or stale')
                if scan_stamp-history[-1][0] <= .02+1e-9:
                    break
                remaining = min(.05-elapsed, .15-(current_now-scan_stamp))
                if remaining <= 0.:
                    raise ValueError('self-filter timed out waiting for scan-aligned joint feedback')
                # Condition.wait releases the joint lock: the source callback
                # can append the missing real sample and wake this scan. Its
                # stamp is never synthesized or replaced by receipt time.
                self.joint_condition.wait(timeout=remaining)
        now = current_now
        stamps = np.asarray([item[0] for item in history])
        index = int(np.searchsorted(stamps, scan_stamp, side='left'))
        if index < len(history) and abs(stamps[index]-scan_stamp) < 1e-8:
            selected = [history[index]]
            q = selected[0][1].copy()
        elif 0 < index < len(history):
            selected = history[index-1:index+1]
            before, after = selected
            if after[0]-before[0] > .05+1e-9:
                raise ValueError('self-filter joint interpolation gap exceeds 50 ms')
            weight = (scan_stamp-before[0])/(after[0]-before[0])
            q = (1.-weight)*before[1]+weight*after[1]
        else:
            selected = [history[0] if index == 0 else history[-1]]
            if abs(selected[0][0]-scan_stamp) > .02+1e-9:
                raise ValueError('self-filter has no joint sample within 20 ms of scan')
            q = selected[0][1].copy()
        if any(not -.005 <= now-stamp <= .15 or abs(scan_stamp-stamp) > .15
               for stamp, _ in selected):
            raise ValueError('self-filter interpolation source samples expired')
        return q, min(stamp for stamp, _ in selected)

    def filter_self_cloud(self, msg):
        """Remove robot hits at the scan epoch; missing evidence emits no frame."""
        try:
            now = rospy.get_time(); stamp = msg.header.stamp.to_sec()
            source_frame = msg.header.frame_id
            if (not math.isfinite(stamp) or stamp <= 0. or
                    not -.005 <= now-stamp <= .15):
                raise ValueError('self-filter scan source time invalid or stale '
                                 '(stamp=%.9f now=%.9f age_ms=%.3f)' %
                                 (stamp, now, (now-stamp)*1000.))
            q, joint_stamp = self.joints_at_scan(stamp, now)
            # No Time(0) fallback is permitted on the planning/perception path.
            scan_tf = self.tf_buffer.lookup_transform('world', source_frame,
                                                       msg.header.stamp, rospy.Duration(.02))
            base_tf = self.tf_buffer.lookup_transform('world', 'base_link',
                                                       msg.header.stamp, rospy.Duration(.02))
            rotation, translation = rigid_transform(scan_tf)
            base_rotation, base_position = rigid_transform(base_tf)
            yaw = math.atan2(base_rotation[1, 0], base_rotation[0, 0])
            # Native physical spheres already include the base ground offset;
            # their established planar model takes world x/y/yaw, not z twice.
            spheres = self.model.body_spheres(q, [base_position[0], base_position[1], yaw])
            outside_self_spheres(np.empty((0, 3)), spheres)  # Validate even an empty source frame.
            ordered = sorted(msg.fields, key=lambda field: field.offset)
            indices = {}; index = 0
            for field in ordered:
                if field.name in ('x', 'y', 'z'):
                    if field.count != 1:
                        raise ValueError('self-filter requires scalar xyz fields')
                    indices[field.name] = index
                index += field.count
            xyz = [indices[name] for name in ('x', 'y', 'z')]
            mapping_active = (getattr(self, 'mapping_pub', None) is not None and
                              self.mapping_pub.get_num_connections() > 0)
            kept_world = []
            kept = []; batch = []
            def filter_batch(rows):
                points = np.asarray([[row[i] for i in xyz] for row in rows], dtype=float)
                world = points @ rotation.T+translation
                mask = outside_self_spheres(world, spheres, padding=.01)
                kept.extend(row for row, keep in zip(rows, mask) if keep)
                if mapping_active:
                    kept_world.append(world[mask])
            for row in point_cloud2.read_points(msg, skip_nans=False):
                batch.append(row)
                if len(batch) == 2048:
                    filter_batch(batch); batch = []
            if batch:
                filter_batch(batch)
            result = point_cloud2.create_cloud(msg.header, msg.fields, kept)
            if self.target_frame != source_frame:
                target_tf = self.tf_buffer.lookup_transform(self.target_frame, source_frame,
                                                             msg.header.stamp, rospy.Duration(.02))
                rigid_transform(target_tf)
                result = tf2_sensor_msgs.do_transform_cloud(result, target_tf)
            # TF headers may describe an interpolated transform cache sample;
            # never renew the source cloud's time or report a different epoch.
            result.header.stamp = msg.header.stamp
            result.header.frame_id = self.target_frame
            end = rospy.get_time()
            with self.joint_lock:
                feedback_valid = self.q is not None
            if (not feedback_valid or not -.005 <= end-stamp <= .15 or
                    not -.005 <= end-joint_stamp <= .15):
                raise ValueError('self-filter feedback expired before cloud publication')
            self.pub.publish(result)
            if mapping_active:
                try:
                    world = np.concatenate(kept_world) if kept_world else np.empty((0, 3))
                    mapping_sensor_origin = translation
                    output_xyz = cloud_xyz(result)
                    if self.target_frame != source_frame:
                        # The transformed full cloud is float32. Group its actual
                        # output points, rather than pre-rounding source points.
                        output_tf = self.tf_buffer.lookup_transform(
                            'world', self.target_frame, msg.header.stamp, rospy.Duration(.02))
                        output_rotation, output_translation = rigid_transform(output_tf)
                        world = output_xyz @ output_rotation.T+output_translation
                        mapping_sensor_origin = output_translation
                    projection_mask = (np.all(np.abs(output_xyz) < self.mapping_local_range, axis=1) &
                                       (world[:, 2] >= self.mapping_origin[2]+.15))
                    indices = mapping_voxel_indices(
                        world, mapping_sensor_origin, self.mapping_resolution, self.mapping_origin,
                        self.mapping_size, self.mapping_max_ray_length, projection_mask)
                    mapping = mapping_xyz_cloud(result, indices)
                    end = rospy.get_time()
                    with self.joint_lock:
                        feedback_valid = self.q is not None
                    if (not feedback_valid or not -.005 <= end-stamp <= .15 or
                            not -.005 <= end-joint_stamp <= .15):
                        raise ValueError('mapping self-filter evidence expired before publication')
                    self.mapping_pub.publish(mapping)
                except (ValueError, IndexError, TypeError, tf2_ros.TransformException) as exc:
                    rospy.logwarn_throttle(2., 'Mapping cloud suppressed: %s', exc)
        except (ValueError, KeyError, IndexError, TypeError, tf2_ros.TransformException) as exc:
            rospy.logwarn_throttle(2., 'Robot self-filter suppressed cloud: %s', exc)

    def cloud_cb(self, msg):
        if self.filter_self:
            self.filter_self_cloud(msg)
            return
        try:
            trans = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id,
                                                    msg.header.stamp, rospy.Duration(0.02))
        except tf2_ros.ExtrapolationException:
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.target_frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(0.02))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as exc:
                rospy.logdebug_throttle(2.0, "Transform lookup failed: %s", exc)
                return
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as exc:
            rospy.logdebug_throttle(2.0, "Transform lookup failed: %s", exc)
            return

        try:
            cloud_out = tf2_sensor_msgs.do_transform_cloud(msg, trans)
            self.pub.publish(cloud_out)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Point cloud transform failed: %s", exc)

if __name__ == '__main__':
    CloudTransformer()
    rospy.spin()
