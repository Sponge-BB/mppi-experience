#!/usr/bin/python3
"""Render existing MPPI predictions outside the control process.

This node subscribes only while a visualization display is connected. It never
publishes goals, actuator commands, enable requests, or tracking faults.
"""
import os
for _key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_key] = '1'

import math
from pathlib import Path
import sys
import threading

sys.path.insert(0, str(Path(__file__).resolve().parent/'mppi'))

import numpy as np
import rospkg
import rospy
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as RosPath
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from active_arm_model import PiperArmModel
from prediction_visualization import decode_prediction


def _marker(stamp, namespace, marker_id, kind, rgb, alpha=1.):
    marker = Marker()
    marker.header.frame_id = 'world'
    marker.header.stamp = rospy.Time.from_sec(float(stamp))
    marker.ns, marker.id = namespace, marker_id
    marker.type, marker.action = kind, Marker.ADD
    marker.pose.orientation.w = 1.
    marker.color.r, marker.color.g, marker.color.b = rgb
    marker.color.a = alpha
    marker.lifetime = rospy.Duration(.75)
    return marker


def _points(values):
    return [Point(*map(float, row)) for row in values]


def render_prediction(model, decoded):
    """Return (MarkerArray, base Path, arm Marker), without ROS publication.

    Candidate states and validity come directly from the controller. Only the
    selected trajectory's FK is evaluated here, for display; no collision or
    optimization is rerun. Sphere radii are physical model radii.
    """
    stamp = decoded['stamp']
    candidates, valid = decoded['candidates'], decoded['valid']
    selected = decoded['selected']
    status = int(decoded['status'])
    rgb = {1: (0., .65, .85), 0: (1., .35, .05), 2: (.65, .25, .8)}[status]
    label = {1: 'FIRST COMMAND DELIVERED', 0: 'REJECTED (not delivered)',
             2: 'CANDIDATE ONLY'}[status]
    markers = MarkerArray()
    clear = _marker(stamp, 'mppi_prediction', 0, Marker.LINE_LIST, rgb)
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)

    for feasible, color, name in ((True, (.15, .65, .15), 'feasible'),
                                   (False, (.9, .15, .15), 'infeasible')):
        marker = _marker(stamp, 'mppi_candidates_'+name, 0,
                         Marker.LINE_LIST, color, .55)
        marker.scale.x = .012
        paths = candidates[valid == feasible]
        if paths.size and paths.shape[1] > 1:
            # Candidate base positions lie on the ground plane; their arm
            # motions are shown by the selected 3-D whole-body ghosts below.
            xy = np.stack((paths[:, :-1, :2], paths[:, 1:, :2]), axis=2)
            xy = xy.reshape((-1, 2))
            marker.points = _points(np.c_[xy, np.full(len(xy), .025)])
        markers.markers.append(marker)

    path = RosPath()
    path.header.frame_id = 'world'
    path.header.stamp = rospy.Time.from_sec(float(stamp))
    arm = _marker(stamp, 'mppi_arm', 0, Marker.LINE_STRIP, rgb)
    arm.scale.x = .016
    if len(selected):
        for row in selected:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = map(float, row[:2])
            pose.pose.position.z = .035
            pose.pose.orientation.z = math.sin(float(row[2])/2)
            pose.pose.orientation.w = math.cos(float(row[2])/2)
            path.poses.append(pose)
        chosen = _marker(stamp, 'mppi_selected_base', 0,
                         Marker.LINE_STRIP, rgb)
        chosen.scale.x = .035
        chosen.points = _points(np.c_[selected[:, :2], np.full(len(selected), .035)])
        markers.markers.append(chosen)

        spheres = model.body_spheres_batch(selected[:, 5:11], selected[:, :3])
        arm.points = _points(spheres[:, -4, :3])
        # SPHERE_LIST has one radius per marker. Grouping equal physical radii
        # keeps the entire robot visible without hundreds of Marker objects.
        radii = np.unique(spheres[0, :, 3])
        indices = np.unique(np.linspace(0, len(selected)-1,
                                       min(4, len(selected))).round().astype(int))
        for ghost_id, index in enumerate(indices):
            alpha = .26-.16*ghost_id/max(1, len(indices)-1)
            for radius_id, radius in enumerate(radii):
                marker = _marker(stamp, 'mppi_whole_body_future',
                                 ghost_id*len(radii)+radius_id,
                                 Marker.SPHERE_LIST, rgb, alpha)
                marker.scale.x = marker.scale.y = marker.scale.z = float(2*radius)
                marker.points = _points(spheres[index, spheres[index, :, 3] == radius, :3])
                markers.markers.append(marker)
    else:
        arm.action = Marker.DELETE

    annotation = _marker(stamp, 'mppi_prediction_info', 0,
                         Marker.TEXT_VIEW_FACING, (.15, .15, .15))
    annotation.scale.z = .07
    anchor = selected[0, :2] if len(selected) else (
        candidates[0, 0, :2] if len(candidates) else np.zeros(2))
    annotation.pose.position.x, annotation.pose.position.y = map(float, anchor)
    annotation.pose.position.z = 1.35
    steps = candidates.shape[1] if len(candidates) else len(selected)
    horizon = max(0, steps-1)*decoded['dt']
    annotation.text = ('MPPI %s | future prediction %.2f s\n'
                       'green: feasible; red: infeasible | validity includes braking'
                       % (label, horizon))
    markers.markers.append(annotation)
    return markers, path, arm


class PredictionVisualizer:
    def __init__(self):
        packages = rospkg.RosPack()
        model_file = Path(packages.get_path('my_description'))/'robot/mm.urdf'
        proxy_file = Path(packages.get_path('mm_remani'))/'config/mppi_shoulder_wrist_proxy.json'
        self.model = PiperArmModel.from_parameters(
            rospy.get_param('/remani_planner_node'), model_file, proxy_file)
        self.marker_pub = rospy.Publisher('/mppi/prediction_markers', MarkerArray, queue_size=1)
        self.path_pub = rospy.Publisher('/mppi/predicted_base', RosPath, queue_size=1)
        self.arm_pub = rospy.Publisher('/mppi/predicted_arm', Marker, queue_size=1)
        self.lock = threading.Lock()
        self.latest = None
        self.subscriber = None
        self.visible = False
        self.timer = rospy.Timer(rospy.Duration(.5), self.tick)

    def receive(self, message):
        # Numerical message storage is bounded by TCPROS queue_size=1. Parsing,
        # FK and Marker construction happen only in this independent process.
        with self.lock:
            self.latest = message.data

    def clear(self):
        if not self.visible:
            return
        path = RosPath()
        path.header.frame_id = 'world'
        path.header.stamp = rospy.Time.now()
        marker = _marker(path.header.stamp.to_sec(), 'mppi_arm', 0,
                         Marker.LINE_STRIP, (0., 0., 0.))
        marker.action = Marker.DELETE
        clear = _marker(path.header.stamp.to_sec(), 'mppi_prediction', 0,
                        Marker.LINE_LIST, (0., 0., 0.))
        clear.action = Marker.DELETEALL
        self.path_pub.publish(path)
        self.arm_pub.publish(marker)
        self.marker_pub.publish(MarkerArray(markers=[clear]))
        self.visible = False

    def tick(self, _event):
        wanted = any(pub.get_num_connections() for pub in
                     (self.marker_pub, self.path_pub, self.arm_pub))
        if not wanted:
            if self.subscriber is not None:
                self.subscriber.unregister()
                self.subscriber = None
            with self.lock:
                self.latest = None
            self.clear()
            return
        if self.subscriber is None:
            self.subscriber = rospy.Subscriber(
                '/mppi/prediction_rollouts', numpy_msg(Float64MultiArray),
                self.receive, queue_size=1, buff_size=256*1024)
        with self.lock:
            data = self.latest
        if data is None:
            self.clear()
            return
        try:
            prediction = decode_prediction(data)
            age = rospy.get_time()-prediction['stamp']
            if age < -.05 or age > .8:
                self.clear()
                return
            markers, path, arm = render_prediction(self.model, prediction)
        except (ValueError, IndexError, FloatingPointError) as exc:
            rospy.logwarn_throttle(5., 'Ignoring invalid MPPI visualization: %s', exc)
            self.clear()
            return
        if self.marker_pub.get_num_connections():
            self.marker_pub.publish(markers)
        if self.path_pub.get_num_connections():
            self.path_pub.publish(path)
        if self.arm_pub.get_num_connections():
            self.arm_pub.publish(arm)
        self.visible = True


if __name__ == '__main__':
    rospy.init_node('mppi_prediction_visualizer')
    PredictionVisualizer()
    rospy.spin()
