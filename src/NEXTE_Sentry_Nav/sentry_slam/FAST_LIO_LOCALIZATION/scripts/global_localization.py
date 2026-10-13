#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import _thread
import threading
import time

import open3d as o3d
import rospy
import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import tf
import tf.transformations

global_map = None
global_map_stamp = rospy.Time(0)
global_map_lock = threading.RLock()
pose_lock = threading.RLock()
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None
cur_scan_stamp = None
ICP_LOG_THROTTLE = 10.0


def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def msg_to_array(pc_msg):
    pc_array = ros_numpy.numpify(pc_msg)
    pc = np.zeros([len(pc_array), 3])
    pc[:, 0] = pc_array['x']
    pc[:, 1] = pc_array['y']
    pc[:, 2] = pc_array['z']
    return pc


def registration_at_scale(pc_scan, pc_map, initial, scale):
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
        1.0 * scale, initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_MAX_ITERATION)
    )

    return result_icp.transformation, result_icp.fitness


def msg_stamp_or_now(msg):
    stamp = msg.header.stamp
    if stamp == rospy.Time(0):
        stamp = rospy.Time.now()
    return stamp


def data_is_fresh(scan_stamp, odom_msg):
    if scan_stamp is None or odom_msg is None:
        return False

    now = rospy.Time.now()
    odom_stamp = msg_stamp_or_now(odom_msg)
    if MAX_DATA_AGE > 0:
        scan_age = (now - scan_stamp).to_sec()
        odom_age = (now - odom_stamp).to_sec()
        if scan_age > MAX_DATA_AGE or odom_age > MAX_DATA_AGE:
            rospy.logwarn_throttle(
                1.0,
                'Skip ICP: stale data, scan_age=%.3fs odom_age=%.3fs',
                scan_age, odom_age)
            return False

    if MAX_SCAN_ODOM_DT > 0:
        dt = abs((scan_stamp - odom_stamp).to_sec())
        if dt > MAX_SCAN_ODOM_DT:
            rospy.logwarn_throttle(
                1.0,
                'Skip ICP: scan/odom not synced, dt=%.3fs > %.3fs',
                dt, MAX_SCAN_ODOM_DT)
            return False

    return True


def yaw_from_matrix(trans):
    return tf.transformations.euler_from_matrix(trans)[2]


def normalize_angle(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def project_to_planar_correction(trans):
    if not PLANAR_LOCALIZATION:
        return trans

    raw_roll, raw_pitch, raw_yaw = tf.transformations.euler_from_matrix(trans)
    planar = np.eye(4)
    planar[:3, :3] = tf.transformations.euler_matrix(0.0, 0.0, raw_yaw)[:3, :3]
    planar[0, 3] = trans[0, 3]
    planar[1, 3] = trans[1, 3]
    planar[2, 3] = MAP_TO_ODOM_Z

    rospy.loginfo_throttle(
        ICP_LOG_THROTTLE,
        'Planar ICP correction: raw rpy=(%.3f, %.3f, %.3f), raw_z=%.3f -> z=%.3f, keep x/y/yaw only',
        raw_roll, raw_pitch, raw_yaw, trans[2, 3], MAP_TO_ODOM_Z)
    return planar


def project_to_planar_delta(trans):
    if not PLANAR_LOCALIZATION:
        return trans

    _, _, raw_yaw = tf.transformations.euler_from_matrix(trans)
    planar = np.eye(4)
    planar[:3, :3] = tf.transformations.euler_matrix(0.0, 0.0, raw_yaw)[:3, :3]
    planar[0, 3] = trans[0, 3]
    planar[1, 3] = trans[1, 3]
    return planar


def correction_step_is_reasonable(candidate, allow_large_correction):
    if allow_large_correction:
        return True

    with pose_lock:
        current_map_to_odom = T_map_to_odom.copy()
    delta = np.matmul(inverse_se3(current_map_to_odom), candidate)
    translation_step = np.linalg.norm(delta[:2, 3])
    yaw_step = abs(normalize_angle(yaw_from_matrix(delta)))

    if MAX_CORRECTION_TRANSLATION_STEP > 0 and translation_step > MAX_CORRECTION_TRANSLATION_STEP:
        rospy.logwarn_throttle(
            ICP_LOG_THROTTLE,
            'Reject ICP correction: translation jump %.3fm > %.3fm',
            translation_step, MAX_CORRECTION_TRANSLATION_STEP)
        return False

    if MAX_CORRECTION_YAW_STEP > 0 and yaw_step > MAX_CORRECTION_YAW_STEP:
        rospy.logwarn_throttle(
            ICP_LOG_THROTTLE,
            'Reject ICP correction: yaw jump %.3frad > %.3frad',
            yaw_step, MAX_CORRECTION_YAW_STEP)
        return False

    rospy.loginfo_throttle(
        ICP_LOG_THROTTLE,
        'Accept ICP correction: translation_step=%.3fm yaw_step=%.3frad',
        translation_step, yaw_step)
    return True


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    data = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)


def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    # 当前scan原点的位姿
    T_odom_to_base_link = pose_to_mat(cur_odom)
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_map_to_base_link)

    # 把地图转换到lidar系下
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 将视角内的地图点提取出来
    if FOV > 3.14:
        # 环状lidar 仅过滤距离
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        # 非环状lidar 保前视范围
        # FOV_FAR>x>0 且角度小于FOV
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    submap_points = global_map_in_map[indices[0], :3]
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(submap_points)

    # 发布fov内点云
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, submap_points[::10])

    return global_map_in_FOV


def global_localization(pose_estimation, allow_large_correction=False):
    global global_map, cur_scan, cur_scan_stamp, cur_odom, T_map_to_odom
    if cur_scan is None or cur_odom is None:
        rospy.logwarn_throttle(1.0, 'Skip ICP: waiting for scan and odometry.')
        return False
    with global_map_lock:
        map_snapshot = global_map
        map_stamp_snapshot = global_map_stamp
    if map_snapshot is None:
        rospy.logwarn_throttle(1.0, 'Skip ICP: waiting for global map.')
        return False

    scan_tobe_mapped = copy.copy(cur_scan)
    scan_stamp = cur_scan_stamp
    odom_for_match = copy.copy(cur_odom)
    if not data_is_fresh(scan_stamp, odom_for_match):
        return False

    rospy.loginfo_throttle(ICP_LOG_THROTTLE, 'Global localization by scan-to-map matching......')

    tic = time.time()

    global_map_in_FOV = crop_global_map_in_FOV(map_snapshot, pose_estimation, odom_for_match)
    if len(global_map_in_FOV.points) < MIN_SUBMAP_POINTS:
        rospy.logwarn_throttle(
            1.0,
            'Skip ICP: submap has only %d points, check initial pose/FOV/map frame.',
            len(global_map_in_FOV.points))
        return False

    # 粗配准
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                    scale=1)
    toc = time.time()
    rospy.loginfo_throttle(ICP_LOG_THROTTLE, 'ICP time: %.3fs', toc - tic)

    # 当全局定位成功时才更新map2odom
    if fitness > LOCALIZATION_TH:
        # Ground robot: do not let 3D ICP roll/pitch/z tilt the TF tree.
        candidate = project_to_planar_correction(transformation)
        if not correction_step_is_reasonable(candidate, allow_large_correction):
            return False

        with pose_lock:
            T_map_to_odom = candidate.copy()

        # 发布map_to_odom
        map_to_odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(candidate)
        quat = tf.transformations.quaternion_from_matrix(candidate)
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        map_to_odom.header.stamp = scan_stamp
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = str(map_stamp_snapshot.to_nsec())
        map_to_odom.pose.covariance[0] = fitness
        pub_map_to_odom.publish(map_to_odom)
        return True
    else:
        rospy.logwarn('Not match!!!!')
        rospy.logwarn('{}'.format(transformation))
        rospy.logwarn('fitness score:{}'.format(fitness))
        return False


def voxel_down_sample(pcd, voxel_size):
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size)
    except:
        # for opend3d 0.7 or lower
        pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down


def initialize_global_map(pc_msg):
    global global_map, global_map_stamp

    if global_map_stamp != rospy.Time(0) and pc_msg.header.stamp == global_map_stamp:
        return

    new_map = o3d.geometry.PointCloud()
    new_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3])
    new_map = voxel_down_sample(new_map, MAP_VOXEL_SIZE)
    with global_map_lock:
        global_map = new_map
        global_map_stamp = pc_msg.header.stamp
    rospy.loginfo(
        'Global map received. stamp=%d points=%d',
        pc_msg.header.stamp.to_nsec(), len(new_map.points))


def cb_map_correction_applied(odom_msg):
    global T_map_to_odom
    correction = project_to_planar_delta(pose_to_mat(odom_msg))
    with pose_lock:
        T_map_to_odom = np.matmul(correction, T_map_to_odom)
        residual = T_map_to_odom.copy()

    translation_residual = np.linalg.norm(residual[:2, 3])
    yaw_residual = abs(normalize_angle(yaw_from_matrix(residual)))
    rospy.loginfo(
        'Map correction applied to ICP initial guess. residual xy=%.3fm yaw=%.3frad',
        translation_residual, yaw_residual)


def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg


def cb_save_cur_scan(pc_msg):
    global cur_scan, cur_scan_stamp
    # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    scan_viz_msg = copy.copy(pc_msg)
    scan_viz_msg.header.frame_id = 'camera_init'
    pub_pc_in_map.publish(scan_viz_msg)

    # 转换为pcd
    # fastlio给的field有问题 处理一下
    pc_for_numpy = copy.copy(pc_msg)
    if len(pc_for_numpy.fields) >= 8:
        pc_for_numpy.fields = [pc_for_numpy.fields[0], pc_for_numpy.fields[1], pc_for_numpy.fields[2],
                               pc_for_numpy.fields[4], pc_for_numpy.fields[5], pc_for_numpy.fields[6],
                               pc_for_numpy.fields[3], pc_for_numpy.fields[7]]
    pc = msg_to_array(pc_for_numpy)

    cur_scan = o3d.geometry.PointCloud()
    cur_scan.points = o3d.utility.Vector3dVector(pc[:, :3])
    cur_scan_stamp = msg_stamp_or_now(pc_msg)


def thread_localization():
    global T_map_to_odom
    rate = rospy.Rate(FREQ_LOCALIZATION)
    while not rospy.is_shutdown():
        # 每隔一段时间进行全局定位
        rate.sleep()
        # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
        with pose_lock:
            pose_estimation = T_map_to_odom.copy()
        global_localization(pose_estimation)


if __name__ == '__main__':
    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    MAP_VOXEL_SIZE = rospy.get_param('~map_voxel_size', 0.4)
    SCAN_VOXEL_SIZE = rospy.get_param('~scan_voxel_size', 0.1)
    ICP_MAX_ITERATION = rospy.get_param('~icp_max_iteration', 20)

    # Global localization frequency (HZ)
    FREQ_LOCALIZATION = rospy.get_param('~localization_frequency', 1.5)

    # The threshold of global localization,
    # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
    LOCALIZATION_TH = rospy.get_param('~localization_threshold', 0.80)

    # FOV(rad), modify this according to your LiDAR type
    FOV = rospy.get_param('~fov', 6.28)

    # The farthest distance(meters) within FOV
    FOV_FAR = rospy.get_param('~fov_far', 30.0)
    MAX_DATA_AGE = rospy.get_param('~max_data_age', 1.0)
    MAX_SCAN_ODOM_DT = rospy.get_param('~max_scan_odom_dt', 0.15)
    MIN_SUBMAP_POINTS = rospy.get_param('~min_submap_points', 100)
    MAX_CORRECTION_TRANSLATION_STEP = rospy.get_param('~max_correction_translation_step', 0.15)
    MAX_CORRECTION_YAW_STEP = rospy.get_param('~max_correction_yaw_step', 0.15)
    PLANAR_LOCALIZATION = rospy.get_param('~planar_localization', True)
    MAP_TO_ODOM_Z = rospy.get_param('~map_to_odom_z', 0.0)
    ICP_LOG_THROTTLE = rospy.get_param('~icp_log_throttle_sec', 10.0)
    INITIAL_POSE_TOPIC = rospy.get_param('~initial_pose_topic', '/initialpose')
    MAP_CORRECTION_APPLIED_TOPIC = rospy.get_param(
        '~map_correction_applied_topic', '/map_correction_applied')

    rospy.loginfo(
        'ICP params: freq=%.2fHz threshold=%.2f map_voxel=%.2f scan_voxel=%.2f '
        'max_scan_odom_dt=%.3fs max_data_age=%.3fs max_step=%.3fm max_yaw=%.3frad '
        'planar=%s map_to_odom_z=%.3f log_throttle=%.1fs',
        FREQ_LOCALIZATION, LOCALIZATION_TH, MAP_VOXEL_SIZE, SCAN_VOXEL_SIZE,
        MAX_SCAN_ODOM_DT, MAX_DATA_AGE,
        MAX_CORRECTION_TRANSLATION_STEP, MAX_CORRECTION_YAW_STEP,
        PLANAR_LOCALIZATION, MAP_TO_ODOM_Z, ICP_LOG_THROTTLE)

    # publisher
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)

    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
    rospy.Subscriber(MAP_CORRECTION_APPLIED_TOPIC, Odometry, cb_map_correction_applied, queue_size=1)

    # 初始化全局地图
    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('/map', PointCloud2))
    rospy.Subscriber('/map', PointCloud2, initialize_global_map, queue_size=1)

    # 初始化
    initial_pose = None
    while not initialized and not rospy.is_shutdown():
        if initial_pose is None:
            rospy.logwarn('Waiting for initial pose on %s....', INITIAL_POSE_TOPIC)
            pose_msg = rospy.wait_for_message(INITIAL_POSE_TOPIC, PoseWithCovarianceStamped)
            initial_pose = pose_to_mat(pose_msg)

        if cur_scan is not None and cur_odom is not None:
            initialized = global_localization(initial_pose, allow_large_correction=True)
            if not initialized:
                rospy.sleep(1.0 / FREQ_LOCALIZATION)
        else:
            rospy.logwarn('First scan or odometry not received!!!!!')
            rospy.sleep(0.5)

    rospy.loginfo('')
    rospy.loginfo('Initialize successfully!!!!!!')
    rospy.loginfo('')
    # 开始定期全局定位
    _thread.start_new_thread(thread_localization, ())

    rospy.spin()
