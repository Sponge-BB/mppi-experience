"""Measured-motion detector changes for isolated V12 builds only.

The inherited real profile required >= 0.22 m/s, excluding slowly translating
targets.  This profile uses the same identity/shape, duration, displacement,
efficiency and direction checks at a lower entry speed; it has no actor input.
"""


def profile_overrides():
    """ROS parameter overrides, applied uniformly to every V12 scene."""
    return {
        '/dynamic_obstacle_tracker/dynamic_enter_speed': 0.12,
        '/dynamic_obstacle_tracker/dynamic_exit_speed': 0.08,
        '/dynamic_obstacle_tracker/velocity_deadband': 0.08,
        '/dynamic_obstacle_tracker/minimum_dynamic_displacement': 0.06,
        '/dynamic_obstacle_tracker/motion_history_duration': 0.80,
    }


def _replace(source, old, new):
    if source.count(old) != 1:
        raise ValueError('V12 perception snapshot marker changed: ' + old[:100])
    return source.replace(old, new)


def transform_core(source):
    """Patch a copied tracker core, retaining all existing health/shape gates.

    Dwell alone cannot make a track background when it currently has coherent
    translational evidence.  A bounded flickering residual still becomes a
    background anchor, and existing anchors still require spatial release.
    """
    old = '''    const bool coherent_motion =
        track.hit_count >= parameters_.min_confirmed_hits &&
        valid_step_count >= parameters_.minimum_dynamic_hits &&
        (parameters_.minimum_motion_cluster_points <= 0 ||
         detection.point_count >= static_cast<std::size_t>(
             parameters_.minimum_motion_cluster_points)) &&
        history_duration >= parameters_.minimum_motion_duration &&
        net_displacement >= parameters_.minimum_dynamic_displacement &&
        robust_speed >= parameters_.dynamic_enter_speed &&
        motion_efficiency >= parameters_.minimum_motion_efficiency &&
        direction_consistency >=
            parameters_.minimum_direction_consistency_fraction &&
        step_speed_plausible &&
        !inside_stationary_background;'''
    evidence = old.replace('coherent_motion', 'translational_motion_evidence')
    evidence = evidence.replace('step_speed_plausible &&\n        !inside_stationary_background;',
                                'step_speed_plausible;')
    marker = '''    const bool stable_stationary_background ='''
    source = _replace(source, marker, evidence + '\n\n' + marker)
    source = _replace(source, old, '''    const bool coherent_motion =
        translational_motion_evidence && !inside_stationary_background;''')
    source = _replace(source, '''    else if (stamp_seconds - track.background_dwell_start_seconds >=
             parameters_.stationary_background_dwell_duration)''',
        '''    else if (!translational_motion_evidence &&
             stamp_seconds - track.background_dwell_start_seconds >=
                 parameters_.stationary_background_dwell_duration)''')
    return source


def transform_node(source):
    """Cluster on voxels, estimate motion from the associated original points.

    A 0.1 m sensor-frame voxel can jump or remain stationary while a slow
    rigid target moves centimeters per frame. Its mean is unsuitable for
    motion confirmation. Source-voxel membership already exists for the
    complete collision export; reuse it without another association or truth.
    """
    marker='''    std::vector<DynamicDetection> detections;'''
    return _replace(source,marker,'''    std::vector<int> motion_membership(downsampled->size(), -1);
    for (std::size_t i = 0; i < clusters.size(); ++i)
      for (int index : clusters[i].source_voxels)
        if (index >= 0 && index < static_cast<int>(motion_membership.size()))
          motion_membership[index] = static_cast<int>(i);
    std::vector<std::vector<Eigen::Vector3f>> original_motion_points(clusters.size());
    for (const pcl::PointXYZ &point : input->points)
    {
      if (!pcl::isFinite(point)) continue;
      const int voxel_index = voxel.getCentroidIndex(point);
      if (voxel_index < 0 || voxel_index >= static_cast<int>(motion_membership.size())) continue;
      const int cluster_index = motion_membership[voxel_index];
      if (cluster_index < 0) continue;
      const Eigen::Vector3d value = rotation * Eigen::Vector3d(point.x, point.y, point.z) + translation;
      if (value.allFinite()) original_motion_points[cluster_index].push_back(value.cast<float>());
    }
    for (std::size_t i = 0; i < clusters.size(); ++i)
      if (original_motion_points[i].size() >= static_cast<std::size_t>(minimum_cluster_points_))
        clusters[i].tracking_detection.centroid = estimateStableMotionCenter(
            original_motion_points[i], stable_motion_center_parameters_);

'''+marker)


CHANGES = [
    'uniform measured-motion profile admits coherent slow translations (entry 0.12 m/s, exit 0.08 m/s)',
    'minimum net displacement 0.06 m and 0.80 s bounded history; identity, direction, efficiency and point-count checks retained',
    'do not learn a dwell-background anchor from a currently coherent translating track; existing anchor release remains required',
    'complete classified collision surfaces and health/freshness gates unchanged; no actor-truth input',
    'cluster with voxels but estimate stable motion centers from original same-frame member points',
]
