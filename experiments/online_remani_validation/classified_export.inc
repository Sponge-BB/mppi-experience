  // Experiment-only atomic full-resolution cloud with detector voxel labels.
  // Unknown/static samples are retained. No geometric target-circle erasure.
  void publishClassified(const ros::Time &stamp,
      const pcl::PointCloud<pcl::PointXYZ> &input,
      pcl::VoxelGrid<pcl::PointXYZ> &voxel, std::size_t count,
      const Eigen::Matrix3d &rotation, const Eigen::Vector3d &translation,
      const std::vector<ClusterDetection> &clusters,
      const std::vector<TrackedDetection> &tracked)
  {
    std::vector<int> membership(count, -1);
    for (std::size_t i=0; i<tracked.size(); ++i) {
      const auto &track=tracked[i];
      if (!track.confirmed || !track.motion_qualified ||
          track.detection_index>=clusters.size()) continue;
      for (const int index: clusters[track.detection_index].source_voxels)
        if (index>=0 && std::size_t(index)<count) membership[index]=int(i);
    }
    sensor_msgs::PointCloud2 output;
    output.header.stamp=stamp; output.header.frame_id=world_frame_;
    output.height=1; output.is_bigendian=false; output.is_dense=true;
    output.fields={makeField("x",0,sensor_msgs::PointField::FLOAT32),
      makeField("y",4,sensor_msgs::PointField::FLOAT32),
      makeField("z",8,sensor_msgs::PointField::FLOAT32),
      makeField("vx",12,sensor_msgs::PointField::FLOAT32),
      makeField("vy",16,sensor_msgs::PointField::FLOAT32),
      makeField("vz",20,sensor_msgs::PointField::FLOAT32),
      makeField("confidence",24,sensor_msgs::PointField::FLOAT32),
      makeField("track_id",28,sensor_msgs::PointField::UINT32),
      makeField("dynamic",32,sensor_msgs::PointField::UINT32)};
    output.point_step=36; output.data.reserve(input.size()*36);
    for (const auto &p: input) {
      if (!pcl::isFinite(p)) continue;
      Eigen::Vector3d world=rotation*Eigen::Vector3d(p.x,p.y,p.z)+translation;
      if (!world.allFinite()) continue;
      int index=voxel.getCentroidIndex(p);
      int member=index>=0 && std::size_t(index)<count ? membership[index] : -1;
      Eigen::Vector3f velocity=Eigen::Vector3f::Zero();
      std::uint32_t id=0, dynamic=0; float confidence=0;
      if (member>=0) {
        const auto &track=tracked[member];
        velocity=track.velocity.cast<float>(); id=track.track_id;
        confidence=float(track.confidence); dynamic=1;
      }
      std::size_t off=output.data.size(); output.data.resize(off+36);
      for (int a=0;a<3;++a) {
        writeValue(output.data,off+4*a,float(world[a]));
        writeValue(output.data,off+12+4*a,velocity[a]);
      }
      writeValue(output.data,off+24,confidence);
      writeValue(output.data,off+28,id); writeValue(output.data,off+32,dynamic);
      ++output.width;
    }
    output.row_step=output.width*output.point_step;
    classified_pub_.publish(output);
  }
