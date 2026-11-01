#!/usr/bin/env python3
"""Generate an isolated frozen-map restore accessor using apply_patch.

The historical helper is read-only. The added method does not change GridMap's
layout and does not initialize ROS subscriptions, timers, or mapping workers.
"""
import hashlib
from pathlib import Path
import subprocess

HERE = Path(__file__).resolve().parent
SOURCE = HERE/'builds/endpoint_time_20261030_01/source/candidate_audit.hpp'
TARGET = HERE/'results/root_cause_20261030_01/native_source/candidate_audit.hpp'

METHOD = r'''
  // Restore only the frozen native planning-query state. No initMap/ROS I/O.
  static void restoreSnapshot(GridMap& map, const std::string& json_path){
    using boost::property_tree::ptree;
    static_assert(sizeof(double)==8 && sizeof(float)==4, "snapshot scalar ABI mismatch");
    ptree meta;boost::property_tree::read_json(json_path,meta);
    if(meta.get<std::string>("schema")!="candidate_native_map_v1" ||
       meta.get<std::string>("esdf_dtype")!="native_endian_float64" ||
       meta.get<std::string>("overlay_dtype")!="native_endian_float32_xyzw_radius" ||
       meta.get<std::string>("esdf_linear_order")!="(x*ny+y)*nz+z")
      throw std::runtime_error("unsupported frozen native map snapshot format");
    auto read_vector=[&](const std::string& key){
      Eigen::Vector3d result;int axis=0;
      for(const auto& item:meta.get_child(key)){
        if(axis>=3)throw std::runtime_error("snapshot vector has more than three entries: "+key);
        const std::string raw=item.second.data();
        // Empty overlays can legitimately export +/-inf AABB as JSON null.
        result[axis++]=raw=="null"?0.0:item.second.get_value<double>();
      }
      if(axis!=3)throw std::runtime_error("snapshot vector has wrong dimension: "+key);
      return result;
    };
    auto resolve_path=[&](const std::string& key){
      const std::string value=meta.get<std::string>(key);
      if(!value.empty() && value.front()=='/')return value;
      const auto slash=json_path.find_last_of('/');
      return (slash==std::string::npos?std::string():json_path.substr(0,slash+1))+value;
    };
    auto read_gzip=[&](const std::string& key,void* storage,std::size_t bytes,
                       const std::string& hash_key){
      const std::string path=resolve_path(key);
      gzFile file=gzopen(path.c_str(),"rb");
      if(!file)throw std::runtime_error("cannot open frozen native map buffer: "+path);
      struct Close {gzFile f;~Close(){if(f)gzclose(f);}} closer{file};
      auto* out=static_cast<unsigned char*>(storage);
      std::uint64_t hash=14695981039346656037ULL;
      std::size_t at=0;
      while(at<bytes){
        const unsigned chunk=static_cast<unsigned>(std::min<std::size_t>(bytes-at,1U<<20));
        const int got=gzread(file,out+at,chunk);
        if(got<=0)throw std::runtime_error("truncated/corrupt frozen native map buffer: "+path);
        for(int i=0;i<got;++i){hash^=out[at+i];hash*=1099511628211ULL;}
        at+=static_cast<std::size_t>(got);
      }
      unsigned char extra;
      if(gzread(file,&extra,1)!=0)
        throw std::runtime_error("oversize/corrupt frozen native map buffer: "+path);
      int error=Z_OK;gzerror(file,&error);
      if(error!=Z_OK && error!=Z_STREAM_END)
        throw std::runtime_error("gzip integrity failure in frozen native map buffer: "+path);
      if(hash!=std::stoull(meta.get<std::string>(hash_key)))
        throw std::runtime_error("FNV64 mismatch in frozen native map buffer: "+path);
    };
    // These structs contain no ROS subscriptions, threads, or mutexes.
    map.mp_=MappingParameters{};map.md_=MappingData{};
    map.mp_.map_origin_=read_vector("map_origin");
    map.mp_.map_size_=read_vector("map_size");
    map.mp_.map_min_boundary_=read_vector("map_min_boundary");
    map.mp_.map_max_boundary_=read_vector("map_max_boundary");
    const Eigen::Vector3d voxels=read_vector("map_voxel_num");
    map.mp_.map_voxel_num_=voxels.cast<int>();
    map.mp_.resolution_=meta.get<double>("resolution");
    if(!(map.mp_.resolution_>0.0) || !voxels.allFinite() ||
       (map.mp_.map_voxel_num_.array()<=0).any() ||
       (voxels-map.mp_.map_voxel_num_.cast<double>()).norm()!=0.0)
      throw std::runtime_error("invalid frozen native map geometry");
    map.mp_.resolution_inv_=1.0/map.mp_.resolution_;
    const std::size_t count=meta.get<std::size_t>("esdf_count");
    const std::size_t expected=static_cast<std::size_t>(map.mp_.map_voxel_num_[0])*
        static_cast<std::size_t>(map.mp_.map_voxel_num_[1])*
        static_cast<std::size_t>(map.mp_.map_voxel_num_[2]);
    if(count!=expected)throw std::runtime_error("frozen ESDF voxel/count mismatch");
    map.md_.distance_buffer_all_.resize(count);
    read_gzip("esdf_path",map.md_.distance_buffer_all_.data(),count*sizeof(double),"esdf_fnv1a64");
    map.md_.dynamic_observation_seq_=meta.get<std::uint64_t>("raw_seq");
    map.md_.esdf_observation_seq_=meta.get<std::uint64_t>("esdf_seq");
    map.md_.local_bound_min_=Eigen::Vector3i::Zero();
    map.md_.local_bound_max_=map.mp_.map_voxel_num_-Eigen::Vector3i::Ones();
    map.md_.occupancy_bound_min_=map.md_.local_bound_min_;
    map.md_.occupancy_bound_max_=map.md_.local_bound_max_;
    const std::size_t overlay_count=meta.get<std::size_t>("overlay_count");
    std::vector<float> overlay(overlay_count*4);
    read_gzip("overlay_path",overlay.data(),overlay.size()*sizeof(float),"overlay_fnv1a64");
    std::lock_guard<std::mutex> lock(map.dynamic_point_overlay_mutex_);
    map.dynamic_point_overlay_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    map.dynamic_point_overlay_cloud_->points.resize(overlay_count);
    map.dynamic_point_overlay_cloud_->width=static_cast<std::uint32_t>(overlay_count);
    map.dynamic_point_overlay_cloud_->height=1;
    map.dynamic_point_overlay_cloud_->is_dense=true;
    map.dynamic_point_overlay_radii_.resize(overlay_count);
    for(std::size_t i=0;i<overlay_count;++i){
      auto& point=map.dynamic_point_overlay_cloud_->points[i];
      point.x=overlay[4*i];point.y=overlay[4*i+1];point.z=overlay[4*i+2];
      map.dynamic_point_overlay_radii_[i]=overlay[4*i+3];
    }
    if(overlay_count)map.dynamic_point_overlay_kdtree_.setInputCloud(map.dynamic_point_overlay_cloud_);
    map.mp_.dynamic_point_overlay_enable_=meta.get<bool>("overlay_enabled");
    map.dynamic_point_overlay_ready_=meta.get<bool>("overlay_ready");
    map.dynamic_point_overlay_frozen_=meta.get<bool>("overlay_frozen");
    map.dynamic_point_overlay_sequence_=meta.get<std::uint64_t>("overlay_sequence");
    map.dynamic_point_overlay_frozen_sequence_=meta.get<std::uint64_t>("overlay_frozen_sequence");
    map.dynamic_point_overlay_refresh_time_.fromNSec(meta.get<std::uint64_t>("overlay_refresh_nsec"));
    map.dynamic_point_overlay_inflation_=meta.get<double>("overlay_inflation");
    map.dynamic_point_overlay_min_=read_vector("overlay_min");
    map.dynamic_point_overlay_max_=read_vector("overlay_max");
    map.mp_.dynamic_point_overlay_influence_distance_=meta.get<double>("overlay_influence_distance");
    map.mp_.dynamic_point_overlay_horizon_=meta.get<double>("overlay_horizon");
    map.mp_.dynamic_point_overlay_time_step_=meta.get<double>("overlay_time_step");
    map.mp_.dynamic_point_overlay_extra_inflation_=meta.get<double>("overlay_extra_inflation");
    map.mp_.dynamic_point_overlay_max_inflation_=meta.get<double>("overlay_max_inflation");
  }
'''


def main():
    source = SOURCE.read_text()
    before = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    includes = '#include <boost/property_tree/ptree.hpp>\n#include <boost/property_tree/json_parser.hpp>\n'
    result = source.replace('#include <Eigen/Dense>\n', '#include <Eigen/Dense>\n'+includes, 1)
    marker = 'class GridMapParallelTestAccessor {\n public:\n'
    if result.count(marker) != 1:
        raise RuntimeError('unexpected historical accessor structure')
    result = result.replace(marker, marker+METHOD, 1)
    if TARGET.exists():
        if TARGET.read_text() != result:
            raise RuntimeError('refusing to overwrite a different generated accessor')
    else:
        patch = '*** Begin Patch\n*** Add File: '+str(TARGET)+'\n'
        patch += ''.join('+'+line+'\n' for line in result.splitlines())
        patch += '*** End Patch\n'
        subprocess.run(['apply_patch'], input=patch, text=True, check=True)
    if hashlib.sha256(SOURCE.read_bytes()).hexdigest() != before:
        raise RuntimeError('historical source changed')
    print(str(TARGET))
    print('historical_sha256='+before)
    print('generated_sha256='+hashlib.sha256(TARGET.read_bytes()).hexdigest())


if __name__ == '__main__':
    main()
