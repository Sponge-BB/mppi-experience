#include "delivery_fast_query.hpp"
#include <Eigen/Core>
#include <chrono>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

namespace {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
struct Overlay {
  Cloud::Ptr cloud{new Cloud};
  std::vector<float> radii;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  Eigen::Vector3d minimum, maximum;
  double inflation = 0.0;
  std::uint64_t sequence = 1, refresh = 1;
  void finish() {
    cloud->width = cloud->size(); cloud->height = 1; cloud->is_dense = true;
    minimum = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    maximum = -minimum;
    inflation = 0.0;
    for (std::size_t i = 0; i < cloud->size(); ++i) {
      const auto& p = cloud->at(i);
      Eigen::Vector3d v(p.x, p.y, p.z);
      minimum = minimum.cwiseMin(v); maximum = maximum.cwiseMax(v);
      inflation = std::max(inflation, static_cast<double>(radii.at(i)));
    }
    tree.setInputCloud(cloud);
  }
  void add(float x, float y, float z, float r) {
    pcl::PointXYZ p; p.x=x; p.y=y; p.z=z;
    cloud->push_back(p); radii.push_back(r);
  }
};
struct Answer {
  bool found = false;
  double distance = 0.0;
  Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
  int index = -1;
};

// Same bounded-domain and minimizer rules as frozen GridMap, using the actual
// installed PCL tree for both paths, not a Python approximation of the tree.
Answer query(const Overlay& o, const Eigen::Vector3d& p, double max_distance,
             double influence, bool fast) {
  Answer out; out.distance = max_distance;
  if (!p.allFinite() || std::isnan(max_distance) || max_distance <= 0.0)
    return out;
  const double near = std::min(max_distance, influence);
  const double limit = near + o.inflation;
  for (int axis=0; axis<3; ++axis)
    if (p(axis) < o.minimum(axis)-limit || p(axis) > o.maximum(axis)+limit)
      return out;
  pcl::PointXYZ q; q.x=p.x(); q.y=p.y(); q.z=p.z();
  std::vector<int> ids; std::vector<float> d2;
  int n = fast ? delivery_fast_query::candidates(
    &o, o.cloud, o.radii, o.sequence, o.refresh, o.tree, q, limit, true, ids, d2)
    : o.tree.radiusSearch(q, limit, ids, d2);
  if (n <= 0) return out;
  double best=std::numeric_limits<double>::infinity(), geometric=0.0;
  for (std::size_t i=0; i<ids.size(); ++i) {
    const double d=std::sqrt(std::max(0.0, static_cast<double>(d2[i])));
    const double clear=d-static_cast<double>(o.radii.at(ids[i]));
    if (clear<best) { best=clear; geometric=d; out.index=ids[i]; }
  }
  if (out.index<0 || best>near) { out.index=-1; return out; }
  out.found=true; out.distance=best;
  if (geometric>1e-6) {
    const auto& p0=o.cloud->at(out.index);
    out.gradient=(p-Eigen::Vector3d(p0.x,p0.y,p0.z))/geometric;
  }
  return out;
}

std::size_t checked=0;
void compare(const Overlay& o, const Eigen::Vector3d& p,
             double max_distance=1.5, double influence=1.2) {
  const Answer a=query(o,p,max_distance,influence,false);
  const Answer b=query(o,p,max_distance,influence,true);
  if (a.found!=b.found || a.distance!=b.distance || a.index!=b.index ||
      (a.gradient-b.gradient).squaredNorm()!=0.0)
    throw std::runtime_error("legacy/fast distance, gradient, or minimizer mismatch");
  ++checked;
}
}

int main() {
  try {
    // The same process changes phases only between paused solver calls.
    unsetenv("DELIVERY_FAST_QUERY");
    if (delivery_fast_query::enabled()) throw std::runtime_error("unset flag enabled");
    for (const char* value : {"0", "1", "0", "1", "10", ""}) {
      setenv("DELIVERY_FAST_QUERY", value, 1);
      const bool expected = value[0]=='1' && value[1]=='\0';
      if (delivery_fast_query::enabled()!=expected)
        throw std::runtime_error("same-process runtime flag did not switch");
    }
    unsetenv("DELIVERY_FAST_QUERY");
    std::mt19937 rng(11); std::uniform_real_distribution<float> u(-2.0f,2.0f);
    for (bool unequal : {false,true}) {
      Overlay o;
      for(int i=0;i<1200;++i) o.add(u(rng),u(rng),u(rng),
        unequal ? 0.02f+0.01f*(i%17) : 0.12f);
      o.finish();
      for(int i=0;i<4000;++i)
        compare(o,Eigen::Vector3d(u(rng),u(rng),u(rng)),0.01+0.2*(i%12));
      compare(o,Eigen::Vector3d(100,0,0));
      compare(o,Eigen::Vector3d::Zero(),0.0);
      compare(o,Eigen::Vector3d::Zero(),-1.0);
    }
    Overlay tie;
    tie.add(-1,0,0,0.2);tie.add(1,0,0,0.2);tie.add(0,1,0,0.2);
    tie.finish(); compare(tie,Eigen::Vector3d::Zero());
    compare(tie,Eigen::Vector3d(-1,0,0));  // overlap, zero gradient
    Overlay edge; edge.add(0,0,0,0);edge.finish();
    compare(edge,Eigen::Vector3d(1,0,0),1.0,1.0); // strict radius boundary
    compare(edge,Eigen::Vector3d(std::nextafter(1.0f,0.0f),0,0),1.0,1.0);
    compare(edge,Eigen::Vector3d(std::nextafter(1.0f,2.0f),0,0),1.0,1.0);
    Overlay changed;
    changed.add(0,0,0,0.01f);changed.add(0.6f,0,0,0.01f);changed.finish();
    compare(changed,Eigen::Vector3d(0.1,0,0));
    // Same sequence, rebuilt overlay at new time: radius cache must invalidate.
    changed.radii[1]=0.6f;++changed.refresh;changed.finish();
    compare(changed,Eigen::Vector3d(0.1,0,0));
    // New cloud but same sequence+time must also invalidate; retained ownership
    // means allocator address reuse cannot make a different cloud look equal.
    changed.cloud.reset(new Cloud);changed.radii.clear();
    changed.add(1,1,0,0.04f);changed.add(1.5f,1,0,0.04f);changed.finish();
    compare(changed,Eigen::Vector3d(1.2,1,0));
    Overlay duplicate;duplicate.add(0,0,0,.1);duplicate.add(0,0,0,.1);
    duplicate.finish();compare(duplicate,Eigen::Vector3d(.2,0,0));

    Overlay dense;
    for(int i=0;i<12000;++i) dense.add(.3*u(rng),.3*u(rng),.3*u(rng),.12f);
    dense.finish();
    std::vector<Eigen::Vector3d> positions;
    for(int i=0;i<1000;++i) positions.emplace_back(.2*u(rng),.2*u(rng),.2*u(rng));
    for(const auto& p:positions)compare(dense,p);
    double timings[2]; volatile double checksum=0.0;
    for(int fast=0;fast<2;++fast) {
      const auto start=std::chrono::steady_clock::now();
      for(const auto& p:positions) checksum+=query(dense,p,1.5,1.2,fast).distance;
      timings[fast]=std::chrono::duration<double,std::milli>(
        std::chrono::steady_clock::now()-start).count();
    }
    std::cout<<"{\"passed\":true,\"runtime_flag_toggle_passed\":true,\"exact_comparisons\":"<<checked
      <<",\"synthetic_dense_1000_queries_legacy_ms\":"<<timings[0]
      <<",\"synthetic_dense_1000_queries_fast_ms\":"<<timings[1]
      <<",\"checksum\":"<<checksum<<"}\n";
  } catch(const std::exception& e) {
    std::cerr<<e.what()<<'\n';return 1;
  }
}
