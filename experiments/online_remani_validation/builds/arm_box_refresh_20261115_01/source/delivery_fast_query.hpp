#pragma once

// Experiment-only acceleration for the immutable dynamic swept-point overlay.
// No GridMap member/layout changes. The caller holds its existing overlay mutex.
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <vector>

namespace delivery_fast_query {

inline bool enabled() {
  // Re-read for the paused same-process A/B probe. Its owner switches this only
  // between complete solver calls, never concurrently with an active query.
  const char* p = std::getenv("DELIVERY_FAST_QUERY");
  return p != nullptr && p[0] == '1' && p[1] == '\0';
}

struct Certificate {
  const void* owner = nullptr;
  // Holding one immutable cloud prevents a deleted cloud's address from being
  // reused and mistaken for the cached generation. Space is bounded per thread.
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud;
  std::uint64_t sequence = 0;
  std::uint64_t refresh_ns = 0;
  bool uniform = false;
};

inline bool uniformRadii(const void* owner,
                         const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                         const std::vector<float>& radii,
                         std::uint64_t sequence, std::uint64_t refresh_ns) {
  thread_local Certificate cached;
  // GridMap replaces cloud+radii together under the overlay mutex; neither is
  // subsequently mutated. Same sequence can be rebuilt at a new query time.
  if (cached.owner != owner || cached.cloud.get() != cloud.get() ||
      cached.sequence != sequence || cached.refresh_ns != refresh_ns) {
    cached.owner = owner;
    cached.cloud = cloud;
    cached.sequence = sequence;
    cached.refresh_ns = refresh_ns;
    cached.uniform = cloud && !radii.empty() && radii.size() == cloud->size();
    if (cached.uniform) {
      const float first = radii.front();
      cached.uniform = std::isfinite(first) && first >= 0.0f &&
        std::all_of(radii.begin(), radii.end(), [first](float r) {
          // Exact, not approximate: unequal radii may change the minimizer.
          return r == first;
        });
    }
  }
  return cached.uniform;
}

inline int candidates(const void* owner,
                      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
                      const std::vector<float>& radii,
                      std::uint64_t sequence, std::uint64_t refresh_ns,
                      const pcl::KdTreeFLANN<pcl::PointXYZ>& tree,
                      const pcl::PointXYZ& query, double search_limit,
                      bool accelerate, std::vector<int>& indices,
                      std::vector<float>& squared_distances) {
  if (accelerate && uniformRadii(owner, cloud, radii, sequence, refresh_ns)) {
    // k=2 detects ties, for which we retain the legacy radiusSearch ordering
    // and hence exactly its gradient choice. It is still constant output size.
    const int count = tree.nearestKSearch(query, 2, indices, squared_distances);
    // PCL 1.10 passes float(radius*radius) to FLANN, which uses strict <.
    // Do not turn a boundary point excluded by radiusSearch into a new contact.
    const float radius_squared = static_cast<float>(search_limit * search_limit);
    if (count <= 0 || !(squared_distances.front() < radius_squared)) {
      indices.clear();
      squared_distances.clear();
      return 0;
    }
    if (count == 1 || squared_distances[0] != squared_distances[1]) {
      indices.resize(1);
      squared_distances.resize(1);
      return 1;
    }
  }
  return tree.radiusSearch(query, search_limit, indices, squared_distances);
}

}  // namespace delivery_fast_query
