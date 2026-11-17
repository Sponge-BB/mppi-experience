#pragma once
// Isolated SIL provenance only. Call published() only inside the native
// overlay mutex, after a refresh has actually replaced the overlay fields.
#include <plan_env/dynamic_point_buffer.h>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace temporal_capture_source {
struct Source {
  plan_env::DynamicPointSnapshot snapshot;
  ros::Time refresh_epoch;
};
inline std::mutex& mutex(){static std::mutex value;return value;}
inline std::unordered_map<const void*,std::shared_ptr<const Source>>& sources(){
  static std::unordered_map<const void*,std::shared_ptr<const Source>> value;return value;
}
inline void published(const void* map,const plan_env::DynamicPointSnapshot& snapshot,
                      const ros::Time& refresh_epoch){
  auto value=std::make_shared<Source>();value->snapshot=snapshot;value->refresh_epoch=refresh_epoch;
  std::lock_guard<std::mutex> lock(mutex());sources()[map]=value;
}
// The caller must hold the overlay mutex while comparing sequence/epoch.
// Lock order is always overlay mutex -> provenance mutex.
inline std::shared_ptr<const Source> publishedSource(const void* map){
  std::lock_guard<std::mutex> lock(mutex());const auto it=sources().find(map);
  return it==sources().end()?std::shared_ptr<const Source>{}:it->second;
}
} // namespace temporal_capture_source
