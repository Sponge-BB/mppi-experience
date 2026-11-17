#include <mm_remani/dynamic_obstacle_tracker_core.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include <iostream>
#include <iomanip>

namespace mm_remani
{
namespace
{

DynamicDetection detection(double x, double y = 0.0, double z = 0.5)
{
  DynamicDetection value;
  value.centroid = Eigen::Vector3d(x, y, z);
  // Four real observations of the visible front face of a rigid object.
  // Motion qualification must not depend on a synthetic centroid alone.
  for (const double dy : {-0.1, 0.1})
  {
    for (const double dz : {-0.1, 0.1})
      value.motion_surface.emplace_back(x, y + dy, z + dz);
  }
  return value;
}

DynamicDetection shapedDetection(
    double x, double extent_x, std::size_t point_count)
{
  DynamicDetection value = detection(x);
  value.extent = Eigen::Vector3d(extent_x, 0.30, 0.80);
  value.point_count = point_count;
  return value;
}

std::vector<Eigen::Vector3f> articulatedTargetPoints(
    double center_x, double leg_side)
{
  std::vector<Eigen::Vector3f> points;
  // A symmetric middle/upper body supplies a stable motion anchor.
  for (const double z : {0.70, 1.00})
  {
    for (const double x_offset : {-0.10, 0.10})
    {
      for (const double y_offset : {-0.10, 0.10})
      {
        points.emplace_back(
            static_cast<float>(center_x + x_offset),
            static_cast<float>(y_offset), static_cast<float>(z));
      }
    }
  }
  points.emplace_back(static_cast<float>(center_x), -0.08f, 1.40f);
  points.emplace_back(static_cast<float>(center_x), 0.08f, 1.40f);

  // Deliberately over-represent whichever leg is currently visible.  The
  // complete collision shape must retain these points, but the target motion
  // centre must not follow them sideways.
  for (int index = 0; index < 10; ++index)
  {
    const double x_offset = 0.01 * static_cast<double>(index % 3);
    points.emplace_back(
        static_cast<float>(center_x + x_offset),
        static_cast<float>(leg_side),
        static_cast<float>(0.02 * static_cast<double>(index % 2)));
  }
  return points;
}

DynamicDetection surfaceDetection(const std::vector<Eigen::Vector3f> &points)
{
  DynamicDetection value;
  value.centroid = estimateStableMotionCenter(
      points, StableMotionCenterParameters(), nullptr, &value.motion_surface);
  value.point_count = points.size();
  if (!points.empty())
  {
    Eigen::Vector3f minimum = points.front();
    Eigen::Vector3f maximum = points.front();
    for (const Eigen::Vector3f &point : points)
    {
      minimum = minimum.cwiseMin(point);
      maximum = maximum.cwiseMax(point);
    }
    value.extent = (maximum - minimum).cast<double>();
  }
  return value;
}

// Frozen real-robot profile recorded 2026-11-16. No tuning to make a fixture pass.
DynamicObstacleTrackerCore::Parameters productionMotionParameters()
{
  DynamicObstacleTrackerCore::Parameters p;
  p.association_gate = 0.35;
  p.association_position_allowance = 0.12;
  p.association_shape_weight = 0.20;
  p.maximum_extent_change = 0.45;
  p.maximum_point_count_ratio = 4.0;
  p.position_alpha = 0.65;
  p.velocity_alpha = 0.30;
  p.regression_velocity_weight = 0.75;
  p.motion_direction_alpha = 0.30;
  p.velocity_deadband = 0.10;
  p.max_speed = 2.0;
  p.dynamic_enter_speed = 0.22;
  p.dynamic_exit_speed = 0.10;
  p.minimum_dynamic_displacement = 0.10;
  p.surface_motion_tolerance = 0.05;  // node: max(.03, .5 * voxel_leaf=.10)
  p.direction_consistency_cosine = 0.60;
  p.minimum_motion_duration = 0.25;
  p.motion_history_duration = 0.60;
  p.minimum_motion_efficiency = 0.70;
  p.minimum_direction_consistency_fraction = 0.75;
  p.maximum_motion_step_speed_ratio = 1.25;
  p.min_dt = 0.03;
  p.max_dt = 0.35;
  p.min_confirmed_hits = 4;
  p.minimum_dynamic_hits = 3;
  p.minimum_motion_cluster_points = 15;
  p.minimum_stationary_hits = 3;
  p.minimum_stationary_background_hits = 5;
  p.stationary_background_minimum_duration = 0.50;
  p.stationary_background_maximum_speed = 0.10;
  p.stationary_background_maximum_displacement = 0.06;
  p.stationary_background_merge_radius = 0.20;
  p.stationary_background_release_radius = 0.25;
  p.stationary_background_dwell_duration = 2.0;
  p.stationary_background_dwell_radius = 0.30;
  p.maximum_stationary_background_anchors = 128;
  p.max_missed_frames = 3;
  return p;
}

DynamicDetection frontFace(double x, int levels = 10)
{
  std::vector<Eigen::Vector3f> points;
  for (int i = 0; i < levels; ++i)
    for (double y : {-0.10, 0.10})
      points.emplace_back(x, y, 0.20 + 0.06 * i);
  return surfaceDetection(points);
}


void printBits(double value) { std::uint64_t bits; std::memcpy(&bits,&value,sizeof(bits)); std::cout<<std::hex<<bits<<std::dec<<','; }
void sample(DynamicObstacleTrackerCore &tracker,int scene,int frame,const std::vector<DynamicDetection> &input,double stamp)
{
 const auto output=tracker.update(input,stamp);
 std::cout<<scene<<','<<frame<<','<<tracker.trackCount()<<','<<tracker.stationaryBackgroundAnchorCount()<<',';
 printBits(tracker.lastStamp());
 for(const auto &d:output) {
  std::cout<<d.detection_index<<','<<d.track_id<<','<<d.confirmed<<','<<d.motion_qualified<<','<<d.ever_motion_qualified<<',';
  for(int i=0;i<3;++i) printBits(d.filtered_position[i]);
  for(int i=0;i<3;++i) printBits(d.velocity[i]);
  printBits(d.confidence);
 }
 std::cout<<'\n';
}
}} // namespace
int main()
{
 using namespace mm_remani;
 std::cout<<"sizes,"<<sizeof(TrackedDetection)<<','<<sizeof(DynamicObstacleTrackerCore)<<'\n';
 for(int scene=0;scene<6;++scene) {
  DynamicObstacleTrackerCore tracker(productionMotionParameters());
  for(int frame=0;frame<90;++frame) {
   std::vector<DynamicDetection> input;
   if(scene==0) {const double x=2.0+.04*std::min(frame,30);input.push_back(surfaceDetection(articulatedTargetPoints(x,frame%2?-.32:.32)));if(frame==17)input.clear();if(frame>35)input.push_back(frontFace(4.0));}
   if(scene==1) {for(int c=2*frame;c<=2*frame+40;++c)for(float z:{.25f,.40f}) {if(input.empty())input.push_back(DynamicDetection());input[0].motion_surface.emplace_back(.02f*c,0.f,z);}std::vector<Eigen::Vector3f> pts=input[0].motion_surface;input[0]=surfaceDetection(pts);}
   if(scene==2) input.push_back(frontFace(2.+.04*std::max(0,frame-20)));
   if(scene==3) input.push_back(frontFace(2.+.04*frame,5));
   if(scene==4) {if(frame%9!=0)input.push_back(frontFace(2.+.02*frame));if(frame%7==0)input.push_back(frontFace(3.,10));}
   if(scene==5) {if(frame==45)tracker.reset(false);if(frame==75)tracker.reset(true);input.push_back(frontFace(2.+.01*(frame%8)));}
   sample(tracker,scene,frame,input,200.+20*scene+.1*frame);
  }
  sample(tracker,scene,90,{frontFace(9.)},200.+20*scene);
 }
}
