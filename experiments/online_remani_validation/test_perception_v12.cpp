#include <mm_remani/dynamic_obstacle_tracker_core.h>
#include <gtest/gtest.h>
#include <cmath>

namespace {
using mm_remani::DynamicDetection;
using mm_remani::DynamicObstacleTrackerCore;

DynamicObstacleTrackerCore::Parameters profile() {
  DynamicObstacleTrackerCore::Parameters p;
  p.association_gate=.35; p.association_position_allowance=.12;
  p.position_alpha=.65; p.velocity_alpha=.30;
  p.regression_velocity_weight=.75; p.motion_direction_alpha=.30;
  p.velocity_deadband=.08; p.dynamic_enter_speed=.12;
  p.dynamic_exit_speed=.08; p.minimum_dynamic_displacement=.06;
  p.direction_consistency_cosine=.60; p.minimum_motion_duration=.25;
  p.motion_history_duration=.80; p.minimum_motion_efficiency=.70;
  p.minimum_direction_consistency_fraction=.75;
  p.min_confirmed_hits=4; p.minimum_dynamic_hits=3;
  p.minimum_motion_cluster_points=15;
  return p;
}

DynamicDetection measured(double x, double y=0.) {
  DynamicDetection d; d.centroid={x,y,.80};
  d.extent={.40,.40,1.20}; d.point_count=195; return d;
}

TEST(V12Perception, CoherentSlowTargetsQualifyWithinBoundedEvidenceWindow) {
  for(double speed:{.17,.18,.20,.26}) {
    DynamicObstacleTrackerCore tracker(profile());
    double first=-1.;std::uint32_t id=0;
    for(int i=0;i<40;++i) {
      auto out=tracker.update({measured(3.-speed*.1*i)},10.+.1*i);
      ASSERT_EQ(out.size(),1u);if(!i)id=out[0].track_id;
      ASSERT_EQ(out[0].track_id,id);
      if(out[0].motion_qualified&&first<0.)first=.1*i;
      if(i>=10)EXPECT_TRUE(out[0].motion_qualified);
    }
    ASSERT_GE(first,0.);EXPECT_LE(first,.60);
    EXPECT_EQ(tracker.stationaryBackgroundAnchorCount(),0u);
  }
}

TEST(V12Perception, InheritedEntryThresholdExcludesMeasuredSlowTranslation) {
  auto p=profile();p.dynamic_enter_speed=.22;
  p.dynamic_exit_speed=.10;p.velocity_deadband=.10;
  p.minimum_dynamic_displacement=.10;p.motion_history_duration=.60;
  DynamicObstacleTrackerCore tracker(p);
  for(int i=0;i<45;++i) {
    auto out=tracker.update({measured(3.-.18*.1*i)},10.+.1*i);
    ASSERT_EQ(out.size(),1u);
    EXPECT_FALSE(out[0].motion_qualified);
  }
}

TEST(V12Perception, SmallScanNoiseDoesNotHideSlowNetTranslation) {
  DynamicObstacleTrackerCore tracker(profile());bool qualified=false;
  for(int i=0;i<30;++i) {
    const double jitter=.004*std::sin(i*1.7);
    auto out=tracker.update({measured(3.-.18*.1*i+jitter,.003*std::cos(i))},20.+.1*i);
    ASSERT_EQ(out.size(),1u);qualified|=out[0].motion_qualified;
    if(i>=12)EXPECT_TRUE(out[0].motion_qualified);
  }
  EXPECT_TRUE(qualified);EXPECT_EQ(tracker.stationaryBackgroundAnchorCount(),0u);
}

TEST(V12Perception, StationaryAndBoundedFlickeringReturnsRemainBackground) {
  for(double amplitude:{.005,.08}) {
    DynamicObstacleTrackerCore tracker(profile());
    for(int i=0;i<50;++i) {
      auto out=tracker.update({measured(2.+(i%2?amplitude:-amplitude))},30.+.1*i);
      ASSERT_EQ(out.size(),1u);EXPECT_FALSE(out[0].motion_qualified);
    }
    EXPECT_GE(tracker.stationaryBackgroundAnchorCount(),1u);
  }
}

TEST(V12Perception, StoppedTargetLosesMotionQualification) {
  DynamicObstacleTrackerCore tracker(profile());
  for(int i=0;i<12;++i)tracker.update({measured(3.-.18*.1*i)},40.+.1*i);
  bool lost=false;
  for(int i=0;i<10;++i) {
    auto out=tracker.update({measured(3.-.18*1.1)},41.2+.1*i);
    ASSERT_EQ(out.size(),1u);if(!out[0].motion_qualified)lost=true;
  }
  EXPECT_TRUE(lost);
}

TEST(V12Perception, CoherentTranslationCannotBecomeDwellBackground) {
  auto p=profile();p.stationary_background_dwell_radius=.5;
  p.stationary_background_dwell_duration=2.;
  DynamicObstacleTrackerCore tracker(p);
  for(int i=0;i<26;++i) {
    auto out=tracker.update({measured(3.-.18*.1*i)},50.+.1*i);
    ASSERT_EQ(out.size(),1u);
    if(i>=8)EXPECT_TRUE(out[0].motion_qualified);
  }
  // In the original code .45m net translation lies inside the .5m dwell
  // radius, so dwell incorrectly labels a moving target as background.
  EXPECT_EQ(tracker.stationaryBackgroundAnchorCount(),0u);
}
}
