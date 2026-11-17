#!/usr/bin/python3
from pathlib import Path
import shutil,json
r=Path(__file__).resolve().parent;b=r/'behavior_candidate';b.mkdir(exist_ok=True)
for rel in ('include/mm_remani/dynamic_obstacle_tracker_core.h','src/dynamic_obstacle_tracker_core.cpp','src/dynamic_obstacle_tracker_node.cpp','test/test_dynamic_obstacle_tracker_core.cpp'):
 p=b/rel;p.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(r/'src/mm_remani'/rel,p)
p=b/'src/dynamic_obstacle_tracker_core.cpp';s=p.read_text();s=s.replace('(point - neighbour).squaredNorm()', '(point - neighbour).head<2>().squaredNorm()').replace('(translated - neighbour).squaredNorm()', '(translated - neighbour).head<2>().squaredNorm()')
s=s.replace('        // Query at most 64 observed points against the complete bounded', '        // Qualification tests planar motion, matching the XY history/shift.\n        // Height sampling may change; retain all original XYZ collision points.\n        // Query at most 64 observed points against the complete bounded')
p.write_text(s)
fixture=[json.loads(x) for x in (r/'replay_full_fixture/selected_surfaces.jsonl').open()]
lines=['// Recorded point sets from 2026-11-17; no fitted tolerances.\n','#include "../src/dynamic_obstacle_tracker_core.cpp"\n','#include "test_dynamic_obstacle_tracker_core.cpp"\n','namespace mm_remani { namespace {\n']
for i,x in enumerate(fixture):
 lines.append('TEST(RecordedPlanarSurface, OriginalRejection%d) {\n'%i)
 for field in ('previous','current'):
  lines.append('const std::vector<Eigen::Vector3f> '+field+'{\n')
  lines.extend('Eigen::Vector3f('+','.join(repr(v) for v in p)+'),\n' for p in x[field]);lines.append('};\n')
 lines.append('const auto before_previous=previous; const auto before_current=current;\n')
 lines.append('SurfaceTranslationDiagnostic evidence;\nEXPECT_TRUE(surfaceSupportsTranslation(previous,current,Eigen::Vector2d('+repr(x['dx'])+','+repr(x['dy'])+'),'+repr(x['tolerance'])+',&evidence));\n')
 lines.append('EXPECT_TRUE(evidence.forward.translated_close); EXPECT_TRUE(evidence.reverse.translated_close);\n')
 lines.append('for (std::size_t j=0;j<previous.size();++j) EXPECT_TRUE(previous[j]==before_previous[j]);\nfor (std::size_t j=0;j<current.size();++j) EXPECT_TRUE(current[j]==before_current[j]);\n}\n')
lines.append('''
std::vector<Eigen::Vector3f> fixtureVerticalFace(float x, float dz) {
 std::vector<Eigen::Vector3f> p;
 for(float y:{-.1f,.1f})for(float z:{.3f,.5f,.7f,.9f})p.emplace_back(x,y,z+dz);
 return p;
}
TEST(PlanarSurfaceBoundary, VerticalOnlyChangeIsNotPlanarMotion) {
 auto a=fixtureVerticalFace(1.f,0.f),b=fixtureVerticalFace(1.f,.35f);
 EXPECT_FALSE(surfaceSupportsTranslation(a,b,Eigen::Vector2d::Zero(),.05));
}
TEST(PlanarSurfaceBoundary, StaticFootprintRemainsStationaryDespiteHeightSampling) {
 auto a=fixtureVerticalFace(1.f,0.f),b=fixtureVerticalFace(1.f,.35f);
 SurfaceTranslationDiagnostic d;
 EXPECT_FALSE(surfaceSupportsTranslation(a,b,Eigen::Vector2d(.25,0),.05,&d));
 EXPECT_FALSE(d.forward.stationary_distinct);
}
TEST(PlanarSurfaceBoundary, PlanarMotionWithHeightChangeKeepsFullCollisionGeometry) {
 auto a=fixtureVerticalFace(1.f,0.f),b=fixtureVerticalFace(1.25f,.35f),saved=b;
 EXPECT_TRUE(surfaceSupportsTranslation(a,b,Eigen::Vector2d(.25,0),.05));
 for(std::size_t i=0;i<b.size();++i)EXPECT_TRUE(b[i]==saved[i]);
 EXPECT_GT(b.front().z(),a.front().z());
}
TEST(PlanarSurfaceBoundary, SlidingStaticVisibilityWithHeightChangeStaysRejected) {
 std::vector<Eigen::Vector3f>a,b;
 for(int i=0;i<=50;++i){a.emplace_back(.02f*i,0.f,.5f);b.emplace_back(.02f*i+.2f,0.f,.8f);}
 EXPECT_FALSE(surfaceSupportsTranslation(a,b,Eigen::Vector2d(.2,0),.05));
}
} } // namespace
''')
(b/'test/test_recorded_planar_surface.cpp').write_text(''.join(lines))
