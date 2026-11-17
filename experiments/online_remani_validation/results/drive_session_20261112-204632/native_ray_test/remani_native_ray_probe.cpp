#include <plan_env/raycast.h>
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>
#include <algorithm>

namespace {
constexpr int NX=300, NY=200, NZ=60, SIZE=NX*NY*NZ;
const Eigen::Vector3d lower(-7.5,-5.,0.), upper(7.5,5.,3.);
std::vector<std::uint32_t> end_flags(SIZE,0), traverse_flags(SIZE,0);
std::vector<int> target_lookup(SIZE,-1);
std::uint32_t epoch=0;
int targets=0;
bool inmap(const Eigen::Vector3d &v) {
 return (v.array()>=lower.array()+1.e-4).all() && (v.array()<=upper.array()-1.e-4).all();
}
int address(const Eigen::Vector3d &v) {
 if(!inmap(v)) return -1;
 const Eigen::Vector3i a=((v-lower)*20.).array().floor().cast<int>();
 const int x=a.x()*NY*NZ+a.y()*NZ+a.z(); return x>=0 && x<SIZE ? x:-1;
}
}

extern "C" void configure_targets(const int *xyz, int n) {
 std::fill(target_lookup.begin(),target_lookup.end(),-1); targets=n;
 for(int i=0;i<n;i++) target_lookup[xyz[3*i]*NY*NZ+xyz[3*i+1]*NZ+xyz[3*i+2]]=i;
}

// Columns: endpoint hits, endpoint misses, full DDA misses, deduplicated DDA
// misses, suppressed by endpoint dedup, suppressed after shared traversal,
// first endpoint-hit point index+1, first full-free-ray point index+1.
extern "C" int replay_frame(const double *xyz, int n, const double *o,
                             std::uint64_t *out, std::uint64_t *totals) {
 std::fill(out,out+targets*8,0);std::fill(totals,totals+9,0);
 if(++epoch==0) { std::fill(end_flags.begin(),end_flags.end(),0);std::fill(traverse_flags.begin(),traverse_flags.end(),0);++epoch; }
 const Eigen::Vector3d origin(o[0],o[1],o[2]); if(!inmap(origin))return -1;
 RayCaster raycaster;Eigen::Vector3d ray_pt;
 for(int i=0;i<n;i++) {
  Eigen::Vector3d p(xyz[3*i],xyz[3*i+1],xyz[3*i+2]);
  if(!p.allFinite() || p.z()<.15)continue;
  ++totals[0]; bool hit=true;
  if(!inmap(p)) {
   hit=false;++totals[1];const Eigen::Vector3d d=p-origin;double t=1000000.;
   for(int a=0;a<3;a++)if(std::abs(d[a])>0) {double hi=(upper[a]-origin[a])/d[a],lo=(lower[a]-origin[a])/d[a];if(hi>0 && hi<t)t=hi;if(lo>0 && lo<t)t=lo;}
   p=origin+(t-1.e-3)*d;
  }
  const double range=(p-origin).norm();
  if(range>4.5) {hit=false;++totals[2];p=origin+(p-origin)/range*4.5;}
  const int end_address=address(p);int target=end_address<0?-1:target_lookup[end_address];
  if(target>=0) {++out[target*8+(hit?0:1)];if(hit && out[target*8+6]==0)out[target*8+6]=i+1;if(!hit && out[target*8+7]==0)out[target*8+7]=i+1;}
  bool active=true,end_suppressed=false;
  if(end_address>=0) {if(end_flags[end_address]==epoch) {active=false;end_suppressed=true;++totals[3];}else end_flags[end_address]=epoch;}
  raycaster.setInput(p/.05,origin/.05);
  int steps=0;
  while(raycaster.step(ray_pt)) {
   if(++steps>2000)return -2;
   ++totals[4];const Eigen::Vector3d cell=(ray_pt+Eigen::Vector3d::Constant(.5))*.05;const int a=address(cell);
   const int t=a<0?-1:target_lookup[a];
   if(t>=0) {++out[t*8+2];if(out[t*8+7]==0)out[t*8+7]=i+1;if(active)++out[t*8+3];else ++out[t*8+(end_suppressed?4:5)];}
   if(active) {++totals[5];if(a>=0) {if(traverse_flags[a]==epoch) {active=false;++totals[6];}else traverse_flags[a]=epoch;}}
  }
 }
 return 0;
}
