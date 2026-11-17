#include <Eigen/Core>
#include <memory>
#include <vector>
#include <cassert>
#include <iostream>
struct Occupancy {std::vector<double> occupancy; std::vector<char> inflated;};
struct View {std::shared_ptr<Occupancy> occupancy; unsigned long observation=3,esdf=3;};
struct Map {struct Params {Eigen::Vector3i map_voxel_num_{4,3,5};double min_occupancy_log_=.4;}mp_;
std::shared_ptr<View> view=std::make_shared<View>(); mutable int reads=0;
std::shared_ptr<View> readSnapshot()const{++reads;return view;}
  bool copyOccupancyEvidence(const Eigen::Vector3i& lo,
                            const Eigen::Vector3i& hi,
                            std::vector<unsigned char>& output) const {
    const auto view = readSnapshot();
    const auto size = mp_.map_voxel_num_;
    if (!view || !view->occupancy || view->observation != view->esdf ||
        (lo.array()<0).any() || (hi.array()>=size.array()).any() ||
        (lo.array()>hi.array()).any()) return false;
    const auto& raw = view->occupancy->occupancy;
    const auto& inflated = view->occupancy->inflated;
    if (raw.size()!=inflated.size() || raw.size()!=std::size_t(size.prod()))
      return false;
    output.resize(std::size_t((hi-lo+Eigen::Vector3i::Ones()).prod()));
    std::size_t destination = 0;
    for (int x=lo.x(); x<=hi.x(); ++x)
      for (int y=lo.y(); y<=hi.y(); ++y) {
        std::size_t source = (std::size_t(x)*size.y()+y)*size.z()+lo.z();
        for (int z=lo.z(); z<=hi.z(); ++z,++source)
          output[destination++] = (raw[source]>mp_.min_occupancy_log_ ? 1 : 0) |
                                  (inflated[source]==1 ? 2 : 0);
      }
    return true;
  }
};
int main(){Map m; m.view->occupancy=std::make_shared<Occupancy>();auto &o=*m.view->occupancy;o.occupancy.resize(60);o.inflated.resize(60);
for(int i=0;i<60;++i){o.occupancy[i]=(i%3==0)?.9:0.;o.inflated[i]=(i%7==0)?1:0;}
std::vector<unsigned char> got,expected;
for(int i=0;i<60;++i){int x=i/15,y=(i/5)%3,z=i%5;if(x>=1&&z>=1)expected.push_back((i%3==0?1:0)|(i%7==0?2:0));}
assert(m.copyOccupancyEvidence({1,0,1},{3,2,4},got));assert(got==expected);assert(m.reads==1);
auto saved=got;assert(!m.copyOccupancyEvidence({-1,0,0},{3,2,4},got));assert(got==saved);
m.view->esdf=2;assert(!m.copyOccupancyEvidence({0,0,0},{3,2,4},got));assert(got==saved);m.view->esdf=3;
o.inflated.pop_back();assert(!m.copyOccupancyEvidence({0,0,0},{3,2,4},got));assert(got==saved);
std::cout<<"occupancy order/bitmask, one snapshot lookup, bounds, generation, storage checks passed\n";
}
