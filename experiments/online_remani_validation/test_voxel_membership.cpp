#include <pcl/filters/voxel_grid.h>
#include <pcl/common/point_tests.h>
#include <cassert>
#include <iostream>
#include <limits>

int main() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>());
  for(int x=-20;x<=20;++x)for(int y=-4;y<=4;++y) {
    input->push_back(pcl::PointXYZ(x*.035f,y*.037f,.35f));
    input->push_back(pcl::PointXYZ(x*.035f+.001f,y*.037f+.001f,.351f));
  }
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setLeafSize(.08f,.08f,.08f);voxel.setSaveLeafLayout(true);voxel.setInputCloud(input);
  pcl::PointCloud<pcl::PointXYZ> reduced;voxel.filter(reduced);
  std::vector<int> counts(reduced.size());
  for(const auto &p:*input) {
    int i=voxel.getCentroidIndex(p);
    assert(i>=0 && size_t(i)<reduced.size());
    auto a=voxel.getGridCoordinates(p.x,p.y,p.z);
    auto b=voxel.getGridCoordinates(reduced[i].x,reduced[i].y,reduced[i].z);
    assert(a==b);++counts[i];
  }
  for(int n:counts)assert(n>0);
  std::cout<<"PASS: "<<input->size()<<" full-resolution points mapped to "<<reduced.size()<<" exact source voxels, including negative coordinates\n";
}
