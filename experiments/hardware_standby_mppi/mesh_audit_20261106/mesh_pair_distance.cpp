// Offline diagnostic only. No ROS, CAN, actuator interfaces or runtime bypass.
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <fcl/BVH/BVH_model.h>
#include <fcl/BV/OBBRSS.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>

using Model = fcl::BVHModel<fcl::OBBRSS>;

std::unique_ptr<Model> mesh(const char* path) {
  Assimp::Importer importer;
  const auto* scene = importer.ReadFile(path, aiProcess_Triangulate |
      aiProcess_JoinIdenticalVertices | aiProcess_PreTransformVertices);
  if (!scene || !scene->HasMeshes()) throw std::runtime_error(importer.GetErrorString());
  std::vector<fcl::Vec3f> vertices;
  std::vector<fcl::Triangle> faces;
  for (unsigned k=0; k<scene->mNumMeshes; ++k) {
    const auto* m=scene->mMeshes[k];
    const auto offset=vertices.size();
    for (unsigned i=0; i<m->mNumVertices; ++i) {
      auto v=m->mVertices[i];
      if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z))
        throw std::runtime_error("nonfinite mesh vertex");
      vertices.emplace_back(v.x,v.y,v.z);
    }
    for (unsigned i=0; i<m->mNumFaces; ++i) {
      const auto& f=m->mFaces[i];
      if (f.mNumIndices!=3) throw std::runtime_error("nontriangle mesh face");
      faces.emplace_back(offset+f.mIndices[0],offset+f.mIndices[1],offset+f.mIndices[2]);
    }
  }
  if (faces.empty() || faces.size()>100000) throw std::runtime_error("mesh size outside audit budget");
  std::unique_ptr<Model> model(new Model);
  if (model->beginModel(faces.size(),vertices.size()) || model->addSubModel(vertices,faces) || model->endModel())
    throw std::runtime_error("BVH build failed");
  return model;
}

fcl::Transform3f transform(const double* a) {
  return fcl::Transform3f(fcl::Matrix3f(a[0],a[1],a[2],a[4],a[5],a[6],a[8],a[9],a[10]),
                          fcl::Vec3f(a[3],a[7],a[11]));
}

int main(int argc,char** argv) {
  try {
    if (argc!=3) throw std::runtime_error("usage: mesh_pair_distance meshA meshB < row-major-3x4-transforms.txt");
    auto a=mesh(argv[1]), b=mesh(argv[2]);
    std::cout<<std::setprecision(16);
    double values[24]; unsigned count=0;
    while (std::cin>>values[0]) {
      for (unsigned i=1;i<24;++i) if (!(std::cin>>values[i])) throw std::runtime_error("truncated transform");
      for (auto value:values) if (!std::isfinite(value)) throw std::runtime_error("nonfinite transform");
      if (++count>256) throw std::runtime_error("audit query budget exceeded");
      const auto ta=transform(values),tb=transform(values+12);
      auto start=std::chrono::steady_clock::now();
      fcl::CollisionResult collision;
      fcl::collide(a.get(),ta,b.get(),tb,fcl::CollisionRequest(1,false),collision);
      fcl::DistanceResult distance;
      if (!collision.isCollision()) fcl::distance(a.get(),ta,b.get(),tb,fcl::DistanceRequest(true,0.,0.),distance);
      const auto elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
      std::cout<<"{\"surface_intersection\":"<<(collision.isCollision()?"true":"false")
               <<",\"surface_distance_m\":"<<(collision.isCollision()?0.:distance.min_distance)
               <<",\"triangles\":["<<a->num_tris<<","<<b->num_tris<<"],\"query_ms\":"<<elapsed;
      if (collision.isCollision()) {
        const auto& c=collision.getContact(0);
        std::cout<<",\"intersecting_triangle_ids\":["<<c.b1<<","<<c.b2<<"]";
      } else {
        std::cout<<",\"nearest_points\":[";
        for (int j=0;j<2;++j) {const auto& p=distance.nearest_points[j];if(j)std::cout<<",";std::cout<<"["<<p[0]<<","<<p[1]<<","<<p[2]<<"]";}
        std::cout<<"]";
      }
      std::cout<<"}"<<std::endl;
    }
  } catch (const std::exception& e) {std::cerr<<e.what()<<std::endl;return 1;}
}
