// Standalone ASAN regression using the REAL MinSnapOpt implementation.
// The production class is deliberately not modified or copied here.
#include "traj_utils/poly_traj_utils.hpp"
#include <cassert>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <vector>

using Cache=std::vector<poly_traj::MinSnapOpt<8>>;
struct Finally {
  std::function<void()> action;
  explicit Finally(std::function<void()> value):action(std::move(value)){}
  Finally(const Finally&)=delete;
  ~Finally(){action();}
};
struct Input {
  Eigen::MatrixXd points=Eigen::MatrixXd::Zero(8,2);
  Eigen::MatrixXd head=Eigen::MatrixXd::Zero(8,4),tail=Eigen::MatrixXd::Zero(8,4);
  Eigen::VectorXd times=Eigen::VectorXd::Constant(3,.7);
  Input(){
    points(0,0)=.4;points(1,0)=.1;points(0,1)=.8;points(1,1)=.25;
    tail(0,0)=1.2;tail(1,0)=.2;
    head(0,1)=.12;head(1,1)=.02;tail(0,1)=.08;tail(1,1)=-.01;
    head(0,2)=.01;head(0,3)=.001;tail(1,2)=-.01;tail(1,3)=.002;
  }
};
void fresh(Cache& cache,const Input& input,double scale){
  cache.clear();cache.resize(2); // resize only an EMPTY vector: no element relocation.
  for(auto& snap:cache){snap.reset(input.times.size());snap.generate(input.points,input.times*scale,input.head,input.tail);}
}
void equal(const Cache& cache,const std::vector<Eigen::MatrixXd>& coefficients,
    const std::vector<Eigen::VectorXd>& positions){
  assert(cache.size()==coefficients.size());
  for(std::size_t n=0;n<cache.size();++n){
    assert((cache[n].get_b().array()==coefficients[n].array()).all());
    assert((cache[n].getTraj(1).getPos(.3).array()==positions[n].array()).all());
  }
}
void replay(Cache& active,const Input& input,bool throw_from_replay){
  Cache saved;
  Finally restore([&](){active.swap(saved);});
  saved.swap(active);
  for(double scale:{1.,1.,1.5}){
    fresh(active,input,scale);
    // Canonical candidate reconstruction mutates only this independently-owned cache.
    for(auto& snap:active)snap.generate(input.points,input.times*scale,input.head,input.tail);
    if(throw_from_replay)throw std::runtime_error("intentional replay unwind");
  }
}
void audit(Cache& active,const Input& input,bool throw_from_replay){
  Cache saved;
  Finally restore([&](){active.swap(saved);});
  saved.swap(active);
  fresh(active,input,1.);
  std::vector<Eigen::MatrixXd> b;
  std::vector<Eigen::VectorXd> p;
  for(const auto& snap:active){b.push_back(snap.get_b());p.push_back(snap.getTraj(1).getPos(.3));}
  try{replay(active,input,throw_from_replay);}catch(const std::runtime_error&){
    equal(active,b,p);throw;
  }
  equal(active,b,p);
}
int main(){
  Input input;Cache active;fresh(active,input,1.);
  std::vector<Eigen::MatrixXd> b;
  std::vector<Eigen::VectorXd> p;
  for(const auto& snap:active){b.push_back(snap.get_b());p.push_back(snap.getTraj(1).getPos(.3));}
  for(int repeat=0;repeat<12;++repeat){
    audit(active,input,false);equal(active,b,p);
    try{audit(active,input,true);assert(false);}catch(const std::runtime_error&){}
    equal(active,b,p);
    // Prove original owning storage remains valid for its next normal reset/generate.
    fresh(active,input,1.);equal(active,b,p);
  }
  std::cout<<"PASS real MinSnapOpt nested vector-swap ownership, ordinary and exception paths\n";
}
