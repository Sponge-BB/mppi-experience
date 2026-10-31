// No ROS graph / controller / planner. Creates only opt-in test diagnostics.
#include "delivery_diagnostics.hpp"
#include <cassert>
#include <dirent.h>
#include <iostream>
#include <vector>

struct PieceMock {
  Eigen::Matrix<double,8,8> c=Eigen::Matrix<double,8,8>::Zero();double T=.4;
  double getDuration() const{return T;}
  const Eigen::Matrix<double,8,8>& getCoeffMat()const{return c;}
};
struct TrajMock {
  std::vector<PieceMock> pieces{PieceMock{},PieceMock{}};
  int getPieceNum()const{return pieces.size();}
  const PieceMock& getPiece(int i)const{return pieces.at(i);}
};
struct PartMock {TrajMock traj;int singul=1;double duration=.8;};
struct DataMock {std::vector<PartMock> singul_traj{PartMock{}};double start_time=10.;};
struct OptMock {
  Eigen::MatrixXd c=Eigen::MatrixXd::Zero(8,8);
  const Eigen::MatrixXd& get_b() const{return c;}
};
int files(const std::string& path) {
  DIR* d=opendir(path.c_str());if(!d)return 0;int n=0;
  while(auto* e=readdir(d))if(e->d_name[0]!='.')++n;closedir(d);return n;
}
int main() {
  unsetenv("COUPLING_DIAGNOSTIC_DIR");DataMock data;
  {delivery_diag::CandidateRecord<DataMock> disabled(data,4);disabled.accept();}
  assert(delivery_diag::directory()==nullptr);
  char templ[]="/tmp/delivery_diag_unit_XXXXXX";char* path=mkdtemp(templ);assert(path);
  setenv("COUPLING_DIAGNOSTIC_DIR",path,1);
  auto m=std::make_shared<shared_collision::Snapshot>();
  m->sequence=3;m->stamp=9.95;m->nx=m->ny=4;m->res=1.;m->healthy=true;
  m->gx=m->gy=-1.;m->field.assign(16,2.);m->body={{.1,.01,.2}};
  m->obstacles={{.2,.3,.1,.05,.25,1}};
  delivery_diag::beginSolve(4,7);delivery_diag::rememberSolve(m,10.);
  {delivery_diag::CandidateRecord<DataMock> accepted(data,4);accepted.acceptance(m,10.01);accepted.accept();}
  assert(files(std::string(path)+"/candidates")==1);assert(files(std::string(path)+"/snapshots")==1);
  coupling_collision::Evaluation e;e.physical=.05;e.gap=-.05;
  {delivery_diag::CandidateRecord<DataMock> rejected(data,4);rejected.acceptance(m,10.02);
    Eigen::VectorXd p=Eigen::VectorXd::Zero(8),v=p;v(0)=.2;
    rejected.rejectSample("static_gap",0,12,.42,10.42,p,v,0.,e,false);}
  assert(files(std::string(path)+"/candidates")==2);assert(files(std::string(path)+"/snapshots")==1);
  assert(delivery_diag::leaf(*m,100.,0.,0.,0.,true,false).find("out_of_bounds")!=std::string::npos);
  std::vector<OptMock> opts(1);std::vector<Eigen::VectorXd> times{Eigen::VectorXd::Constant(1,.4)};
  std::vector<int> signs{1};Eigen::Matrix<double,8,1> b=Eigen::Matrix<double,8,1>::Zero();b(0)=1;
  Eigen::Vector2d p(100.,0.),v(.2,0.),a(0.,0.);
  delivery_diag::geometryError(opts,times,signs,{m,10.},0,0,0,0.,0.,b,b,b,p,v,a);
  delivery_diag::geometryError(opts,times,signs,{m,10.},0,0,1,0.,.05,b,b,b,p,v,a);
  assert(files(std::string(path)+"/geometry_errors")==1);
  // Diagnostic I/O failures must never alter the caller's outcome/exception.
  setenv("COUPLING_DIAGNOSTIC_DIR","/nonexistent-delivery-diag-parent/no-dir",1);
  {delivery_diag::CandidateRecord<DataMock> broken(data,4);broken.acceptance(m,10.);broken.reject("fixture_reject");}
  std::cout<<"DELIVERY_DIAGNOSTICS_PASSED "<<path<<std::endl;
}
