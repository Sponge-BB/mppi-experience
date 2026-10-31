// Native MinSnapOpt<8> tests only: no ROS graph or command publication.
#include "clearance_native.hpp"
#include "traj_utils/poly_traj_utils.hpp"
#include <cassert>
#include <iostream>
#include <thread>

using Times=std::vector<Eigen::VectorXd>;
using Matrices=std::vector<Eigen::MatrixXd>;
struct Input { Times times;Matrices points,head,tail; };
struct Gradient { Times times;Matrices points,head,tail; };
static Input input() {
  Input x;x.times={Eigen::Vector2d(.43,.48),Eigen::Vector2d(.51,.56)};
  x.head.resize(2);x.tail.resize(2);x.points.resize(2);
  for(int b=0;b<2;++b) {
    x.head[b]=Eigen::MatrixXd::Zero(8,4);x.tail[b]=Eigen::MatrixXd::Zero(8,4);
    const double origin=.15*b;
    x.head[b](0,0)=origin+.017;x.head[b](1,0)=-.033;
    x.tail[b](0,0)=origin+.15;x.tail[b](1,0)=.014;
    x.head[b](0,1)=.13;x.tail[b](0,1)=.14;
    x.head[b](1,1)=.02;x.tail[b](1,1)=.03;
    x.points[b]=Eigen::MatrixXd::Zero(8,1);
    x.points[b](0,0)=origin+.083;x.points[b](1,0)=-.009;
  }
  return x;
}
static std::shared_ptr<shared_collision::Snapshot> fixture() {
  auto m=std::make_shared<shared_collision::Snapshot>();
  m->stamp=999.95;m->healthy=true;m->gx=-3.;m->gy=-3.;m->res=.1;m->nx=61;m->ny=61;
  m->margin=.1;m->comfort=.32;m->weight=17.;
  m->body={{.31,.12,.18},{-.27,-.08,.30},{.04,.25,.28}};
  for(int iy=0;iy<m->ny;++iy)for(int ix=0;ix<m->nx;++ix) {
    const double x=m->gx+ix*m->res,y=m->gy+iy*m->res;
    m->field.push_back(.36+.02*x+.025*y+.005*x*y);
  }
  return m;
}
static void bindSnapshot(const std::shared_ptr<shared_collision::Snapshot>& m) {
  std::shared_ptr<const shared_collision::Snapshot> c=m;
  std::atomic_store(&shared_collision::store(),c);
}
static double evaluate(const Input& x,Gradient* gradient=nullptr) {
  std::vector<poly_traj::MinSnapOpt<8>> opts(2);Times gradT(2);
  for(int b=0;b<2;++b) {
    opts[b].reset(x.times[b].size());
    opts[b].generate(x.points[b],x.times[b],x.head[b],x.tail[b]);
    opts[b].get_gdC().setZero();gradT[b]=Eigen::VectorXd::Zero(x.times[b].size());
  }
  const double cost=clearance_native::addPenalty(opts,x.times,std::vector<int>{1,-1},gradT,8);
  if(gradient) {
    gradient->points.resize(2);gradient->head.resize(2);gradient->tail.resize(2);
    for(int b=0;b<2;++b) {
      gradient->points[b]=Eigen::MatrixXd::Zero(8,1);
      opts[b].getGrad2TP(gradT[b],gradient->points[b],gradient->head[b],gradient->tail[b]);
    }
    gradient->times=gradT;
  }
  return cost;
}
static clearance_native::Multipliers multipliersFor(const Input& x) {
  std::vector<poly_traj::MinSnapOpt<8>> opts(2);
  for(int b=0;b<2;++b) {
    opts[b].reset(x.times[b].size());
    opts[b].generate(x.points[b],x.times[b],x.head[b],x.tail[b]);
    opts[b].get_gdC().setConstant(7.);
  }
  const auto result=clearance_native::prepareMultipliers(opts,x.times,std::vector<int>{1,-1},8,1e5);
  const auto frame=coupling_geometry::currentSnapshot();assert(frame);
  size_t offset=0;bool positive=false;
  for(int block=0;block<2;++block) {
    assert((opts[block].get_gdC().array()==7.).all());
    for(int piece=0;piece<2;++piece)for(int j=0;j<=8;++j) {
      const double s=double(j)*x.times[block][piece]/8;
      Eigen::Matrix<double,8,1> b0,b1;b0.setZero();b1.setZero();b0[0]=1.;
      for(int k=1;k<8;++k)b0[k]=b0[k-1]*s;
      for(int k=1;k<8;++k)b1[k]=k*b0[k-1];
      const auto c=opts[block].get_b().block(piece*8,0,8,2);
      const Eigen::Vector2d p=c.transpose()*b0,v=c.transpose()*b1;
      const int gear=block ? -1 : 1;
      const double yaw=std::atan2(gear*v.y(),gear*v.x());
      for(const auto& body:frame->body) {
        auto one=*frame;one.body={body};
        const auto sample=clearance_penalty::evaluate(one,p.x(),p.y(),yaw,1e5);
        assert(sample.valid && offset<result.size());
        const double expected=1e5*std::max(0.,-sample.gap);
        assert(std::abs(result[offset++]-expected)<1e-9);positive|=expected>0.;
      }
    }
  }
  assert(offset==result.size() && result.size()==108 && positive);
  return result;
}
static void near(double a,double b,const char* label,double tolerance=5e-6) {
  const double error=std::abs(a-b)/(1.+std::abs(a)+std::abs(b));
  if(!std::isfinite(error)||error>tolerance) {
    std::cerr<<label<<" analytic="<<a<<" numeric="<<b<<" rel="<<error<<'\n';std::abort();
  }
}
static void sampleGradients(const shared_collision::Snapshot& m) {
  const Eigen::Vector2d p(.017,-.033),v(.13,.04);
  const double multipliers[]={2000.,3500.,900.};
  for(int mode:{0,1})for(int gear:{1,-1}) {
    const auto sample=[&](const Eigen::Vector2d& pp,const Eigen::Vector2d& vv) {
      return clearance_native::sampleCost(m,pp,vv,gear,1e5,
          mode ? multipliers : nullptr,mode ? 3 : 0);
    };
    const auto e=sample(p,v);assert(e.valid && e.gt==0. && e.cost>0.);
    for(int d=0;d<2;++d) {
      auto plus=p,minus=p;plus[d]+=1e-6;minus[d]-=1e-6;
      near(e.gp[d],(sample(plus,v).cost-sample(minus,v).cost)/2e-6,"sample position");
      plus=v;minus=v;plus[d]+=1e-6;minus[d]-=1e-6;
      near(e.gv[d],(sample(p,plus).cost-sample(p,minus).cost)/2e-6,"sample velocity");
    }
    const auto zero=sample(p,Eigen::Vector2d::Zero());
    assert(zero.valid && zero.gv.allFinite() && zero.gv.isZero(0.));
  }
  assert(!clearance_native::sampleCost(m,p,v,0,1e5).valid);
  assert(!clearance_native::sampleCost(m,{NAN,0.},v,1,1e5).valid);
  assert(!clearance_native::sampleCost(m,p,{0.,NAN},1,1e5).valid);
}
struct Mock {
  Eigen::VectorXd times=Eigen::VectorXd::Ones(1);
  Eigen::MatrixXd b=Eigen::MatrixXd::Zero(8,2),gd=Eigen::MatrixXd::Zero(8,2);
  const Eigen::VectorXd& get_T1() const {return times;}
  const Eigen::MatrixXd& get_b() const {return b;}
  Eigen::MatrixXd& get_gdC() {return gd;}
};
static void probe(Eigen::Vector2d p={.423698995724897,4.9096377779231}) {
  std::vector<Mock> opts(1);opts[0].b.row(0)=p.transpose();
  opts[0].b.row(1)=Eigen::Vector2d(-.1277832969887734,.5422823571572659).transpose();
  Times times{Eigen::VectorXd::Ones(1)},grad{Eigen::VectorXd::Zero(1)};
  clearance_native::addPenalty(opts,times,std::vector<int>{1},grad,2);
}
static void fatalProbe(Eigen::Vector2d p={.423698995724897,4.9096377779231}) {
  bool fatal=false;
  try {probe(p);}catch(const domain_step::RejectedTrial&) {
    assert(false && "fatal failure became recoverable");
  }catch(const std::exception&) {fatal=true;}
  assert(fatal);
}
int main() {
  unsetenv("COUPLING_DIAGNOSTIC_DIR");setenv("COUPLING_ENGINEERING","1",1);
  // Disabled path must work before ROS time exists and without any snapshot.
  std::vector<Mock> arbitrary(1);arbitrary[0].gd.setConstant(7.);
  Times arbitrary_times,arbitrary_grad{Eigen::VectorXd::Constant(1,9.)};
  assert(!clearance_native::active());
  assert(clearance_native::addPenalty(arbitrary,arbitrary_times,std::vector<int>{},arbitrary_grad,-1)==0.);
  assert((arbitrary[0].gd.array()==7.).all() && arbitrary_grad[0][0]==9.);
  for(double bad:{0.,-1.,double(NAN),double(INFINITY)}) {
    bool fatal=false;try {clearance_native::Active invalid(bad);}catch(const std::invalid_argument&) {fatal=true;}
    assert(fatal && !clearance_native::active());
  }
  ros::Time::init();ros::Time::setNow(ros::Time(1000.));auto m=fixture();bindSnapshot(m);
  sampleGradients(*m);Input x=input();
  {
    clearance_native::Active enabled(clearance_penalty::fixed_rho);
    assert(clearance_native::active() && clearance_native::currentRho()==1e5);
    {clearance_native::Active nested(2e5);assert(clearance_native::currentRho()==2e5);}
    assert(clearance_native::currentRho()==1e5);
    bool other_active=true;std::thread other([&]{other_active=clearance_native::active();});other.join();
    assert(!other_active);
    const auto prepared=multipliersFor(x);
    auto copied=prepared;
    {clearance_native::Active immutable(1e5,copied);copied.clear();assert(evaluate(x)>0.);}
    for(int mode:{0,1}) {
      const auto lambda=mode ? prepared : clearance_native::Multipliers(prepared.size(),0.);
      clearance_native::Active selected(1e5,lambda);
      Gradient gradient;const double cost=evaluate(x,&gradient);assert(cost>0.);
      constexpr double epsilon=2e-6;
      for(int b=0;b<2;++b)for(int p=0;p<2;++p) {
        auto plus=x,minus=x;plus.times[b][p]+=epsilon;minus.times[b][p]-=epsilon;
        near(gradient.times[b][p],(evaluate(plus)-evaluate(minus))/(2*epsilon),"duration");
      }
      for(int b=0;b<2;++b)for(int d=0;d<8;++d) {
        auto plus=x,minus=x;plus.points[b](d,0)+=epsilon;minus.points[b](d,0)-=epsilon;
        near(gradient.points[b](d,0),(evaluate(plus)-evaluate(minus))/(2*epsilon),"waypoint");
        if(d>=2)assert(gradient.points[b](d,0)==0.);
        for(int derivative=0;derivative<4;++derivative) {
          plus=x;minus=x;plus.head[b](d,derivative)+=epsilon;minus.head[b](d,derivative)-=epsilon;
          near(gradient.head[b](d,derivative),(evaluate(plus)-evaluate(minus))/(2*epsilon),"head");
          plus=x;minus=x;plus.tail[b](d,derivative)+=epsilon;minus.tail[b](d,derivative)-=epsilon;
          near(gradient.tail[b](d,derivative),(evaluate(plus)-evaluate(minus))/(2*epsilon),"tail");
          if(d>=2)assert(gradient.head[b](d,derivative)==0. && gradient.tail[b](d,derivative)==0.);
        }
      }
    }
    const double cost=evaluate(x);
    for(double bad:{-1.,double(NAN),double(INFINITY)}) {
      bool fatal=false;try {clearance_native::Active invalid(1e5,{bad});}
      catch(const std::invalid_argument&) {fatal=true;}
      assert(fatal && clearance_native::currentRho()==1e5);
    }
    {clearance_native::Active wrong_count(1e5,{0.});
      bool fatal=false;try {evaluate(x);}catch(const std::invalid_argument&) {fatal=true;}
      assert(fatal);
    }
    // Same immutable frame is retained through a nested native Scope.
    {coupling_geometry::Scope frozen;bindSnapshot(nullptr);near(evaluate(x),cost,"frozen frame",0.);}
    bindSnapshot(m);
    m=fixture();m->stamp=998.;bindSnapshot(m);fatalProbe();
    m=fixture();m->healthy=false;bindSnapshot(m);fatalProbe();
    m=fixture();m->field.pop_back();bindSnapshot(m);fatalProbe();
    m=fixture();m->body[1].r=NAN;bindSnapshot(m);fatalProbe();
    m=fixture();bindSnapshot(m);fatalProbe({NAN,0.});
    // Fixed collocation weights make the inactive AL constant independent of
    // piece duration. A T*omega/K measure would incorrectly return -5*T here.
    m=fixture();m->body={{0.,0.,.1}};m->field.assign(61*61,.5);bindSnapshot(m);
    {
      clearance_native::Active negative_offset(1e5,clearance_native::Multipliers(9,1000.));
      std::vector<Mock> opts(1);opts[0].b(1,0)=.1;
      for(double duration:{.5,1.,2.}) {
        opts[0].times[0]=duration;opts[0].gd.setZero();
        Times times{opts[0].times},grad{Eigen::VectorXd::Zero(1)};
        near(clearance_native::addPenalty(opts,times,std::vector<int>{1},grad,2),-5.,"AL offset duration",1e-12);
        assert(grad[0].isZero(0.) && opts[0].gd.isZero(0.));
      }
    }
    // Recorded outside-body case: only the domain switch makes it recoverable.
    m=fixture();m->gx=-7.5;m->gy=-5.;m->res=.05;m->nx=300;m->ny=200;
    m->body={{0.,0.,.1},{.125,0.,.1}};m->field.assign(300*200,1.);bindSnapshot(m);
    setenv("COUPLING_DOMAIN_RECOVERY","0",1);fatalProbe();
    setenv("COUPLING_DOMAIN_RECOVERY","1",1);
    bool typed=false;try {probe();}catch(const domain_step::RejectedTrial& e) {
      typed=true;assert(e.body_index==1);near(e.y,5.03130554094473,"recorded domain",1e-12);
    }
    assert(typed);
    m->stamp=998.;bindSnapshot(m);fatalProbe();
    m=fixture();m->field.pop_back();bindSnapshot(m);fatalProbe();
    m=fixture();bindSnapshot(m);fatalProbe({NAN,0.});
  }
  assert(!clearance_native::active());bindSnapshot(nullptr);Gradient disabled;
  assert(evaluate(x,&disabled)==0.);
  for(const auto& t:disabled.times)assert(t.isZero(0.));
  for(const auto& p:disabled.points)assert(p.isZero(0.));
  std::cout<<"CLEARANCE_NATIVE_PASSED two blocks/four pieces; both gears; all-eight-dim "
               "MINCO waypoint/head/tail/duration gradients; zero/nonzero immutable AL; K8 indexing; "
               "fixed weights/no duration reward; disabled identity; frozen scope; "
               "stale/malformed/nonfinite fatal; typed domain opt-in; no ROS graph\n";
}
