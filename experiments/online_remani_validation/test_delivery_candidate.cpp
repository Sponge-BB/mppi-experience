#include "delivery_accepted_candidate.hpp"
#include <cassert>
#include <iostream>
#include <limits>
#ifdef NDEBUG
#error "Candidate regression tests require active assertions; remove -DNDEBUG."
#endif
#ifdef DELIVERY_NATIVE_TEST
#include "traj_utils/poly_traj_utils.hpp"
#endif

using delivery_candidate::Scope;
int main() {
  Eigen::VectorXd initial(3),gradient(3),accepted(3),rejected(3);
  initial<<1,2,3;gradient<<.1,.2,.3;accepted<<2,3,4;rejected<<9,8,7;
  {
    Scope off(false);off.completed(initial,gradient,10);off.accepted(initial,10);
    assert(!off.usable()&&off.x.size()==0&&off.accepted_steps==0);
  }
  {
    Scope scope(true);scope.beginEvaluation();scope.completed(initial,gradient,10);
    assert(scope.usable()&&scope.initial&&scope.x==initial);
    scope.beginEvaluation();scope.completed(accepted,gradient,5);scope.accepted(accepted,5);
    assert(scope.usable()&&!scope.initial&&scope.accepted_steps==1&&scope.x==accepted);
    // Last trial is fully evaluated but rejected; it MUST NOT replace iterate.
    scope.beginEvaluation();scope.completed(rejected,gradient,100);
    assert(scope.x==accepted);
    // A budget callback returns 1e100 without completing f/g; ignore progress.
    scope.beginEvaluation();scope.accepted(rejected,1e100);
    assert(scope.x==accepted&&scope.accepted_steps==1);
    // Even a completed trial is not the point reported in progress: ignore it.
    scope.completed(rejected,gradient,100);scope.accepted(accepted,100);
    assert(scope.x==accepted&&scope.accepted_steps==1);
    scope.fail();assert(!scope.usable());
    scope.completed(initial,gradient,1);scope.accepted(initial,1);assert(!scope.usable());
  }
  {
    Scope outer(true);assert(delivery_candidate::active()==&outer);
    { Scope inner(false);assert(delivery_candidate::active()==&inner); }
    assert(delivery_candidate::active()==&outer);
  }
  assert(delivery_candidate::active()==nullptr);
  {
    Scope scope(true);Eigen::VectorXd bad=gradient;bad[1]=std::numeric_limits<double>::quiet_NaN();
    scope.completed(initial,bad,1);assert(!scope.usable()&&scope.invalid);
  }
  // Two segments with gear reversal; compare the canonical reconstruction to
  // the optimizer callback's exact P/T/gear indexing and signed boundary rule.
  const int dim=8;
  std::vector<int> pieces={2,3};
  std::vector<Eigen::MatrixXd> starts(2,Eigen::MatrixXd::Random(dim,4));
  std::vector<Eigen::MatrixXd> ends(2,Eigen::MatrixXd::Random(dim,4));
  Eigen::VectorXd packed(dim*3+5+dim+1);
  for(int i=0;i<packed.size();++i)packed[i]=.1*i-.5;
  packed[dim*3]=-2.;packed[dim*3+1]=0.;packed[dim*3+2]=1.;
  packed[packed.size()-1]=.7;
  std::vector<delivery_candidate::Part> parts;
  assert(delivery_candidate::decode(packed,pieces,dim,starts,ends,.05,parts));
  assert(parts.size()==2&&parts[0].points.cols()==1&&parts[1].points.cols()==2);
  assert(parts[0].points==Eigen::Map<const Eigen::MatrixXd>(packed.data(),dim,1));
  assert(parts[1].points==Eigen::Map<const Eigen::MatrixXd>(packed.data()+dim,dim,2));
  assert(parts[0].times[0]==.2&&parts[0].times[1]==1.&&parts[1].times[0]==2.5);
  assert(parts[0].initial==starts[0]&&parts[1].final==ends[1]);
  assert(parts[0].final.col(0)==parts[1].initial.col(0));
  assert(parts[0].final.col(0)==Eigen::Map<const Eigen::VectorXd>(packed.data()+dim*3+5,dim));
  assert((parts[0].final.col(1).head(2)+parts[1].initial.col(1).head(2)).norm()<1e-15);
  assert(std::abs(parts[0].final(0,1)-.05*std::cos(.7))<1e-15);
  assert(parts[0].final.col(2)==ends[0].col(2)&&parts[1].initial.col(3)==starts[1].col(3));
  assert(parts[0].final.col(1).tail(6)==ends[0].col(1).tail(6));
#ifdef DELIVERY_NATIVE_TEST
  {
    // Exercise the ACTUAL native polynomial generator. Old callback and rebuilt
    // canonical parts must yield identical coefficients and sampled output.
    std::vector<poly_traj::MinSnapOpt<8>> callback(2),reconstructed(2);
    for(int n=0;n<2;++n) {
      callback[n].reset(pieces[n]);reconstructed[n].reset(pieces[n]);
      Eigen::MatrixXd p=Eigen::Map<const Eigen::MatrixXd>(packed.data()+(n?dim:0),dim,pieces[n]-1);
      Eigen::VectorXd times(pieces[n]);
      for(int j=0;j<pieces[n];++j) {
        const double v=packed[dim*3+(n?2:0)+j];
        times[j]=v>0.?((.5*v+1.)*v+1.):1./((.5*v-1.)*v+1.);
      }
      Eigen::MatrixXd ini=starts[n],fin=ends[n];
      const Eigen::VectorXd gear=packed.segment(dim*3+5,dim);
      if(n>0) { ini.col(0)=gear;ini.col(1).head(2)=Eigen::Vector2d(-.05*cos(.7),-.05*sin(.7)); }
      if(n<1) { fin.col(0)=gear;fin.col(1).head(2)=Eigen::Vector2d(.05*cos(.7),.05*sin(.7)); }
      callback[n].generate(p,times,ini,fin);
      // Contaminate its cache as a rejected trial would; recovery overwrites it.
      reconstructed[n].generate(p+Eigen::MatrixXd::Ones(p.rows(),p.cols()),times,ini,fin);
      assert((callback[n].get_b()-reconstructed[n].get_b()).norm()>1.);
      reconstructed[n].generate(parts[n].points,parts[n].times,parts[n].initial,parts[n].final);
      assert(callback[n].get_b()==reconstructed[n].get_b());
      assert(callback[n].getInitConstrainPoints(8)==reconstructed[n].getInitConstrainPoints(8));
      assert(callback[n].get_T1()==reconstructed[n].get_T1());
    }
  }
#endif
  Eigen::VectorXd wrong=packed.head(packed.size()-1);
  assert(!delivery_candidate::decode(wrong,pieces,dim,starts,ends,.05,parts));
  packed[0]=std::numeric_limits<double>::infinity();
  assert(!delivery_candidate::decode(packed,pieces,dim,starts,ends,.05,parts));
  std::cout<<"delivery_candidate: accepted-cache, nonfinite, disabled, nesting, and canonical gear/time decode passed\n";
}
