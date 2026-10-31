#pragma once
// Recovery-only additive static collocation penalty and MINCO propagation.
// The caller owns a single original-request deadline, restart limits, and all
// unchanged native/publication gates. This header never changes any budget.
#include "clearance_penalty.hpp"
#include "coupling_native_integration.hpp"

namespace clearance_native {
using Multipliers=std::vector<double>;
struct State {
  bool active=false;
  double rho=0.;
  std::shared_ptr<const Multipliers> multipliers;
};
inline State& state() { static thread_local State value; return value; }
inline bool active() { return state().active; }
inline double currentRho() { return state().rho; }

// A warm-restart scope only. Nesting restores prior state; default is disabled.
class Active {
  State previous_;
public:
  explicit Active(double rho):previous_(state()) {
    if (!std::isfinite(rho) || rho <= 0.)
      throw std::invalid_argument("clearance recovery rho must be finite and positive");
    state() = {true,rho,nullptr};
  }
  Active(double rho,const Multipliers& multipliers):previous_(state()) {
    if (!std::isfinite(rho) || rho <= 0.)
      throw std::invalid_argument("clearance recovery rho must be finite and positive");
    for(double value:multipliers)if(!std::isfinite(value) || value<0.)
      throw std::invalid_argument("clearance multipliers must be finite and nonnegative");
    auto immutable=std::make_shared<const Multipliers>(multipliers);
    state() = {true,rho,std::move(immutable)};
  }
  Active(const Active&) = delete;
  Active& operator=(const Active&) = delete;
  ~Active() { state()=previous_; }
};

struct ObjectiveSample {
  double cost=0.,gt=0.;
  double physical=std::numeric_limits<double>::infinity();
  double gap=std::numeric_limits<double>::infinity();
  Eigen::Vector2d gp=Eigen::Vector2d::Zero(),gv=Eigen::Vector2d::Zero();
  bool valid=false;
  coupling_collision::Evaluation::Failure failure=
      coupling_collision::Evaluation::Failure::malformed;
};
// Mathematical explicit-frame operation; addPenalty owns freshness checks.
// At zero speed use the same bounded derivative as the native geometry cost;
// atan2 itself is not differentiable there. No dynamic quantity is charged.
inline ObjectiveSample sampleCost(const clearance_penalty::Snapshot& frame,
                                  const Eigen::Vector2d& p,const Eigen::Vector2d& v,
                                  int gear,double rho,const double* lambda=nullptr,
                                  size_t lambda_count=0) {
  ObjectiveSample out;
  if (!p.allFinite() || !v.allFinite() || (gear!=-1 && gear!=1)) return out;
  const double yaw=std::atan2(gear*v.y(),gear*v.x());
  const auto value=clearance_penalty::evaluate(frame,p.x(),p.y(),yaw,rho,lambda,lambda_count);
  out.failure=value.failure;
  if (!value.valid) return out;
  out.cost=value.cost;out.physical=value.physical;out.gap=value.gap;
  out.gp=Eigen::Vector2d(value.gx,value.gy);
  out.gv=value.gyaw*Eigen::Vector2d(-v.y(),v.x())/std::max(1e-8,v.squaredNorm());
  out.valid=std::isfinite(out.cost) && out.gp.allFinite() && out.gv.allFinite();
  if (!out.valid) out.failure=coupling_collision::Evaluation::Failure::malformed;
  return out;
}

namespace detail {
template<class SnapOptContainer,class TimeContainer,class SignContainer,class GradContainer>
void validate(SnapOptContainer& opts,const TimeContainer& real_times,
              const SignContainer& signs,GradContainer& gradT,int dim) {
  if (dim<2 || opts.size()!=real_times.size() || opts.size()!=signs.size() ||
      opts.size()!=gradT.size())
    throw std::invalid_argument("clearance penalty container shape mismatch");
  for (size_t block=0;block<opts.size();++block) {
    const auto& times=real_times[block];
    if ((signs[block]!=-1 && signs[block]!=1) || times.size()<1 ||
        times.size()!=gradT[block].size() || times.size()!=opts[block].get_T1().size() ||
        opts[block].get_b().rows()!=8*times.size() || opts[block].get_b().cols()!=dim ||
        opts[block].get_gdC().rows()!=8*times.size() || opts[block].get_gdC().cols()!=dim ||
        !opts[block].get_b().allFinite() || !opts[block].get_gdC().allFinite() ||
        !gradT[block].allFinite() || !times.allFinite() || (times.array()<=0.).any() ||
        (times-opts[block].get_T1()).cwiseAbs().maxCoeff()>1e-10)
      throw std::invalid_argument("invalid/stale clearance polynomial block");
  }
}
inline void requireSample(const ObjectiveSample& value,
                          const clearance_penalty::Snapshot& frame,
                          const Eigen::Vector2d& p,const Eigen::Vector2d& v,int gear) {
  if(value.valid)return;
  if (coupling_native::domainRecoveryEnabled() &&
      value.failure==coupling_collision::Evaluation::Failure::static_map_domain) {
    const double yaw=std::atan2(gear*v.y(),gear*v.x());
    const double co=std::cos(yaw),si=std::sin(yaw);
    for (size_t i=0;i<frame.body.size();++i) {
      const auto& body=frame.body[i];
      const double px=p.x()+co*body.x-si*body.y;
      const double py=p.y()+si*body.x+co*body.y;
      const double fx=(px-frame.gx)/frame.res,fy=(py-frame.gy)/frame.res;
      if (fx<0. || fy<0. || fx>=frame.nx-1 || fy>=frame.ny-1)
        throw domain_step::RejectedTrial(px,py,frame.gx,frame.gy,
            frame.gx+(frame.nx-1)*frame.res,
            frame.gy+(frame.ny-1)*frame.res,i);
    }
    throw std::logic_error("clearance domain failure without outside body disk");
  }
  throw std::runtime_error("invalid static clearance objective sample");
}
}  // namespace detail

// Exactly one nominal-state multiplier update, with no cross-request state.
// Ordering is block / piece / j=0..K / body. No gradient is changed here.
// The caller must retain topology/body ordering and include preparation in its
// original request deadline; these are not continuous-time constraints.
template<class SnapOptContainer,class TimeContainer,class SignContainer>
Multipliers prepareMultipliers(SnapOptContainer& opts,const TimeContainer& real_times,
                               const SignContainer& signs,int dim,
                               double rho=clearance_penalty::fixed_rho) {
  if(!coupling_geometry::enabled())
    throw std::runtime_error("clearance multiplier preparation requires engineering scope");
  if(!std::isfinite(rho) || rho<=0.)
    throw std::invalid_argument("clearance recovery rho must be finite and positive");
  // Reuse shape/finite checks without touching any existing coefficient/time
  // gradients. Native caches must describe the accepted nominal x already.
  std::vector<Eigen::VectorXd> dummy;
  for(const auto& times:real_times)dummy.push_back(Eigen::VectorXd::Zero(times.size()));
  detail::validate(opts,real_times,signs,dummy,dim);
  coupling_geometry::Scope scope;
  if(!scope.valid())throw std::runtime_error("unhealthy/stale clearance preparation snapshot");
  const auto frame=scope.snapshot();Multipliers result;
  constexpr int K=coupling_native::geometry_quadrature_intervals;
  for(size_t block=0;block<opts.size();++block) {
    const auto& times=real_times[block];
    for(Eigen::Index piece=0;piece<times.size();++piece) {
      const Eigen::Matrix<double,8,2> c=opts[block].get_b().block(piece*8,0,8,2);
      for(int j=0;j<=K;++j) {
        const double s=double(j)*times[piece]/K;
        Eigen::Matrix<double,8,1> b0,b1;b0.setZero();b1.setZero();b0[0]=1.;
        for(int k=1;k<8;++k)b0[k]=b0[k-1]*s;
        for(int k=1;k<8;++k)b1[k]=k*b0[k-1];
        const Eigen::Vector2d p=c.transpose()*b0,v=c.transpose()*b1;
        const auto value=sampleCost(*frame,p,v,signs[block],rho);
        detail::requireSample(value,*frame,p,v,signs[block]);
        const double yaw=std::atan2(signs[block]*v.y(),signs[block]*v.x());
        const double co=std::cos(yaw),si=std::sin(yaw);
        for(const auto& body:frame->body) {
          const auto distance=shared_collision::mapDistance(*frame,
              p.x()+co*body.x-si*body.y,p.y()+si*body.x+co*body.y);
          const double lambda=rho*std::max(0.,frame->margin-(distance.d-body.r));
          if(!std::isfinite(lambda))throw std::runtime_error("non-finite clearance multiplier");
          result.push_back(lambda);
        }
      }
    }
  }
  return result;
}

template<class SnapOptContainer,class TimeContainer,class SignContainer,class GradContainer>
double addPenalty(SnapOptContainer& opts,const TimeContainer& real_times,
                  const SignContainer& signs,GradContainer& gradT,int dim) {
  // Exact disabled identity, before inspecting containers, ROS time, snapshots,
  // or even the engineering switch. Tests rely on this ordering.
  if (!active()) return 0.;
  if (!coupling_geometry::enabled()) return 0.;
  detail::validate(opts,real_times,signs,gradT,dim);
  coupling_geometry::Scope scope;
  if (!scope.valid())
    throw std::runtime_error("unhealthy/stale clearance snapshot at solve boundary");
  const auto frame=scope.snapshot();
  double cost=0.;
  constexpr int K=coupling_native::geometry_quadrature_intervals;
  size_t expected=0,lambda_offset=0;
  for(const auto& times:real_times)expected+=size_t(times.size())*(K+1)*frame->body.size();
  const auto multipliers=state().multipliers;
  if(multipliers && multipliers->size()!=expected)
    throw std::invalid_argument("clearance multiplier count/topology mismatch");
  for (size_t block=0;block<opts.size();++block) {
    const auto& times=real_times[block];
    for (Eigen::Index piece=0;piece<times.size();++piece) {
      const double duration=times[piece];
      const Eigen::Matrix<double,8,2> coefficients=
          opts[block].get_b().block(piece*8,0,8,2);
      for (int sample=0;sample<=K;++sample) {
        const double alpha=double(sample)/K,s=alpha*duration;
        Eigen::Matrix<double,8,1> beta0,beta1,beta2;
        beta0.setZero();beta1.setZero();beta2.setZero();beta0[0]=1.;
        for (int k=1;k<8;++k) beta0[k]=beta0[k-1]*s;
        for (int k=1;k<8;++k) beta1[k]=k*beta0[k-1];
        for (int k=2;k<8;++k) beta2[k]=k*(k-1)*beta0[k-2];
        const Eigen::Vector2d p=coefficients.transpose()*beta0;
        const Eigen::Vector2d v=coefficients.transpose()*beta1;
        const Eigen::Vector2d a=coefficients.transpose()*beta2;
        const double* lambda=multipliers ? multipliers->data()+lambda_offset : nullptr;
        const auto value=sampleCost(*frame,p,v,signs[block],currentRho(),lambda,
            multipliers ? frame->body.size() : 0);
        detail::requireSample(value,*frame,p,v,signs[block]);
        lambda_offset+=frame->body.size();
        const double omega=(sample==0 || sample==K) ? .5 : 1.;
        // Fixed collocation weights, NOT duration-weighted integration. AL's
        // negative lambda^2/(2*rho) offset must not create an artificial reward
        // for lengthening safe trajectories. Multipliers and weights are fixed
        // throughout one restart, including duration perturbations.
        const double measure=omega/K;
        cost+=measure*value.cost;
        opts[block].get_gdC().block(piece*8,0,8,2)+=
            measure*(beta0*value.gp.transpose()+beta1*value.gv.transpose());
        // Static gt is exactly zero. Only moving local collocation states
        // contribute; there is no measure derivative or prefix-time gradient.
        gradT[block][piece]+=measure*alpha*(value.gp.dot(v)+value.gv.dot(a));
        if (!std::isfinite(cost) ||
            !opts[block].get_gdC().block(piece*8,0,8,2).allFinite() ||
            !gradT[block].allFinite())
          throw std::runtime_error("non-finite static clearance integral gradient");
      }
    }
  }
  return cost;
}
}  // namespace clearance_native
