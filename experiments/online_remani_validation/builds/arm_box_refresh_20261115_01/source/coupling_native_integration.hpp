#include "native_collision_export.hpp"
#pragma once
// Additive native MINCO coefficient/time gradients for fixed-arm SIL only.
// No native 3D collision, wheel, search, or publication check is removed.
#include "coupling_geometry_ros.hpp"
#include "delivery_diagnostics.hpp"
#include "domain_step_rejection.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace coupling_native {
constexpr int geometry_quadrature_intervals=8;
inline bool domainRecoveryEnabled() {
  const char* value=std::getenv("COUPLING_DOMAIN_RECOVERY");
  return value && value[0]=='1' && value[1]=='\0';
}

// opts[t].get_b()/get_gdC() use ascending 8-coefficient polynomial blocks.
// real_times[t] and gradT[t] are per-piece Eigen vectors; signs[t] is the
// native trajectory-block gear. gdC and gradT must already contain any native
// gradients to retain. The native getGrad2TP call MUST happen after this call,
// propagating these coefficient derivatives to waypoint/boundary variables.
//
// Each sample's absolute time is frozen_scope.now + all preceding real piece
// durations + alpha*T. Thus dJ/dT includes BOTH local state/time sampling and
// every later sample's explicit time dependence (including across gear blocks).
template<class SnapOptContainer,class TimeContainer,class SignContainer,class GradContainer>
double addGeometry(SnapOptContainer& opts,const TimeContainer& real_times,
                   const SignContainer& signs,GradContainer& gradT,int dim) {
  if (native_collision_export::enabled()) return 0.; // Remove only the later projected objective.
  if (!coupling_geometry::enabled()) return 0.;
  if (dim<2 || opts.size()!=real_times.size() || opts.size()!=signs.size() ||
      opts.size()!=gradT.size()) throw std::invalid_argument("geometry container shape mismatch");
  coupling_geometry::Scope scope;
  if (!scope.valid()) throw std::runtime_error("unhealthy/stale geometry snapshot at solve boundary");
  const auto frame=scope.snapshot();const double epoch=scope.now();
  delivery_diag::rememberSolve(frame,epoch);
  for (size_t block=0;block<opts.size();++block) {
    const auto& times=real_times[block];
    if ((signs[block]!=-1 && signs[block]!=1) || times.size()<1 || times.size()!=gradT[block].size() ||
        times.size()!=opts[block].get_T1().size() ||
        opts[block].get_b().rows()!=8*times.size() || opts[block].get_b().cols()!=dim ||
        opts[block].get_gdC().rows()!=8*times.size() || opts[block].get_gdC().cols()!=dim ||
        !opts[block].get_b().allFinite() || !opts[block].get_gdC().allFinite() ||
        !gradT[block].allFinite() ||
        !times.allFinite() || (times.array()<=0.).any() ||
        (times-opts[block].get_T1()).cwiseAbs().maxCoeff()>1e-10)
      throw std::invalid_argument("invalid/stale geometry polynomial block");
  }
  double cost=0.,prefix=0.;
  constexpr int K=geometry_quadrature_intervals;
  for (size_t block=0;block<opts.size();++block) {
    const auto& times=real_times[block];
    for (Eigen::Index piece=0;piece<times.size();++piece) {
      const double duration=times[piece],step=duration/K;
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
        const auto value=coupling_geometry::projectionCost(
            *frame,p,v,signs[block],epoch+prefix+alpha*duration);
        if (!value.valid) {
          delivery_diag::geometryError(opts,real_times,signs,{frame,epoch},block,
            int(piece),sample,prefix,s,beta0,beta1,beta2,p,v,a);
          if (domainRecoveryEnabled() &&
              value.failure==coupling_collision::Evaluation::Failure::static_map_domain) {
            const double yaw=std::atan2(signs[block]*v.y(),signs[block]*v.x());
            const double co=std::cos(yaw),si=std::sin(yaw);
            for (size_t i=0;i<frame->body.size();++i) {
              const auto& body=frame->body[i];
              const double px=p.x()+co*body.x-si*body.y;
              const double py=p.y()+si*body.x+co*body.y;
              const double fx=(px-frame->gx)/frame->res,fy=(py-frame->gy)/frame->res;
              if (fx<0. || fy<0. || fx>=frame->nx-1 || fy>=frame->ny-1)
                throw domain_step::RejectedTrial(px,py,frame->gx,frame->gy,
                  frame->gx+(frame->nx-1)*frame->res,
                  frame->gy+(frame->ny-1)*frame->res,i);
            }
            throw std::logic_error("domain failure without an outside projected disk");
          }
          throw std::runtime_error("invalid projected geometry objective sample");
        }
        const double omega=(sample==0 || sample==K) ? .5 : 1.;
        const double measure=omega*step;
        cost+=measure*value.cost;
        opts[block].get_gdC().block(piece*8,0,8,2)+=
          measure*(beta0*value.gp.transpose()+beta1*value.gv.transpose());
        gradT[block][piece]+=omega*(value.cost/K+
          step*alpha*(value.gp.dot(v)+value.gv.dot(a)+value.gt));
        const double preceding_time_gradient=measure*value.gt;
        if (!std::isfinite(preceding_time_gradient))
          throw std::runtime_error("non-finite projected geometry time gradient");
        for (size_t earlier=0;earlier<block;++earlier)
          gradT[earlier].array()+=preceding_time_gradient;
        for (Eigen::Index earlier=0;earlier<piece;++earlier)
          gradT[block][earlier]+=preceding_time_gradient;
        if (!std::isfinite(cost) ||
            !opts[block].get_gdC().block(piece*8,0,8,2).allFinite())
          throw std::runtime_error("non-finite projected geometry integral gradient");
        for (size_t earlier=0;earlier<=block;++earlier)
          if (!gradT[earlier].allFinite())
            throw std::runtime_error("non-finite projected geometry duration gradient");
      }
      prefix+=duration;
    }
  }
  if (!std::isfinite(cost)) throw std::runtime_error("non-finite projected geometry integral");
  return cost;
}
}  // namespace coupling_native
