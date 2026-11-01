#include <Eigen/Core>
#include <vector>
namespace failure_probe { enum { BASE_FEAS=0 }; struct Timer { explicit Timer(int){} }; }
class PolyTrajOptimizer {
public:
  Eigen::Matrix2d B_h_;
  std::vector<int> singul_container_;
  double wei_feas_, max_vel_, non_singul_v_, mobile_base_wheel_base_,
    mobile_base_wheel_radius_, max_wheel_omega_, max_wheel_alpha_;
  bool smoothedL1(const double&, const double&, double&, double&);
  bool feasibilityGradCostCar(const Eigen::Vector2d&, const Eigen::Vector2d&,
    const Eigen::Vector2d&, int, Eigen::Vector2d&, Eigen::Vector2d&,
    Eigen::Vector2d&, double&, bool);
};
bool PolyTrajOptimizer::smoothedL1(const double &x, const double &mu, double &f, double &df){
    if (x < 0.0)
    {
      f=0;
      df=0;
      return false;
    }
    else if (x > mu)
    {
      f = x - 0.5 * mu;
      df = 1.0;
      return true;
    }
    else
    {
      const double xdmu = x / mu;
      const double sqrxdmu = xdmu * xdmu;
      const double mumxd2 = mu - 0.5 * x;
      f = mumxd2 * sqrxdmu * xdmu;
      df = sqrxdmu * ((-0.5) * xdmu + 3.0 * mumxd2 / mu);

      return true;
    }
  }
bool PolyTrajOptimizer::feasibilityGradCostCar(const Eigen::Vector2d &vel, const Eigen::Vector2d &acc, const Eigen::Vector2d &jer, const int trajid,
                                                 Eigen::Vector2d &gradv_2d, Eigen::Vector2d &grada_2d, Eigen::Vector2d &gradj_2d,
                                                 double &cost_mm_feasible, bool dense_sample){
    bool ret = false;
    double pen, f, df;
    failure_probe::Timer timer(failure_probe::BASE_FEAS);
    cost_mm_feasible = 0;

    double vTv = vel.transpose() * vel;
    if (vTv < 1e-8) { vTv = 1e-8; }
    const double aTv = acc.transpose() * vel;
    const double aTBv = acc.transpose() * B_h_ * vel;
    const double jTBv = jer.transpose() * B_h_ * vel;
    double v_norm = vel.norm();
    if (v_norm < 1e-4) { v_norm = 1e-4; }
    const double vTv_inv = 1.0 / vTv; // avoid siguality vel = 0
    const double vTv_inv2 = vTv_inv * vTv_inv;

    
    // Check minimum speed
    if(dense_sample){ 
      pen = non_singul_v_ * non_singul_v_ - vel.squaredNorm();
      f = 0; df = 0;
      if(smoothedL1(pen, 0.005, f, df)){
        cost_mm_feasible += wei_feas_ * 1e6 * f;
        gradv_2d += -wei_feas_ * 1e6 * df * 2.0 * vel;
        ret = true;
      }
    }

    // Linear velocity
    pen = vel.squaredNorm() - max_vel_ * max_vel_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * vel;
      ret = true;
    }

    double omega = aTBv * vTv_inv;
    double wheel_omega_left = (2.0 * singul_container_[trajid] * v_norm - mobile_base_wheel_base_ * omega) / (2.0 * mobile_base_wheel_radius_);
    Eigen::Vector2d dOmegadV = B_h_.transpose() * acc / vTv - (aTBv * vel + vel.transpose() * B_h_.transpose() * acc * vel) / vTv / vTv;
    Eigen::Vector2d dOmegadA = vTv_inv * (B_h_ * vel);
    // Left wheel rotation speed
    pen = wheel_omega_left * wheel_omega_left - max_wheel_omega_ * max_wheel_omega_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_omega_left * (2.0 * singul_container_[trajid] / v_norm * vel - mobile_base_wheel_base_ * dOmegadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += -wei_feas_ * df * 2.0 * wheel_omega_left * mobile_base_wheel_base_ * dOmegadA / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }

    // Right wheel rotation speed
    double wheel_omega_right = (2.0 * singul_container_[trajid] * v_norm + mobile_base_wheel_base_ * omega) / (2 * mobile_base_wheel_radius_);
    pen = wheel_omega_right * wheel_omega_right - max_wheel_omega_ * max_wheel_omega_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_omega_right * (2.0 * singul_container_[trajid] / v_norm * vel + mobile_base_wheel_base_ * dOmegadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += wei_feas_ * df * 2.0 * wheel_omega_right * mobile_base_wheel_base_ * dOmegadA / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }

    double alpha = jTBv * vTv_inv - 2.0 * aTBv * aTv * vTv_inv2;
    Eigen::Vector2d dAlphadV = (B_h_.transpose() * jer * vTv - 2.0 * jTBv * vel) * vTv_inv2
                              - 2.0 * (aTBv * acc + aTv * B_h_.transpose() * acc) * vTv_inv2
                              + 8.0 * vTv_inv2 * vTv_inv * aTBv * aTv * vel;
    Eigen::Vector2d dAlphadA = -2.0 * vTv_inv2 * (aTv * B_h_ * vel + aTBv * vel);
    Eigen::Vector2d dAlphadJ = B_h_ * vel * vTv_inv;

    // Left wheel rotational acceleration
    double wheel_alpha_left = (2.0 * singul_container_[trajid] * aTv / v_norm - mobile_base_wheel_base_ * alpha) / (2.0 * mobile_base_wheel_radius_);
    pen = wheel_alpha_left * wheel_alpha_left - max_wheel_alpha_ * max_wheel_alpha_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_alpha_left * (2.0 * singul_container_[trajid] * (v_norm * acc - aTv * vel / v_norm) * vTv_inv - mobile_base_wheel_base_ * dAlphadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += wei_feas_ * df * 2.0 * wheel_alpha_left * (2.0 * singul_container_[trajid] * vel / v_norm - mobile_base_wheel_base_ * dAlphadA) / (2.0 * mobile_base_wheel_radius_);
      gradj_2d += -wei_feas_ * df * 2.0 * wheel_alpha_left * mobile_base_wheel_base_ * dAlphadJ / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }

    // Right wheel rotational acceleration
    double wheel_alpha_right = (2.0 * singul_container_[trajid] * aTv / v_norm + mobile_base_wheel_base_ * alpha) / (2.0 * mobile_base_wheel_radius_);
    pen =  wheel_alpha_right * wheel_alpha_right - max_wheel_alpha_ * max_wheel_alpha_;
    f = 0; df = 0;
    if(smoothedL1(pen, 0.005, f, df)){
      cost_mm_feasible += wei_feas_ * f;
      gradv_2d += wei_feas_ * df * 2.0 * wheel_alpha_right * (2.0 * singul_container_[trajid] * (v_norm * acc - aTv * vel / v_norm) * vTv_inv + mobile_base_wheel_base_ * dAlphadV) / (2.0 * mobile_base_wheel_radius_);
      grada_2d += wei_feas_ * df * 2.0 * wheel_alpha_right * (2.0 * singul_container_[trajid] * vel / v_norm + mobile_base_wheel_base_ * dAlphadA) / (2.0 * mobile_base_wheel_radius_);
      gradj_2d += wei_feas_ * df * 2.0 * wheel_alpha_right * mobile_base_wheel_base_ * dAlphadJ / (2.0 * mobile_base_wheel_radius_);
      ret = true;
    }
    return ret;
  }
extern "C" int native_wheel_probe(const double *p, int gear, int dense,
                                  const double *x, double *out) {
  if(gear!=1 && gear!=-1)return -1;
  PolyTrajOptimizer o;
  o.B_h_ << 0.,-1.,1.,0.;
  o.singul_container_={gear};
  o.wei_feas_=p[0]; o.max_vel_=p[1]; o.non_singul_v_=p[2];
  o.mobile_base_wheel_base_=p[3]; o.mobile_base_wheel_radius_=p[4];
  o.max_wheel_omega_=p[5]; o.max_wheel_alpha_=p[6];
  Eigen::Vector2d gv=Eigen::Vector2d::Zero(),ga=gv,gj=gv;
  double cost=0.;
  bool active=o.feasibilityGradCostCar(Eigen::Vector2d(x[0],x[1]),
    Eigen::Vector2d(x[2],x[3]),Eigen::Vector2d(x[4],x[5]),0,gv,ga,gj,cost,dense!=0);
  out[0]=cost; out[1]=gv[0];out[2]=gv[1];out[3]=ga[0];out[4]=ga[1];
  out[5]=gj[0];out[6]=gj[1];
  return active?1:0;
}
