
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
namespace remani_planner {
struct MMState{
        double time_stamp;
        Eigen::Vector2d car_pos;
        Eigen::Vector2d car_vel;
        double car_yaw;
        double car_v; // >0 foreward <0 backward
        double car_a; // >0 foreward <0 backward
        double car_omega;
        double car_domega;
        Eigen::Matrix2d car_R;
        double car_input_vel;
        double car_input_omega;
        Eigen::VectorXd mani_pos;
        Eigen::VectorXd mani_vel;

        Eigen::Vector3d getCarXYYaw(){
            return Eigen::Vector3d(car_pos(0), car_pos(1), car_yaw);
        }

        Eigen::Vector4d getCarXYYawV(){
            return Eigen::Vector4d(car_pos(0), car_pos(1), car_yaw, car_v);
        }
    };
}
#include "backend_component_probe_minco.hpp"
#include "coupling_collision.hpp"
namespace failure_probe {
enum { BASE_FEAS, INTEGRAL, SMOOTH };
struct Timer { explicit Timer(int) {} };
}
class PolyTrajOptimizer {
public:
 int traj_dim_=8,manipulator_dof_=6,dense_sample_resolution_=20;
 bool mani_hold_enabled_=false; double wei_mani_hold_=0.;
 Eigen::VectorXd mani_hold_reference_=Eigen::VectorXd::Zero(6);
 Eigen::Matrix2d B_h_;
 std::vector<int> singul_container_;
 std::vector<poly_traj::MinSnapOpt<8>> SnapOpt_container_;
 struct Cps { Eigen::MatrixXd points; }; std::vector<Cps> cps_container_;
 double wei_feas_,max_vel_,non_singul_v_,mobile_base_wheel_base_,
   mobile_base_wheel_radius_,max_wheel_omega_,max_wheel_alpha_,wei_time_;
 bool smoothedL1(const double&,const double&,double&,double&);
 bool smoothedLog(const double&,const double&,double&,double&);
 bool feasibilityGradCostCar(const Eigen::Vector2d&,const Eigen::Vector2d&,
   const Eigen::Vector2d&,int,Eigen::Vector2d&,Eigen::Vector2d&,Eigen::Vector2d&,double&,bool);
 template<typename V> void addPVAJGradCost2CT(int,V&,Eigen::VectorXd&,const int&);
 template<typename V> void VirtualT2RealT(const V&,Eigen::VectorXd&);
 template<typename V,typename G> void VirtualTGradCost(const Eigen::VectorXd&,const V&,
   const Eigen::VectorXd&,G&,double&);
 bool obstacleGradCostforMM(int,const Eigen::VectorXd&,const Eigen::VectorXd&,int,
   Eigen::VectorXd&,Eigen::VectorXd&,double&,double&,double&) {return false;}
 bool feasibilityGradCostJoint(const Eigen::VectorXd&,const Eigen::VectorXd&,
   const Eigen::VectorXd&,Eigen::VectorXd&,Eigen::VectorXd&,Eigen::VectorXd&,double&)
   {return false;}
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
bool PolyTrajOptimizer::smoothedLog(const double &x, const double &mu, double &l, double &grad){
    if (x <= -mu){
      l = 0.0;
      grad = 0.0;
      return false;
    }else if (x <= 0){
      double e1 = x + mu;
      double e1_2 = e1 * e1;
      double mu_4 = 1.0 / (mu * mu * mu * mu);
      l = 0.5 * e1_2 * e1 * (mu - x) * mu_4;
      grad = e1_2 * (mu - 2 * x) * mu_4;
      return true;
    }else if(x <= mu){
      double e2 = x - mu;
      double e2_2 = e2 * e2;
      double mu_4 = 1.0 / (mu * mu * mu * mu);
      l = 0.5 * (x + mu) * e2_2 * e2 * mu_4 + 1.0;
      grad = e2_2 * (mu + 2 * x) * mu_4;
      return true;
    }else{
      l = 1.0;
      grad = 0.0;
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
template <typename EIGENVEC>
  void PolyTrajOptimizer::VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      RT(i) = VT(i) > 0.0 ? ((0.5 * VT(i) + 1.0) * VT(i) + 1.0)
                          : 1.0 / ((0.5 * VT(i) - 1.0) * VT(i) + 1.0);
    }
  }
template <typename EIGENVEC, typename EIGENVECGD>
  void PolyTrajOptimizer::VirtualTGradCost(
      const Eigen::VectorXd &RT, const EIGENVEC &VT,
      const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
      double &costT)
  {
    for (int i = 0; i < VT.size(); ++i)
    {
      double gdVT2Rt;
      if (VT(i) > 0)
      {
        gdVT2Rt = VT(i) + 1.0;
      }
      else
      {
        double denSqrt = (0.5 * VT(i) - 1.0) * VT(i) + 1.0;
        gdVT2Rt = (1.0 - VT(i)) / (denSqrt * denSqrt);
      }

      gdVT(i) = (gdRT(i) + wei_time_) * gdVT2Rt;
    }

    costT = RT.sum() * wei_time_;
  }
template <typename EIGENVEC>
  void PolyTrajOptimizer::addPVAJGradCost2CT(const int trajid, EIGENVEC &gdT, Eigen::VectorXd &costs, const int &K){
    failure_probe::Timer timer(failure_probe::INTEGRAL);
    costs.setZero();
    int N = gdT.size();
    Eigen::VectorXd pos(traj_dim_), pos_next(traj_dim_), vel(traj_dim_), acc(traj_dim_), jer(traj_dim_), snap(traj_dim_);
    Eigen::VectorXd dense_pos(traj_dim_), dense_vel(traj_dim_), dense_acc(traj_dim_), dense_jer(traj_dim_), dense_sna(traj_dim_);
    Eigen::VectorXd gradp(traj_dim_), gradv(traj_dim_), grada(traj_dim_), gradj(traj_dim_), gradp_traj_piece(traj_dim_);
    Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4; 
    Eigen::Matrix<double, 8, 1> Densebeta0, Densebeta1, Densebeta2, Densebeta3, Densebeta4;
    double s1, s2, s3, s4, s5, s6, s7;
    double step, alpha;
    Eigen::MatrixXd gradViolaPc, gradViolaVc, gradViolaAc, gradViolaJc;
    Eigen::MatrixXd gradViolaVc_dense, gradViolaAc_dense, gradViolaJc_dense;
    gradViolaPc.resize(8, traj_dim_);
    gradViolaVc.resize(8, traj_dim_);
    gradViolaAc.resize(8, traj_dim_);
    gradViolaJc.resize(8, traj_dim_);
    gradViolaVc_dense.resize(8, traj_dim_);
    gradViolaAc_dense.resize(8, traj_dim_);
    gradViolaJc_dense.resize(8, traj_dim_);
    double gradViolaPt, gradViolaVt, gradViolaAt, gradViolaJt, gradViolaVt_dense, gradViolaAt_dense, gradViolaJt_dense;

    double omg, dense_omg;
    int i_dp = 0;

    for (int i = 0; i < N; ++i){
      
      Eigen::MatrixXd c;
      c.resize(8, traj_dim_);
      c = SnapOpt_container_[trajid].get_b().block(i * 8, 0, 8, traj_dim_);
      
      step = SnapOpt_container_[trajid].get_T1()(i) / K;
      s1 = 0.0;
      gradp_traj_piece.setZero();

      
      for (int j = 0; j <= K; ++j){
        s2 = s1 * s1;
        s3 = s2 * s1;
        s4 = s2 * s2;
        s5 = s4 * s1;
        s6 = s3 * s3;
        s7 = s4 * s3;
        beta0(0) = 1.0, beta0(1) = s1, beta0(2) = s2, beta0(3) = s3, beta0(4) = s4, beta0(5) = s5, beta0(6) = s6, beta0(7) = s7;
        beta1(0) = 0.0, beta1(1) = 1.0, beta1(2) = 2.0 * s1, beta1(3) = 3.0 * s2, beta1(4) = 4.0 * s3, beta1(5) = 5.0 * s4, beta1(6) = 6.0 * s5, beta1(7) = 7.0 * s6;
        beta2(0) = 0.0, beta2(1) = 0.0, beta2(2) = 2.0, beta2(3) = 6.0 * s1, beta2(4) = 12.0 * s2, beta2(5) = 20.0 * s3, beta2(6) = 30.0 * s4, beta2(7) = 42.0 * s5;
        beta3(0) = 0.0, beta3(1) = 0.0, beta3(2) = 0.0, beta3(3) = 6.0, beta3(4) = 24.0 * s1, beta3(5) = 60.0 * s2, beta3(6) = 120.0 * s3, beta3(7) = 210.0 * s4;
        beta4(0) = 0.0, beta4(1) = 0.0, beta4(2) = 0.0, beta4(3) = 0.0, beta4(4) = 24.0, beta4(5) = 120.0 * s1, beta4(6) = 360.0 * s2, beta4(7) = 840.0 * s3;
        alpha = 1.0 / K * j;
        pos = c.transpose() * beta0;
        vel = c.transpose() * beta1;
        acc = c.transpose() * beta2;
        jer = c.transpose() * beta3;
        snap = c.transpose() * beta4;

        omg = (j == 0 || j == K) ? 0.5 : 1.0;
        cps_container_[trajid].points.col(i_dp) = pos;
        // collision
        gradp.setZero();
        gradv.setZero();
        double cost_car = 0;
        double cost_mani = 0;
        double cost_self = 0;
        
        if(obstacleGradCostforMM(i_dp, pos, vel, trajid, gradp, gradv, cost_car, cost_mani, cost_self))
        {
          gradp_traj_piece += gradp;
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaPc;
          gdT(i) += omg * ((cost_car + cost_mani + cost_self) / K + step * gradViolaPt);

          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaVc;
          gdT(i) += omg * (step * gradViolaVt);

          costs(0) += omg * step * cost_car;
          costs(1) += omg * step * cost_mani;
          costs(2) += omg * step * cost_self;
        }

        if(mani_hold_enabled_){
          gradp.setZero();
          const Eigen::VectorXd arm_error = pos.tail(manipulator_dof_) - mani_hold_reference_;
          const double cost_mani_hold = wei_mani_hold_ * arm_error.squaredNorm();
          gradp.tail(manipulator_dof_) = 2.0 * wei_mani_hold_ * arm_error;
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaPc;
          gdT(i) += omg * (cost_mani_hold / K + step * gradViolaPt);
          costs(5) += omg * step * cost_mani_hold;
        }
        
        // car feasibility
        Eigen::Vector2d gradv_2d, grada_2d, gradj_2d;
        gradv_2d.setZero(); grada_2d.setZero(); gradj_2d.setZero();
        double cost_mm_feasible = 0.0;
        if(feasibilityGradCostCar(vel.head(2), acc.head(2), jer.head(2), trajid,
                                  gradv_2d, grada_2d, gradj_2d,
                                  cost_mm_feasible, false)){
          gradv.setZero(); grada.setZero(); gradj.setZero();
          gradv.head(2) = gradv_2d; grada.head(2) = grada_2d; gradj.head(2) = gradj_2d;

          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaVc;
          gdT(i) += omg * (cost_mm_feasible / K + step * gradViolaVt);

          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaAc;
          gdT(i) += omg * (step * gradViolaAt);

          gradViolaJc = beta3 * gradj.transpose();
          gradViolaJt = alpha * gradj.transpose() * snap;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaJc;
          gdT(i) += omg * (step * gradViolaJt);

          costs(3) += omg * step * cost_mm_feasible;
        }
        
        // dense sampling
        double min_dense_vel = 0.1; 
        double pen_min_vel = min_dense_vel * min_dense_vel - vel.head(2).squaredNorm();
        double pen_min_vel_log, grad_pen_min_vel_log;
        if(smoothedLog(pen_min_vel, 0.001, pen_min_vel_log, grad_pen_min_vel_log)){
          double special_step = step / dense_sample_resolution_;
          double special_s1 = s1;
          int disQuantity;
          double dense_alpha;
          Eigen::VectorXd gradv_dense(traj_dim_), grada_dense(traj_dim_), gradj_dense(traj_dim_);
          if(j == 0){
            disQuantity = dense_sample_resolution_ / 2;
            dense_alpha = 1.0 / K * j - 1.0 / K / dense_sample_resolution_;
          }else if(j == K){
            special_s1 = special_s1 - step / 2.0;
            disQuantity = dense_sample_resolution_ / 2;
            dense_alpha = 1.0 / K * j - 0.5 / K - 1.0 / K / dense_sample_resolution_;
          }else{
            special_s1 = special_s1 - step / 2.0;
            disQuantity = dense_sample_resolution_;
            dense_alpha = 1.0 / K * j - 0.5 / K - 1.0 / K / dense_sample_resolution_;
          }

          for(int l = 0; l <= disQuantity; l++){
            s2 = special_s1 * special_s1;
            s3 = s2 * special_s1;
            s4 = s2 * s2;
            s5 = s4 * special_s1;
            s6 = s3 * s3;
            s7 = s4 * s3;

            Densebeta0 << 1.0, special_s1, s2, s3, s4, s5, s6, s7;
            Densebeta1 << 0.0, 1.0, 2.0 * special_s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
            Densebeta2 << 0.0, 0.0, 2.0, 6.0 * special_s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
            Densebeta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * special_s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
            Densebeta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * special_s1, 360.0 * s2, 840 * s3;
            dense_alpha += 1.0 / K / dense_sample_resolution_;

            special_s1 += special_step;

            dense_pos = c.transpose() * Densebeta0;
            dense_vel = c.transpose() * Densebeta1;
            dense_acc = c.transpose() * Densebeta2;
            dense_jer = c.transpose() * Densebeta3;
            dense_sna = c.transpose() * Densebeta4;
            
            dense_omg = (l == 0 || l == disQuantity) ? 0.5 : 1.0;

            gradv_2d.setZero(); grada_2d.setZero(); gradj_2d.setZero();
            gradv_dense.setZero(); grada_dense.setZero(); gradj_dense.setZero(); gradv.setZero();
            double cost_mm_feasible = 0.0;
            if(feasibilityGradCostCar(dense_vel.head(2), dense_acc.head(2), dense_jer.head(2), trajid,
                                      gradv_2d, grada_2d, gradj_2d,
                                      cost_mm_feasible, true)){
              double cost_dense_log = pen_min_vel_log * cost_mm_feasible;
              
              gradv.head(2)       = -2 * grad_pen_min_vel_log * cost_mm_feasible * vel.head(2);
              gradv_dense.head(2) = pen_min_vel_log * gradv_2d;
              grada_dense.head(2) = pen_min_vel_log * grada_2d;
              gradj_dense.head(2) = pen_min_vel_log * gradj_2d;

              gradViolaVc = beta1 * gradv.transpose();
              gradViolaVt = alpha * gradv.transpose() * acc;
              gradViolaVc_dense = Densebeta1  * gradv_dense.transpose();
              gradViolaVt_dense = dense_alpha * gradv_dense.transpose() * dense_acc;
              gradViolaAc_dense = Densebeta2  * grada_dense.transpose();
              gradViolaAt_dense = dense_alpha * grada_dense.transpose() * dense_jer;
              gradViolaJc_dense = Densebeta3  * gradj_dense.transpose();
              gradViolaJt_dense = dense_alpha * gradj_dense.transpose() * dense_sna;

              SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += dense_omg * special_step * (gradViolaVc + gradViolaVc_dense + gradViolaAc_dense + gradViolaJc_dense);
              gdT(i) += dense_omg * (cost_dense_log / K / dense_sample_resolution_ + special_step * (gradViolaVt + gradViolaVt_dense + gradViolaAt_dense + gradViolaJt_dense));
              costs(3) += dense_omg * special_step * cost_dense_log;
            }
          }
        }
        // joint feasibility
        gradp.setZero(); gradv.setZero(); grada.setZero();
        double cost_joint_feasible = 0.0;
        if(feasibilityGradCostJoint(pos, vel, acc, gradp, gradv, grada, cost_joint_feasible)){
          gradViolaPc = beta0 * gradp.transpose();
          gradViolaPt = alpha * gradp.transpose() * vel;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaPc;
          gdT(i) += omg * (cost_joint_feasible / K + step * gradViolaPt);

          gradViolaVc = beta1 * gradv.transpose();
          gradViolaVt = alpha * gradv.transpose() * acc;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaVc;
          gdT(i) += omg * (step * gradViolaVt);

          gradViolaAc = beta2 * grada.transpose();
          gradViolaAt = alpha * grada.transpose() * jer;
          SnapOpt_container_[trajid].get_gdC().block(i * 8, 0, 8, traj_dim_) += omg * step * gradViolaAc;
          gdT(i) += omg * (step * gradViolaAt);

          costs(4) += omg * step * cost_joint_feasible;
        }
        s1 += step;
        if (j != K || (j == K && i == N - 1)){
          ++i_dp;
        }
      }
    }
  }
namespace coupling_geometry {
using Snapshot=shared_collision::Snapshot;
using Evaluation=coupling_collision::Evaluation;
inline bool enabled(){return true;}
static double probe_epoch=0.;
class Scope {
public:
 bool valid()const {auto f=shared_collision::snapshot();return f&&shared_collision::fresh(*f,probe_epoch);}
 std::shared_ptr<const Snapshot> snapshot()const{return shared_collision::snapshot();}
 double now()const{return probe_epoch;}
};
inline Evaluation staticProjectedPose(const Snapshot& f,const Eigen::Vector3d& q){
 coupling_collision::Options op;op.include_dynamic=false;
 return coupling_collision::evaluate(f,q.x(),q.y(),q.z(),0.,op);
}
}
namespace delivery_diag {
template<class... T> void rememberSolve(const T&...){}
// The captured valid static curves never enter geometryError; fail loudly if they do.
template<class A,class B,class C,class E,class F,class G,class H,class I,class J,class K,class L,class M,class N,class O,class P>
void geometryError(const A&,const B&,const C&,std::pair<std::shared_ptr<const shared_collision::Snapshot>,double>,
 const E&,const F&,const G&,const H&,const I&,const J&,const K&,const L&,const M&,const N&,const O&){
 throw std::runtime_error("invalid geometry probe sample");
}
}
namespace domain_step {struct RejectedTrial:std::runtime_error {
template<class... T> explicit RejectedTrial(T...):std::runtime_error("outside projected map"){} };}
namespace coupling_native {
constexpr int geometry_quadrature_intervals=8;
inline bool domainRecoveryEnabled(){return false;}}

namespace coupling_geometry {
struct ObjectiveSample {
  double cost=0.,gt=0.;
  Eigen::Vector2d gp=Eigen::Vector2d::Zero(),gv=Eigen::Vector2d::Zero();
  bool valid=false,dynamic_known=false;
  Evaluation::Failure failure=Evaluation::Failure::malformed;
  // valid covers only this additive scalar objective. dynamic_known indicates
  // forecast coverage, NOT collision freedom. Neither certifies native
  // long-horizon dynamic feasibility.
};
inline ObjectiveSample projectionCost(const Snapshot& frame,const Eigen::Vector2d& p,
                                     const Eigen::Vector2d& v,int gear,
                                     double absolute_query_time) {
  ObjectiveSample out;
  if (!p.allFinite() || !v.allFinite() || (gear!=-1 && gear!=1) ||
      !std::isfinite(absolute_query_time)) return out;
  const double forecast=absolute_query_time-frame.stamp;
  if (forecast<0. || !std::isfinite(forecast)) return out;
  // Exact native heading convention; dYaw/dv is gear-independent. At zero
  // speed the original native atan2 convention is not differentiable. This
  // objective uses its bounded native derivative; hard endpoint checks must
  // use their explicit pose yaw, not reconstruct it from zero velocity.
  const double yaw=std::atan2(gear*v.y(),gear*v.x());
  const auto stat=staticProjectedPose(frame,Eigen::Vector3d(p.x(),p.y(),yaw));
  if (!stat.valid) { out.failure=stat.failure; return out; }
  double gx=stat.gx,gy=stat.gy,gyaw=stat.gyaw;
  out.cost=stat.cost;out.gt=stat.gt;
  out.dynamic_known=std::isfinite(forecast) && forecast>=0. &&
    std::isfinite(frame.max_prediction) && frame.max_prediction>0. &&
    forecast<=frame.max_prediction;
  if (forecast<0. || !std::isfinite(forecast)) return ObjectiveSample{};
  if (out.dynamic_known) {
    coupling_collision::Options options;options.include_static=false;
    options.dynamic_soft_taper_seconds=std::min(.25,frame.max_prediction);
    const auto dyn=coupling_collision::evaluate(frame,p.x(),p.y(),yaw,forecast,options);
    if (!dyn.valid) return ObjectiveSample{};
    out.cost+=dyn.cost;gx+=dyn.gx;gy+=dyn.gy;gyaw+=dyn.gyaw;out.gt+=dyn.gt;
  }
  // Outside the prediction horizon ONLY THE ADDITIONAL DYNAMIC OBJECTIVE is
  // absent. Original native conservative costs and constraints stay active.
  // C2 taper makes this additive objective/gradient continuous at the cutoff;
  // shortDynamicPose still returns UNKNOWN beyond it for hard checks.
  out.gp=Eigen::Vector2d(gx,gy);
  out.gv=gyaw*Eigen::Vector2d(-v.y(),v.x())/std::max(1e-8,v.squaredNorm());
  out.valid=std::isfinite(out.cost) && std::isfinite(out.gt) &&
            out.gp.allFinite() && out.gv.allFinite();
  if (out.valid) out.failure=Evaluation::Failure::none;
  return out;
}
}

namespace coupling_native {
template<class SnapOptContainer,class TimeContainer,class SignContainer,class GradContainer>
double addGeometry(SnapOptContainer& opts,const TimeContainer& real_times,
                   const SignContainer& signs,GradContainer& gradT,int dim) {
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
          throw std::runtime_error("invalid projected geometry sample");
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
}

static thread_local std::string probe_error;
extern "C" const char* backend_component_probe_error(){return probe_error.c_str();}
extern "C" int backend_component_evaluate(int blocks,const int* pieces,const int* gears,
 const double* ini,const double* fin,const double* params,int snapshot_n,const double* snapshot,
 double epoch,int nx,const double* xv,double* costs,double* gradients,double* coeff,double* times_out){
 try {
  if(blocks<1||blocks>30)throw std::invalid_argument("invalid block count");
  int total=0,np=0;for(int t=0;t<blocks;++t){
    if(pieces[t]<1||pieces[t]>1000||(gears[t]!=1&&gears[t]!=-1))throw std::invalid_argument("bad pieces/gear");
    total+=pieces[t];np+=8*(pieces[t]-1);
  }
  if(nx!=np+total+9*(blocks-1))throw std::invalid_argument("wrong x dimension");
  Eigen::Map<const Eigen::VectorXd>x(xv,nx);
  if(!x.allFinite())throw std::invalid_argument("nonfinite x");
  if(!shared_collision::bind(snapshot,snapshot_n))throw std::invalid_argument("invalid frozen map transport");
  if(!shared_collision::snapshot()->obstacles.empty())throw std::invalid_argument("static probe cannot replay dynamic actors");
  coupling_geometry::probe_epoch=epoch;
  PolyTrajOptimizer opt;opt.SnapOpt_container_.resize(blocks);opt.cps_container_.resize(blocks);
  opt.B_h_<<0.,-1.,1.,0.;opt.singul_container_.assign(gears,gears+blocks);
  opt.wei_feas_=params[0];opt.max_vel_=params[1];opt.non_singul_v_=params[2];
  opt.mobile_base_wheel_base_=params[3];opt.mobile_base_wheel_radius_=params[4];
  opt.max_wheel_omega_=params[5];opt.max_wheel_alpha_=params[6];
  const double time_weight=params[7];const int K=int(params[8]);
  opt.dense_sample_resolution_=int(params[9]);const bool opt_gear=params[10]!=0.;
  if(K<1||opt.dense_sample_resolution_<2)throw std::invalid_argument("bad quadrature resolution");
  std::vector<Eigen::MatrixXd>P(blocks),I(blocks),F(blocks);
  std::vector<Eigen::VectorXd>T(blocks),vt(blocks);
  int offset=0;
  for(int t=0;t<blocks;++t){P[t]=Eigen::Map<const Eigen::MatrixXd>(xv+offset,8,pieces[t]-1);offset+=8*(pieces[t]-1);}
  for(int t=0;t<blocks;++t){vt[t]=x.segment(offset,pieces[t]);T[t].resize(pieces[t]);
    opt.VirtualT2RealT(vt[t],T[t]);offset+=pieces[t];}
  const int gear_offset=offset,angle_offset=offset+8*(blocks-1);
  int output_piece=0;
  for(int t=0;t<blocks;++t){
    I[t]=Eigen::Map<const Eigen::Matrix<double,8,4>>(ini+32*t);
    F[t]=Eigen::Map<const Eigen::Matrix<double,8,4>>(fin+32*t);
    if(t>0){double a=x[angle_offset+t-1];I[t].col(0)=x.segment(gear_offset+8*(t-1),8);
      I[t].col(1).head(2)=Eigen::Vector2d(-opt.non_singul_v_*cos(a),-opt.non_singul_v_*sin(a));}
    if(t<blocks-1){double a=x[angle_offset+t];F[t].col(0)=x.segment(gear_offset+8*t,8);
      F[t].col(1).head(2)=Eigen::Vector2d(opt.non_singul_v_*cos(a),opt.non_singul_v_*sin(a));}
    opt.SnapOpt_container_[t].reset(pieces[t]);
    opt.SnapOpt_container_[t].generate(P[t],T[t],I[t],F[t]);
    opt.cps_container_[t].points.resize(8,pieces[t]*K+1);
    for(int p=0;p<pieces[t];++p){times_out[output_piece]=T[t][p];
      for(int r=0;r<8;++r)for(int c=0;c<8;++c)
        coeff[output_piece*64+r*8+c]=opt.SnapOpt_container_[t].get_b()(p*8+r,c);
      ++output_piece;
    }
  }
  for(int component=0;component<4;++component){
    Eigen::Map<Eigen::VectorXd>g(gradients+component*nx,nx);g.setZero();
    std::vector<Eigen::VectorXd>gdT(blocks);costs[component]=0.;
    for(int t=0;t<blocks;++t){gdT[t]=Eigen::VectorXd::Zero(pieces[t]);
      opt.SnapOpt_container_[t].get_gdC().setZero();
      if(component==0){double c=0.;opt.SnapOpt_container_[t].initGradCost(gdT[t],c);costs[component]+=c;}
      if(component==2){Eigen::VectorXd c(6);opt.addPVAJGradCost2CT(t,gdT[t],c,K);costs[component]+=c[3];}
    }
    if(component==3)costs[component]=coupling_native::addGeometry(opt.SnapOpt_container_,T,
      opt.singul_container_,gdT,8);
    opt.wei_time_=component==1?time_weight:0.;
    int po=0,to=np;
    for(int t=0;t<blocks;++t){
      Eigen::Map<Eigen::MatrixXd>gp(g.data()+po,8,pieces[t]-1);po+=8*(pieces[t]-1);
      Eigen::MatrixXd gi,gf;opt.SnapOpt_container_[t].getGrad2TP(gdT[t],gp,gi,gf);
      if(opt_gear){
       if(t>0){double a=x[angle_offset+t-1];g.segment(gear_offset+8*(t-1),8)+=gi.col(0);
        g[angle_offset+t-1]+=gi.topRows(2).col(1).dot(Eigen::Vector2d(opt.non_singul_v_*sin(a),-opt.non_singul_v_*cos(a)));}
       if(t<blocks-1){double a=x[angle_offset+t];g.segment(gear_offset+8*t,8)+=gf.col(0);
        g[angle_offset+t]+=gf.topRows(2).col(1).dot(Eigen::Vector2d(-opt.non_singul_v_*sin(a),opt.non_singul_v_*cos(a)));}
      }
      Eigen::Map<Eigen::VectorXd>gt(g.data()+to,pieces[t]);to+=pieces[t];double ct=0.;
      opt.VirtualTGradCost(T[t],vt[t],gdT[t],gt,ct);costs[component]+=ct;
      gp.bottomRows(6).setZero();
    }
    for(int t=0;t<blocks-1;++t)g.segment(gear_offset+8*t+2,6).setZero();
    if(!std::isfinite(costs[component])||!g.allFinite())throw std::runtime_error("nonfinite component result");
  }
  probe_error.clear();return 0;
 }catch(const std::exception& e){probe_error=e.what();return -1;}
}
