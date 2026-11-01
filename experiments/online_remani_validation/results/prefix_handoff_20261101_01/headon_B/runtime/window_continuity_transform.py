"""Fixed-window dynamic environmental increment; native static/self stay intact."""
from native_collision_runtime import replace_once


def transform_optimizer(text):
    text = replace_once(text, '#include "coverage_collision.hpp"',
                        '#include "coverage_collision.hpp"\n#include "window_continuity.hpp"')
    anchor = '    failure_probe::Timer timer(failure_probe::INTEGRAL);\n    costs.setZero();'
    text = replace_once(text, anchor,
        '    window_continuity::Scope static_soft_cost(window_continuity::Mode::Static);\n' + anchor)
    # Static-only samples must bypass evaluatePlanningEDTWithGrad's swept layer.
    base_begin = text.index('      Eigen::Vector3d dist_grad;\n{failure_probe::Timer timer(failure_probe::EDT);')
    base_end = text.index('\n    }\n\n    if(manipulator_dof_ < 1)', base_begin)
    old = text[base_begin:base_end]
    text = text[:base_begin] + '''      if(window_continuity::enabled()){
        const auto penalty=window_continuity::environment(*grid_map_,car_pts[i],
          mm_config_->getBaseSphereRadius(i)+safe_margin_,environment_base_weight);
        costp+=penalty.cost;ret=ret||penalty.cost!=0.;
        gradp.head(mobile_base_dof_)+=penalty.gradient.head(2);
        gradv.head(mobile_base_dof_)+=penalty.gradient.head(2).transpose()*car_dPtsdYaw_list[i]*dYawdV;
      }else{
''' + old + '\n      }' + text[base_end:]

    arm_begin = text.index('        Eigen::Vector3d dist_grad;\n{failure_probe::Timer timer(failure_probe::EDT);')
    arm_end = text.index('\n        Eigen::VectorXd dzdP', arm_begin)
    old = text[arm_begin:arm_end]
    text = text[:arm_begin] + '''        if(window_continuity::enabled()){
          const auto penalty=window_continuity::environment(*grid_map_,pt_on_link,
            manipulator_thickness_+safe_margin_mani_,environment_arm_weight*factor);
          costp_mani+=penalty.cost;ret=ret||penalty.cost!=0.;
          Eigen::Vector4d gradient4=Eigen::Vector4d::Zero();gradient4.head(3)=penalty.gradient;
          for(int k=0;k<i+3;++k)
            gradp(k)+=(gradient4.transpose()*(T_grad_list[k]*manipulator_link_pts_[i].col(j)))(0,0);
          const double yaw_gradient=(gradient4.transpose()*(dTdYaw*manipulator_link_pts_[i].col(j)))(0,0);
          gradv.head(2)+=yaw_gradient*dYawdV;
        }else{
''' + old + '\n        }\n        if(window_continuity::increment())continue;' + text[arm_end:]
    text = replace_once(text, '    // check mani link with robot\n',
        '    if(window_continuity::increment())return ret;\n\n    // check mani link with robot\n')
    anchor = '''        if (j != K || (j == K && i == N - 1)){
          ++i_dp;
        }
      }
    }
  }

  bool PolyTrajOptimizer::obstacleGradCostforMM'''
    addition = '''        if (j != K || (j == K && i == N - 1)){
          ++i_dp;
        }
      }
      if(window_continuity::enabled()){
        candidate_collision_repair::sample_weight()=candidate_collision_repair::weight(trajid,i);
        window_continuity::integratePiece(trajid,i,c,K,SnapOpt_container_[trajid].get_gdC(),costs,
          [&](const Eigen::VectorXd& p,const Eigen::VectorXd& v,Eigen::VectorXd& gp,Eigen::VectorXd& gv,
              double& car,double& arm,double& self){
            obstacleGradCostforMM(0,p,v,trajid,gp,gv,car,arm,self);
          });
        candidate_collision_repair::sample_weight()=1.;
      }
    }
  }

  bool PolyTrajOptimizer::obstacleGradCostforMM'''
    return replace_once(text, anchor, addition)
