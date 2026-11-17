"""Pure transforms for an isolated active-arm child; never edit production.

Caller copies active_arm_coupling.hpp, then applies these functions to copied
sources. Every source anchor is checked exactly once; unknown layouts fail.
"""


def once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('active-arm source anchor count %d: %s' % (text.count(old), old[:100]))
    return text.replace(old, new, 1)


def transform_collab(text):
    text = once(text, '#include "failure_probe.hpp"', '#include "failure_probe.hpp"\n#include "active_arm_coupling.hpp"')
    text = once(text, '  std::vector<std::array<double,8>> rows;\n};',
                '  std::vector<std::array<double,8>> rows;\n  active_arm_coupling::Seed arm;\n};')
    text = once(text, '  output=Seed();\n  if(values.size()<10', '''  output=Seed();
  if(!values.empty()&&values[0]==2.){
    active_arm_coupling::Seed whole;
    if(!active_arm_coupling::parse(values,whole))return false;
    output.stamp=whole.stamp;output.generation=whole.generation;output.revision=whole.revision;
    output.goal=whole.goal;output.odom=whole.odom;output.cloud=whole.cloud;output.dynamic=whole.dynamic;
    for(const auto& row:whole.rows)output.rows.push_back(row.base);
    output.arm=std::move(whole);return true;
  }
  if(values.size()<10''')
    text = once(text, 'std::vector<std::array<double,8>>& rows,std::string& reason) {',
                'std::vector<std::array<double,8>>& rows,std::string& reason,double* source_origin=nullptr) {')
    text = once(text, '  reason="eligible";return true;',
                '  if(source_origin)*source_origin=first[0];\n  reason="eligible";return true;')
    text = once(text, '  if(!prefixRows(head.col(0),yaw,sign,rows,reason))return reject(reason);',
                '  double arm_prefix_origin=0.;\n  if(!prefixRows(head.col(0),yaw,sign,rows,reason,&arm_prefix_origin))return reject(reason);')
    text = once(text, '  const std::size_t raw_samples=rows.size();', '''
  if(active_arm_coupling::armEnabled()){
    const auto& s=seed();const double now=ros::Time::now().toSec();
    if(head.rows()!=8||!active_arm_coupling::fresh(s.arm,now))return reject("active_arm_seed_missing_or_stale");
    const auto origin=active_arm_coupling::interpolate(s.arm,arm_prefix_origin);
    if((origin.q-head.col(0).tail(6)).cwiseAbs().maxCoeff()>.10 ||
       (origin.dq-head.col(1).tail(6)).cwiseAbs().maxCoeff()>.30)
      return reject("active_arm_prefix_measurement_mismatch");
  }
  const std::size_t raw_samples=rows.size();''')
    text = once(text, '  const int np=rows.size()-1;', '''  if(active_arm_coupling::armEnabled()){
    const auto arm=active_arm_coupling::interpolate(seed().arm,arm_prefix_origin+last[0]);
    middle.col(0).tail(6)=arm.q;middle.col(1).tail(6)=arm.dq;
    middle.col(2).tail(6)=arm.ddq;middle.col(3).tail(6)=arm.dddq;
  }
  const int np=rows.size()-1;''')
    text = once(text, '  poly_traj::MinSnapOpt<8> prefix;', '''  if(active_arm_coupling::armEnabled())for(int i=0;i<np-1;++i)
    prefix_points.col(i).tail(6)=active_arm_coupling::interpolate(seed().arm,arm_prefix_origin+rows[i+1][0]).q;
  poly_traj::MinSnapOpt<8> prefix;''')
    text = once(text, '  for(int i=1;i<nt;++i)merged_points.col(np+i-1)=native_points.col(i);', '''  if(active_arm_coupling::armEnabled())for(int i=0;i<np;++i)
    merged_points.col(i).tail(6)=active_arm_coupling::interpolate(seed().arm,arm_prefix_origin+rows[i+1][0]).q;
  for(int i=1;i<nt;++i)merged_points.col(np+i-1)=native_points.col(i);''')
    return text


def transform_bridge(text):
    text = once(text, '  coupling_objective::FrozenTargets targets;', '''  coupling_objective::FrozenTargets targets;
  active_arm_coupling::JointState arm_delivery;
  bool arm_valid=false;''')
    text = once(text, '    value.valid=!value.targets.execution.empty();', '''    if(active_arm_coupling::enabled()){
      if(start.size()!=8||!active_arm_coupling::fresh(s.arm,value.request_time)){
        value.reason="active_arm_seed_missing_or_stale";return;
      }
      const auto arm_origin=active_arm_coupling::interpolate(s.arm,begin);
      if((arm_origin.q-start.tail(6)).cwiseAbs().maxCoeff()>.10){
        value.reason="active_arm_measurement_mismatch";return;
      }
      if(value.targets.has_delivery){
        value.arm_delivery=active_arm_coupling::interpolate(s.arm,delivery);value.arm_valid=true;
      }
    }
    value.valid=!value.targets.execution.empty();''')
    text = once(text, '    if(!config.execution_enabled&&!config.delivery_enabled)', '''    if(active_arm_coupling::enabled()&&config.delivery_enabled&&!prediction.arm_valid){
      valid=false;reason="active_arm_delivery_unavailable";
    }
    if(!config.execution_enabled&&!config.delivery_enabled)''')
    text = once(text, '      .str("reason",reason).add("exec_weight",config.execution_weight)',
                '      .add("active_arm_coupling",active_arm_coupling::enabled()?"true":"false")\n      .str("reason",reason).add("exec_weight",config.execution_weight)')
    text = once(text, '  std::vector<coupling_objective::Sample> samples;', '''  std::vector<coupling_objective::Sample> samples;
  std::vector<active_arm_coupling::JointState> arm_samples;''')
    text = once(text, 's.acc=c.transpose()*b2;samples.push_back(s);', '''s.acc=c.transpose()*b2;samples.push_back(s);
    if(active_arm_coupling::enabled()){
      if(opts[0].get_b().cols()!=8)throw std::invalid_argument("active arm requires 8D slow curve");
      const auto ac=opts[0].get_b().block(i*8,2,8,6);active_arm_coupling::JointState arm;
      arm.q=ac.transpose()*b0;arm.dq=ac.transpose()*b1;arm.ddq=ac.transpose()*b2;arm_samples.push_back(arm);
    }''')
    text = once(text, '  const auto r=coupling_objective::evaluate(samples,f->prediction.targets,f->config);', '''  active_arm_coupling::Result whole;
  coupling_objective::Result r;
  if(active_arm_coupling::enabled()){
    whole=active_arm_coupling::evaluate(samples,arm_samples,f->prediction.targets,
      f->prediction.arm_delivery,f->config,active_arm_coupling::settings());r=whole.base;
  }else r=coupling_objective::evaluate(samples,f->prediction.targets,f->config);''')
    text = once(text, '    norm+=r.grad_p[k].squaredNorm()+r.grad_v[k].squaredNorm();', '''    if(active_arm_coupling::enabled()){
      opts[s.traj].get_gdC().block(s.piece*8,2,8,6)+=b0*whole.grad_q[k].transpose()+b1*whole.grad_dq[k].transpose();
      gradT[s.traj](s.piece)+=s.u*(whole.grad_q[k].dot(arm_samples[k].dq)+whole.grad_dq[k].dot(arm_samples[k].ddq));
      norm+=whole.grad_q[k].squaredNorm()+whole.grad_dq[k].squaredNorm();
    }
    norm+=r.grad_p[k].squaredNorm()+r.grad_v[k].squaredNorm();''')
    return text


def transform_optimizer_unfreeze(text):
    old = '''    // Experiment-only hard elimination of fixed arm coordinates.
    // L-BFGS starts at q_fixed, and all direction/history entries stay zero.
    for(auto &gp : gradP_container)
      gp.bottomRows(opt->manipulator_dof_).setZero();
    for(auto &gg : gradGear_container)
      gg.bottomRows(opt->manipulator_dof_).setZero();'''
    new = '''    // One binary: the A group retains the fixed-arm gradient elimination.
    if(!active_arm_coupling::armEnabled()){
      for(auto &gp : gradP_container)gp.bottomRows(opt->manipulator_dof_).setZero();
      for(auto &gg : gradGear_container)gg.bottomRows(opt->manipulator_dof_).setZero();
    }'''
    return once(text, old, new)


def transform_joint_feedback(text):
    start = text.index('    for(int i = 0; i < manipulator_dim_; ++i){', text.index('void REMANIReplanFSM::mmManiOdomCallback'))
    end = text.index('    // Fallback only:', start)
    new = '''    // Standard JointState effort is torque, never acceleration. Require
    // named finite q/dq; until an acceleration estimator exists use zero as an
    // explicit boundary-model approximation, not as a measured acceleration.
    Eigen::VectorXd q(manipulator_dim_),dq(manipulator_dim_);
    for(int i=0;i<manipulator_dim_;++i){
      const std::string name="joint"+std::to_string(i+1);
      const auto found=std::find(msg->name.begin(),msg->name.end(),name);
      if(found==msg->name.end()){ROS_WARN_THROTTLE(1.,"[ActiveArm] missing named joint");return;}
      const std::size_t index=std::distance(msg->name.begin(),found);
      if(index>=msg->position.size()||index>=msg->velocity.size()||
         !std::isfinite(msg->position[index])||!std::isfinite(msg->velocity[index])){
        ROS_WARN_THROTTLE(1.,"[ActiveArm] invalid q/dq feedback");return;
      }
      q[i]=msg->position[index];dq[i]=msg->velocity[index];
    }
    mm_state_pos_.tail(manipulator_dim_)=q;mm_state_vel_.tail(manipulator_dim_)=dq;
    mm_state_acc_.tail(manipulator_dim_).setZero();
'''
    return text[:start] + new + text[end:]


def transform_live_collision_export(text):
    # Current measured-q sphere snapshot only. The new fast executor MUST do
    # candidate FK; rotating this snapshot is not a moving-arm rollout check.
    text = once(text, '  Eigen::VectorXd fixed_q_;', '  Eigen::VectorXd fixed_q_,joint_lower_,joint_upper_;')
    start = text.index('  void joints(const sensor_msgs::JointState::ConstPtr& msg){')
    end = text.index('  void sample(', start)
    callback = '''  void joints(const sensor_msgs::JointState::ConstPtr& msg){
    Eigen::VectorXd q(fixed_q_.size());bool valid=msg->header.stamp.toSec()>0.;
    for(Eigen::Index i=0;valid&&i<q.size();++i){
      const std::string name="joint"+std::to_string(i+1);
      const auto found=std::find(msg->name.begin(),msg->name.end(),name);
      if(found==msg->name.end()){valid=false;break;}
      const std::size_t k=std::distance(msg->name.begin(),found);
      if(k>=msg->position.size()){valid=false;break;}q[i]=msg->position[k];
      valid=std::isfinite(q[i])&&q[i]>=joint_lower_[i]-1e-6&&q[i]<=joint_upper_[i]+1e-6;
    }
    q_matches_=valid;
    if(valid){fixed_q_=q;joint_stamp_=msg->header.stamp.toSec();}
  }
'''
    text = text[:start] + callback + text[end:]
    old = '''    if(lo.empty() || lo!=hi || lo.size()!=std::size_t(model_->getManiDof()) ||
       !std::all_of(lo.begin(),lo.end(),[](double q){return std::isfinite(q);}))
      throw std::runtime_error("native collision export requires exactly fixed finite arm limits");
    fixed_q_=Eigen::Map<const Eigen::VectorXd>(lo.data(),lo.size());'''
    new = '''    if(lo.empty()||lo.size()!=hi.size()||lo.size()!=std::size_t(model_->getManiDof()))
      throw std::runtime_error("native collision export requires finite joint limits");
    for(std::size_t i=0;i<lo.size();++i)if(!std::isfinite(lo[i])||!std::isfinite(hi[i])||lo[i]>hi[i])
      throw std::runtime_error("invalid active joint bounds");
    joint_lower_=Eigen::Map<const Eigen::VectorXd>(lo.data(),lo.size());
    joint_upper_=Eigen::Map<const Eigen::VectorXd>(hi.data(),hi.size());
    fixed_q_=.5*(joint_lower_+joint_upper_); // not healthy until a named joint message arrives
'''
    text = once(text, old, new)
    text = once(text, '    Eigen::Vector3i lo,hi;', '''    // Cover the entire allowed arm sweep in z, not just today's posture.
    // Unknown/outside-map queries remain invalid at the consumer.
    lower.z()=std::min(lower.z(),-.10);upper.z()=std::max(upper.z(),2.20);
    Eigen::Vector3i lo,hi;''')
    text = once(text, '    h[4]=h[4]==1. && h[19]==1.?1.:0.;',
                '    h[4]=h[4]==1. && h[19]==1. && now.toSec()-h[24]>=-.05 && now.toSec()-h[24]<=max_age_?1.:0.;')
    text = text.replace('native self-collision policy retained (Piper self-check remains disabled)',
                        'measured-q snapshot only; candidate FK and original self-collision policy require separate validation')
    return text
