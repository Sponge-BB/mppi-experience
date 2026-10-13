/**
 * @file rrt_star.cpp
 * @brief RRT* 路径规划算法实现
 * 
 * 基于原 rrt.cpp 改写，保持相同的约束和接口，
 * 但采用纯 RRT* 思想（单树 + rewire + informed sampling）
 * 
 * 主要区别：
 * 1. 单树生长（从起点向目标），而非双向树
 * 2. 每次添加新节点后执行 rewire 操作优化树结构
 * 3. 找到初始路径后使用 Informed RRT* 在椭球区域内采样
 * 4. 持续优化直到时间用尽
 */

#include "path_searching/rrt_star.h"

namespace remani_planner {

// PathNodeRRTStar constructors/destructor
PathNodeRRTStar::PathNodeRRTStar() {
  parent = nullptr;
  node_state = RRT_NOT_EXPAND;
  g_score = 1e6;
  singul = 0;
  layer = 0;
  yaw = 0.0;
}

PathNodeRRTStar::PathNodeRRTStar(int dof) {
  parent = nullptr;
  node_state = RRT_NOT_EXPAND;
  state = Eigen::VectorXd::Zero(dof);
  g_score = 1e6;
  singul = 0;
  layer = 0;
  yaw = 0.0;
}

PathNodeRRTStar::~PathNodeRRTStar() {}

// RrtStarPlanning constructor/destructor
RrtStarPlanning::RrtStarPlanning() :
    random_gen_(std::random_device{}()),
    random_dis_(0.0, 1.0),
    norm_dis_(0.0, 1.0) {
  have_path_ = false;
  tree_count_ = 0;
  start_node_ = nullptr;
  goal_node_ = nullptr;
  best_goal_node_ = nullptr;
  c_best_ = 1e6;
}

RrtStarPlanning::~RrtStarPlanning() {
  for (auto it = node_pool_.begin(); it != node_pool_.end(); ++it) {
    delete it->second;
  }
  node_pool_.clear();
}

int RrtStarPlanning::RRTStarSearchAndGetSimplePath(
    const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<double>& start_yaw_list,
    const std::vector<Eigen::VectorXd>& end_pt_list, const std::vector<double>& end_yaw_list,
    const std::vector<double>& start_g_score_list, const std::vector<int>& start_layer_list,
    const std::vector<double>& end_g_score_list, const std::vector<int>& end_layer_list,
    const std::vector<int>& start_singul_list, const std::vector<int>& end_singul_list,
    std::vector<Eigen::VectorXd>& path, std::vector<double>& yaw_list, std::vector<double>& t_list)
{
  std::vector<Eigen::VectorXd> path_full;
  std::vector<double> t_list_full, yaw_list_full;

  if (start_pt_list.size() < 1) {
    ROS_ERROR("[RRT* plan]: start pt < 1");
    return false;
  }
  if (end_pt_list.size() < 1) {
    ROS_ERROR("[RRT* plan]: end pt < 1");
    return false;
  }

  init(start_pt_list, end_pt_list);
  bool status = search(start_pt_list, start_yaw_list, end_pt_list, end_yaw_list,
                       start_g_score_list, start_layer_list, end_g_score_list, end_layer_list,
                       start_singul_list, end_singul_list);

  if (status == false) {
    return status;
  }

  getTraj(path_full, yaw_list_full, t_list_full);

  path.clear();
  t_list.clear();
  yaw_list.clear();

  Eigen::VectorXd temp(traj_dim_);
  temp = path_full[0];
  path.push_back(temp);
  yaw_list.push_back(yaw_list_full[0]);

  double t_total = 0.0;
  for (size_t i = 0; i < t_list_full.size(); i++) {
    t_total += t_list_full[i];
    temp = path_full[i + 1];
    path.push_back(temp);
    yaw_list.push_back(yaw_list_full[i + 1]);
    t_list.push_back(t_total);
    t_total = 0.0;
  }

  return status;
}

bool RrtStarPlanning::search(
    const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<double>& start_yaw_list,
    const std::vector<Eigen::VectorXd>& end_pt_list, const std::vector<double>& end_yaw_list,
    const std::vector<double>& start_g_score_list, const std::vector<int>& start_layer_list,
    const std::vector<double>& end_g_score_list, const std::vector<int>& end_layer_list,
    const std::vector<int>& start_singul_list, const std::vector<int>& end_singul_list)
{
  ros::Time time_start = ros::Time::now();

  // 1. 初始化起点节点
  int start_num = start_pt_list.size();
  if (start_num < 1) {
    ROS_ERROR("[RRT*]: Fail! Start list size zero!");
    return false;
  }

  for (int i = 0; i < start_num; ++i) {
    PathNodeRRTStarPtr start_node = new PathNodeRRTStar(traj_dim_);
    start_node->state = start_pt_list[i];
    start_node->yaw = start_yaw_list[i];
    start_node->node_state = PathNodeRRTStar::RRT_IN_TREE;
    start_node->g_score = start_g_score_list[i];
    start_node->index = calculateIndex(start_node);
    start_node->layer = start_layer_list[i];
    start_node->singul = start_singul_list[i];
    node_pool_.insert(std::make_pair(start_node->index, start_node));
    ++tree_count_;

    if (i == 0) {
      start_node_ = start_node;
    }
  }

  // 2. 存储目标点信息（用于目标检测和偏置采样）
  int end_num = end_pt_list.size();
  if (end_num < 1) {
    ROS_ERROR("[RRT*]: Fail! End list size zero!");
    return false;
  }

  // 创建虚拟目标节点（用于采样偏置，不加入树中）
  goal_node_ = new PathNodeRRTStar(traj_dim_);
  goal_node_->state = end_pt_list[0];
  goal_node_->yaw = end_yaw_list[0];
  goal_node_->layer = end_layer_list[0];
  goal_node_->singul = end_singul_list[0];

  // 3. 主搜索循环
  int loop = 0;
  PathNodeRRTStarPtr q_new, q_nearest;
  Eigen::VectorXd rand_s;
  double rand_yaw;

  // 清除之前的可视化
  clearVisualization();

  while (true) {
    // 检查终止条件
    double elapsed = (ros::Time::now() - time_start).toSec();
    if ((have_path_ && elapsed > 0.01) || elapsed > max_sample_time_) {
      break;
    }

    ++loop;

    // 定期可视化树的生长过程
    if (enable_vis_ && (loop % vis_interval_ == 0)) {
      visualizeTree();
      if (have_path_) {
        visualizePath();
      }
    }

    // 4. 采样策略
    double rand_val = random_dis_(random_gen_);
    if (rand_val < goal_bias_ * 0.3 && !have_path_) {
      // 策略1: 直接采样目标点（30% 的 goal_bias 概率）
      int goal_idx = rand() % end_num;
      rand_s = end_pt_list[goal_idx];
      rand_yaw = end_yaw_list[goal_idx];
    } else if (rand_val < goal_bias_ && !have_path_) {
      // 策略2: 目标偏置采样 - 在树最近节点到目标方向上采样（70% 的 goal_bias 概率）
      sampleGoalBiased(rand_s, rand_yaw);
    } else if (have_path_ && c_best_ < 1e5) {
      // 策略3: Informed RRT* - 在椭球区域内采样（找到路径后优化）
      sampleInformed(rand_s, rand_yaw);
    } else {
      // 策略4: 均匀随机采样
      sample(rand_s, rand_yaw);
    }

    // 5. 找到最近节点
    q_nearest = nearest(rand_s, rand_yaw);
    if (q_nearest == nullptr) {
      continue;
    }

    // 6. Steer: 从最近节点向采样点扩展
    q_new = steer(q_nearest, rand_s, rand_yaw, time_resolution_);
    if (q_new == nullptr) {
      continue;
    }

    // 7. 如果是新节点，选择最佳父节点并加入树中
    if (q_new->node_state == PathNodeRRTStar::RRT_NOT_EXPAND) {
      // 在邻域内寻找最佳父节点
      std::vector<PathNodeRRTStarPtr> neighbors = nearNeighbors(q_new->state, q_new->yaw, rewire_radius_);

      PathNodeRRTStarPtr best_parent = q_nearest;
      double best_cost = q_nearest->g_score + estimateHeuristic(q_nearest, q_new);

      for (auto& neighbor : neighbors) {
        if (neighbor == q_new) continue;
        double new_cost = neighbor->g_score + estimateHeuristic(neighbor, q_new);
        if (new_cost < best_cost && !checkCollision(neighbor, q_new)) {
          best_cost = new_cost;
          best_parent = neighbor;
        }
      }

      // 连接到最佳父节点
      linkNode(best_parent, q_new);
      q_new->node_state = PathNodeRRTStar::RRT_IN_TREE;
      ++tree_count_;

      // 8. Rewire: 检查是否可以通过新节点改进邻居节点的路径
      for (auto& neighbor : neighbors) {
        if (neighbor == q_new || neighbor == best_parent) continue;
        double new_cost = q_new->g_score + estimateHeuristic(q_new, neighbor);
        if (new_cost < neighbor->g_score && !checkCollision(q_new, neighbor)) {
          linkNode(q_new, neighbor);
        }
      }

      // 9. 检查是否到达目标
      for (int i = 0; i < end_num; ++i) {
        double dist_to_goal = (q_new->state - end_pt_list[i]).norm();
        double yaw_diff = std::fabs(q_new->yaw - end_yaw_list[i]);
        if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff;

        if (dist_to_goal < goal_tolerance_ && yaw_diff < M_PI / 6.0) {
          // 检查是否能无碰撞连接到目标
          PathNodeRRTStarPtr temp_goal = createNode(end_pt_list[i], end_yaw_list[i]);
          if (!checkCollision(q_new, temp_goal)) {
            // 成功到达目标
            double path_cost = q_new->g_score + estimateHeuristic(q_new, temp_goal);
            if (!have_path_ || path_cost < c_best_) {
              have_path_ = true;
              c_best_ = path_cost;

              // 更新最佳目标节点
              linkNode(q_new, temp_goal);
              temp_goal->node_state = PathNodeRRTStar::RRT_IN_TREE;
              temp_goal->layer = end_layer_list[i];
              temp_goal->singul = end_singul_list[i];
              best_goal_node_ = temp_goal;

              ROS_INFO("[RRT*]: Found path with cost: %.3f, loop: %d, time: %.3f ms",
                       c_best_, loop, (ros::Time::now() - time_start).toSec() * 1000.0);
            }
          }
          break;
        }
      }
    }
    // 如果节点已存在且通过新路径更短，则更新
    else if (q_new->node_state == PathNodeRRTStar::RRT_IN_TREE) {
      double new_cost = q_nearest->g_score + estimateHeuristic(q_nearest, q_new);
      if (new_cost < q_new->g_score && !checkCollision(q_nearest, q_new)) {
        linkNode(q_nearest, q_new);
      }
    }
  }

  // 最终可视化
  visualizeTree();
  if (have_path_) {
    visualizePath();
  }

  if (!have_path_) {
    ROS_WARN("[RRT*]: Failed to find path. Tree size: %d, loop: %d, time: %.3f ms",
             tree_count_, loop, (ros::Time::now() - time_start).toSec() * 1000.0);
    return false;
  }

  ROS_INFO("[RRT*]: Search complete. Final cost: %.3f, tree size: %d, time: %.3f ms",
           c_best_, tree_count_, (ros::Time::now() - time_start).toSec() * 1000.0);

  return true;
}

PathNodeRRTStarPtr RrtStarPlanning::createNode(const Eigen::VectorXd& s, const double yaw) {
  auto it = node_pool_.find(calculateIndex(s, yaw));
  if (it != node_pool_.end()) {
    return it->second;
  }
  PathNodeRRTStarPtr node = new PathNodeRRTStar(traj_dim_);
  node->node_state = PathNodeRRTStar::RRT_NOT_EXPAND;
  node->state = s;
  node->yaw = yaw;
  node->index = calculateIndex(node);
  node_pool_.insert(std::make_pair(node->index, node));
  return node;
}

void RrtStarPlanning::sample(Eigen::VectorXd& s_state, double& s_yaw) {
  Eigen::VectorXd sample_state(traj_dim_);
  for (int i = 0; i < traj_dim_; ++i) {
    sample_state(i) = min_size_[i] + (max_size_[i] - min_size_[i]) * random_dis_(random_gen_);
  }
  s_state = sample_state;
  s_yaw = min_size_[traj_dim_] + (max_size_[traj_dim_] - min_size_[traj_dim_]) * random_dis_(random_gen_);
}

void RrtStarPlanning::sampleGoalBiased(Eigen::VectorXd& s_state, double& s_yaw) {
  // 目标偏置采样：在当前树最近节点到目标点的方向上采样
  // 这样可以引导树向目标方向生长，同时保持一定的随机性
  
  if (goal_node_ == nullptr || node_pool_.empty()) {
    sample(s_state, s_yaw);
    return;
  }

  // 找到树中距离目标最近的节点
  PathNodeRRTStarPtr nearest_to_goal = nullptr;
  double min_dist = 1e10;
  for (auto& pair : node_pool_) {
    if (pair.second->node_state == PathNodeRRTStar::RRT_IN_TREE) {
      double dist = (pair.second->state - goal_node_->state).norm();
      if (dist < min_dist) {
        min_dist = dist;
        nearest_to_goal = pair.second;
      }
    }
  }

  if (nearest_to_goal == nullptr) {
    sample(s_state, s_yaw);
    return;
  }

  // 计算从最近节点到目标的方向
  Eigen::VectorXd direction = goal_node_->state - nearest_to_goal->state;
  double total_dist = direction.norm();
  
  if (total_dist < 1e-6) {
    // 已经非常接近目标，直接采样目标点
    s_state = goal_node_->state;
    s_yaw = goal_node_->yaw;
    return;
  }
  
  direction.normalize();

  // 在方向上随机选择一个距离（从最近节点到目标之间）
  // 使用高斯分布，中心偏向目标方向的中间位置
  double rand_ratio = 0.3 + 0.7 * random_dis_(random_gen_);  // 0.3~1.0 之间
  double sample_dist = total_dist * rand_ratio;

  // 计算采样点的基础位置
  Eigen::VectorXd base_pos = nearest_to_goal->state + direction * sample_dist;

  // 添加横向扰动，使采样不完全沿直线
  // 扰动幅度随距离减小（越接近目标扰动越小）
  double lateral_noise_scale = 0.3 * (1.0 - rand_ratio);  // 越接近目标，扰动越小
  
  Eigen::VectorXd sample_state = base_pos;
  for (int i = 0; i < traj_dim_; ++i) {
    double noise = lateral_noise_scale * (max_size_[i] - min_size_[i]) * (random_dis_(random_gen_) - 0.5);
    sample_state(i) += noise;
    // 裁剪到有效范围
    sample_state(i) = std::max(min_size_[i], std::min(max_size_[i], sample_state(i)));
  }
  
  s_state = sample_state;

  // yaw 朝向目标方向，但加一些扰动
  double target_yaw = atan2(goal_node_->state(1) - nearest_to_goal->state(1),
                           goal_node_->state(0) - nearest_to_goal->state(0));
  double yaw_noise = 0.3 * (random_dis_(random_gen_) - 0.5);  // ±0.15 rad 扰动
  s_yaw = target_yaw + yaw_noise;
  
  // 归一化到 [-pi, pi]
  while (s_yaw > M_PI) s_yaw -= 2 * M_PI;
  while (s_yaw < -M_PI) s_yaw += 2 * M_PI;
}

void RrtStarPlanning::sampleInformed(Eigen::VectorXd& s_state, double& s_yaw) {
  // Informed RRT*: 在椭球区域内采样
  // 椭球的焦点为起点和终点，长轴长度为当前最佳路径代价

  Eigen::VectorXd s_center = (sample_start_ + sample_goal_) / 2.0;
  Eigen::VectorXd a1 = (sample_goal_ - sample_start_);
  double c_min_local = a1.norm();
  
  if (c_min_local < 1e-6 || c_best_ < c_min_local) {
    // 退化情况，使用普通采样
    sample(s_state, s_yaw);
    return;
  }
  
  a1.normalize();

  // 构建旋转矩阵
  Eigen::MatrixXd M = a1 * Eigen::VectorXd::Ones(traj_dim_).transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd V = svd.matrixV();
  Eigen::MatrixXd U = svd.matrixU();
  Eigen::MatrixXd C_middle(traj_dim_, traj_dim_);
  C_middle.setZero();
  for (int i = 0; i < traj_dim_ - 1; ++i) {
    C_middle(i, i) = 1.0;
  }
  C_middle(traj_dim_ - 1, traj_dim_ - 1) = U.determinant() * V.determinant();
  Eigen::MatrixXd C = U * C_middle;

  // 构建缩放矩阵
  Eigen::MatrixXd L(traj_dim_, traj_dim_);
  L.setZero();
  double c_max_local = c_best_;
  double semi_major = c_max_local / 2.0;
  double semi_minor = sqrt(c_max_local * c_max_local - c_min_local * c_min_local) / 2.0;

  L(0, 0) = semi_major;
  for (int i = 1; i < traj_dim_; ++i) {
    L(i, i) = semi_minor;
  }

  // 在单位球内均匀采样
  Eigen::VectorXd sample_state(traj_dim_);
  for (int i = 0; i < traj_dim_; ++i) {
    sample_state(i) = norm_dis_(random_gen_);
  }
  sample_state.normalize();
  double r = pow(random_dis_(random_gen_), 1.0 / traj_dim_);
  sample_state *= r;

  // 变换到椭球空间
  s_state = C * L * sample_state + s_center;
  s_yaw = min_size_[traj_dim_] + (max_size_[traj_dim_] - min_size_[traj_dim_]) * random_dis_(random_gen_);

  // 裁剪到有效范围
  for (int i = 0; i < traj_dim_; ++i) {
    s_state[i] = std::min(std::max(min_size_[i], s_state[i]), max_size_[i]);
  }
}

PathNodeRRTStarPtr RrtStarPlanning::nearest(const Eigen::VectorXd& s, const double yaw) {
  PathNodeRRTStarPtr q_nearest = nullptr;
  double min_dis = 1e6;

  for (auto& it : node_pool_) {
    PathNodeRRTStarPtr node = it.second;
    if (node->node_state != PathNodeRRTStar::RRT_IN_TREE) {
      continue;
    }
    double dist = estimateHeuristic(s, yaw, node->state, node->yaw);
    if (dist > 1e-2 && dist < min_dis) {
      min_dis = dist;
      q_nearest = node;
    }
  }
  return q_nearest;
}

std::vector<PathNodeRRTStarPtr> RrtStarPlanning::nearNeighbors(
    const Eigen::VectorXd& s, const double yaw, double radius_time) {
  std::vector<PathNodeRRTStarPtr> neighbors;

  for (auto& it : node_pool_) {
    PathNodeRRTStarPtr node = it.second;
    if (node->node_state != PathNodeRRTStar::RRT_IN_TREE) {
      continue;
    }

    // 检查是否在半径内
    bool in_range = true;
    if ((node->state.head(mobile_base_dof_) - s.head(mobile_base_dof_)).norm() > max_vel_ * radius_time) {
      in_range = false;
    }
    for (int j = 0; j < manipulator_dof_ && in_range; ++j) {
      if (calAngleErr(node->state(mobile_base_dof_ + j), s(mobile_base_dof_ + j)) > max_joint_vel_ * radius_time) {
        in_range = false;
      }
    }

    if (in_range) {
      neighbors.push_back(node);
    }
  }

  return neighbors;
}

PathNodeRRTStarPtr RrtStarPlanning::steer(
    PathNodeRRTStarPtr& q_near, const Eigen::VectorXd& s_rand,
    const double s_yaw, double step_time) {
  Eigen::VectorXd dir = s_rand - q_near->state;
  Eigen::VectorXd s_new(traj_dim_);

  // 确定运动方向（前进/后退）
  int singul;
  if (q_near->parent == nullptr) {
    singul = (((s_rand - q_near->state).head(2)).dot(Eigen::Vector2d(cos(q_near->yaw), sin(q_near->yaw)))) >= 0 ? 1 : -1;
  } else {
    singul = (((s_rand - q_near->state).head(2)).dot((q_near->state - q_near->parent->state).head(2))) >= 0 ? q_near->singul : -q_near->singul;
  }
  if (q_near->singul != 0 && singul != q_near->singul) {
    singul = q_near->singul;
  }

  // 使用 Dubins 曲线进行 steering
  ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_), s(dubins_curve_);
  from[0] = q_near->state[0];
  from[1] = q_near->state[1];
  if (singul == 1) {
    from[2] = q_near->yaw;
  } else {
    from[2] = q_near->yaw + M_PI;
    if (from[2] > M_PI) from[2] -= 2 * M_PI;
  }

  to[0] = s_rand[0];
  to[1] = s_rand[1];
  if (singul == 1) {
    to[2] = s_yaw;
  } else {
    to[2] = s_yaw + M_PI;
    if (to[2] > M_PI) to[2] -= 2 * M_PI;
  }

  double len = dubins_curve_->distance(from(), to());
  double yaw;

  if (len > max_vel_ * step_time) {
    dubins_curve_->interpolate(from(), to(), max_vel_ * step_time / len, s());
    auto reals = s.reals();
    s_new(0) = reals[0];
    s_new(1) = reals[1];
    yaw = reals[2];
  } else {
    s_new.head(mobile_base_dof_) = s_rand.head(mobile_base_dof_);
    yaw = to[2];
  }

  if (singul == -1) {
    yaw += M_PI;
    if (yaw > M_PI) yaw -= 2 * M_PI;
  }

  // 机械臂关节运动
  if (dir.tail(manipulator_dof_).norm() > 1e-2) {
    s_new.tail(manipulator_dof_) = q_near->state.tail(manipulator_dof_) +
                                    dir.tail(manipulator_dof_).normalized() * max_joint_vel_ * step_time;
  } else {
    s_new.tail(manipulator_dof_) = q_near->state.tail(manipulator_dof_);
  }

  // 裁剪到有效范围
  for (int i = mobile_base_dof_; i < traj_dim_; ++i) {
    if (s_new(i) < std::min(s_rand(i), q_near->state(i)) || s_new(i) > std::max(s_rand(i), q_near->state(i))) {
      s_new(i) = s_rand(i);
    }
  }

  // 碰撞检测
  if (checkCollision(q_near, s_new, yaw)) {
    return nullptr;
  }

  PathNodeRRTStarPtr q_new = createNode(s_new, yaw);
  q_new->singul = singul;
  return q_new;
}

void RrtStarPlanning::rewire(PathNodeRRTStarPtr q_new, double near_time) {
  std::vector<PathNodeRRTStarPtr> neighbors = nearNeighbors(q_new->state, q_new->yaw, near_time);

  // 1. 为 q_new 选择最佳父节点
  PathNodeRRTStarPtr best_parent = nullptr;
  double min_cost = q_new->g_score;

  for (auto& neighbor : neighbors) {
    if (neighbor == q_new) continue;
    double new_cost = neighbor->g_score + estimateHeuristic(neighbor, q_new);
    if (new_cost < min_cost && !checkCollision(neighbor, q_new)) {
      min_cost = new_cost;
      best_parent = neighbor;
    }
  }

  if (best_parent != nullptr) {
    linkNode(best_parent, q_new);
  }

  // 2. 检查是否可以通过 q_new 改进邻居的路径
  for (auto& neighbor : neighbors) {
    if (neighbor == q_new) continue;
    double new_cost = q_new->g_score + estimateHeuristic(q_new, neighbor);
    if (new_cost < neighbor->g_score && !checkCollision(q_new, neighbor)) {
      linkNode(q_new, neighbor);
    }
  }
}

void RrtStarPlanning::linkNode(PathNodeRRTStarPtr& parent, PathNodeRRTStarPtr& child) {
  if (parent == child) return;

  PathNodeRRTStarPtr pre_parent = child->parent;
  if (pre_parent == parent) {
    return;
  } else if (pre_parent != nullptr) {
    pre_parent->children.erase(child->index);
  }

  child->parent = parent;
  parent->children.insert(std::make_pair(child->index, child));

  // 确定运动方向
  int singul;
  if (parent->parent != nullptr) {
    singul = ((child->state - parent->state).head(2)).dot((parent->state - parent->parent->state).head(2)) >= 0 ? parent->singul : -parent->singul;
  } else {
    singul = ((child->state - parent->state).head(2)).dot(Eigen::Vector2d(cos(parent->yaw), sin(parent->yaw))) >= 0 ? 1 : -1;
  }

  child->singul = singul;
  if (parent->singul != 0 && singul != parent->singul) {
    child->singul = parent->singul;
  }

  propagateGscore(child);
}

void RrtStarPlanning::propagateGscore(PathNodeRRTStarPtr q) {
  double new_g = q->parent->g_score + estimateHeuristic(q->parent, q);
  if (std::fabs(q->g_score - new_g) < 1e-6) {
    return;
  }
  q->g_score = new_g;

  // 递归更新所有子节点
  for (auto& it : q->children) {
    propagateGscore(it.second);
  }
}

double RrtStarPlanning::estimateHeuristic(PathNodeRRTStarPtr& x1, PathNodeRRTStarPtr& x2) {
  return estimateHeuristic(x1->state, x1->yaw, x2->state, x2->yaw);
}

double RrtStarPlanning::estimateHeuristic(
    const Eigen::VectorXd& s1, const double& yaw1,
    const Eigen::VectorXd& s2, const double& yaw2) {
  double ret = sqrt((s1 - s2).squaredNorm() + (yaw1 - yaw2) * (yaw1 - yaw2));
  return ret;
}

std::string RrtStarPlanning::calculateIndex(PathNodeRRTStarPtr& q) {
  return calculateIndex(q->state, q->yaw);
}

std::string RrtStarPlanning::calculateIndex(const Eigen::VectorXd& s, const double yaw) {
  std::string ret;
  for (int i = 0; i < traj_dim_; ++i) {
    int k = round(s(i) * 100.0);
    ret += std::to_string(k);
  }
  ret += std::to_string(round(yaw * 100.0));
  return ret;
}

double RrtStarPlanning::calAngleErr(double angle1, double angle2) {
  return std::fabs(angle1 - angle2);
}

bool RrtStarPlanning::checkCollision(PathNodeRRTStarPtr& cur_state) {
  Eigen::Vector3d xt;
  xt[0] = cur_state->state[0];
  xt[1] = cur_state->state[1];
  xt[2] = cur_state->yaw;
  return mm_config_->checkcollision(xt, cur_state->state.tail(manipulator_dof_), false);
}

bool RrtStarPlanning::checkCollision(
    PathNodeRRTStarPtr& cur_state, const Eigen::VectorXd& next_state, const double next_yaw) {
  if (cur_state->node_state == PathNodeRRTStar::RRT_COLLISION) {
    return true;
  }

  // 确定运动方向
  int singul;
  if (cur_state->parent == nullptr) {
    singul = ((next_state - cur_state->state).head(2)).dot(Eigen::Vector2d(cos(cur_state->yaw), sin(cur_state->yaw))) >= 0 ? 1 : -1;
  } else {
    singul = ((next_state - cur_state->state).head(2)).dot((cur_state->state - cur_state->parent->state).head(2)) >= 0 ? cur_state->singul : -cur_state->singul;
  }
  if (cur_state->singul != 0 && singul != cur_state->singul) {
    singul = cur_state->singul;
  }

  ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_), s(dubins_curve_);
  from[0] = cur_state->state[0];
  from[1] = cur_state->state[1];
  from[2] = cur_state->yaw;
  to[0] = next_state[0];
  to[1] = next_state[1];
  to[2] = next_yaw;

  if (singul == -1) {
    from[2] = cur_state->yaw + M_PI;
    if (from[2] > M_PI) from[2] -= 2 * M_PI;
    to[2] = next_yaw + M_PI;
    if (to[2] > M_PI) to[2] -= 2 * M_PI;
  }

  Eigen::Vector3d xt;
  std::vector<double> reals;
  double dis = dubins_curve_->distance(from(), to());
  int check_num_car = ceil(dis / 0.01);

  Eigen::VectorXd delta_theta = next_state.tail(manipulator_dof_) - cur_state->state.tail(manipulator_dof_);
  double max_delta_theta = delta_theta.lpNorm<Eigen::Infinity>();
  int check_num_theta = ceil(max_delta_theta / 0.01);
  int piece_num_temp = std::max(check_num_car, check_num_theta);
  piece_num_temp = std::max(piece_num_temp, 10);

  for (int i = 0; i < piece_num_temp; ++i) {
    double temp_i = (double)i / (double)piece_num_temp;
    dubins_curve_->interpolate(from(), to(), temp_i, s());
    reals = s.reals();
    if (singul == -1) {
      reals[2] = reals[2] + M_PI;
      if (reals[2] > M_PI) reals[2] -= 2 * M_PI;
    }
    xt = Eigen::Vector3d(reals[0], reals[1], reals[2]);
    if (mm_config_->checkcollision(xt, cur_state->state.tail(manipulator_dof_) + delta_theta * temp_i, false)) {
      return true;
    }
  }
  return false;
}

bool RrtStarPlanning::checkCollision(PathNodeRRTStarPtr& cur_state, PathNodeRRTStarPtr& next_state) {
  if (cur_state->node_state == PathNodeRRTStar::RRT_COLLISION ||
      next_state->node_state == PathNodeRRTStar::RRT_COLLISION) {
    return true;
  }
  return checkCollision(cur_state, next_state->state, next_state->yaw);
}

void RrtStarPlanning::smoothPath(PathNodeRRTStarPtr end_node) {
  if (!have_path_ || end_node == nullptr) return;

  // 简化路径：尝试跳过中间节点
  PathNodeRRTStarPtr node = end_node;
  PathNodeRRTStarPtr temp;
  while (node != nullptr) {
    temp = node->parent;
    while (temp != nullptr) {
      if (!checkCollision(temp, node)) {
        linkNode(temp, node);
      }
      temp = temp->parent;
    }
    node = node->parent;
  }
}

bool RrtStarPlanning::getTraj(
    std::vector<Eigen::VectorXd>& traj,
    std::vector<double>& yaw_list,
    std::vector<double>& t_list) {
  traj.clear();
  yaw_list.clear();
  t_list.clear();

  if (!have_path_ || best_goal_node_ == nullptr) return false;

  // 平滑路径
  smoothPath(best_goal_node_);

  // 收集路径点
  PathNodeRRTStarPtr node = best_goal_node_;
  while (node != nullptr) {
    traj.push_back(node->state);
    yaw_list.push_back(node->yaw);
    if (node->parent != nullptr) {
      t_list.push_back(getTime(node->parent, node));
    }
    node = node->parent;
  }

  std::reverse(traj.begin(), traj.end());
  std::reverse(yaw_list.begin(), yaw_list.end());
  std::reverse(t_list.begin(), t_list.end());

  return true;
}

double RrtStarPlanning::getTime(PathNodeRRTStarPtr& pre_node, PathNodeRRTStarPtr& cur_node) {
  double t = -1.0;
  Eigen::VectorXd pre_state = pre_node->state;
  Eigen::VectorXd cur_state = cur_node->state;

  ompl::base::ScopedState<> from(dubins_curve_), to(dubins_curve_);
  from[0] = pre_node->state[0];
  from[1] = pre_node->state[1];
  from[2] = pre_node->yaw;
  to[0] = cur_node->state[0];
  to[1] = cur_node->state[1];
  to[2] = cur_node->yaw;

  if (cur_node->singul == -1) {
    from[2] = pre_node->yaw + M_PI;
    if (from[2] > M_PI) from[2] -= 2 * M_PI;
    to[2] = cur_node->yaw + M_PI;
    if (to[2] > M_PI) to[2] -= 2 * M_PI;
  }

  double len = dubins_curve_->distance(from(), to());
  t = std::max(len / max_vel_, t);

  for (int i = 0; i < manipulator_dof_; ++i) {
    t = std::max(calAngleErr(pre_state(mobile_base_dof_ + i), cur_state(mobile_base_dof_ + i)) / max_joint_vel_, t);
  }
  return t;
}

double RrtStarPlanning::getCost() {
  if (!have_path_) return -1.0;
  return c_best_;
}

bool RrtStarPlanning::getLayer(int& start_layer, int& end_layer) {
  if (!have_path_ || best_goal_node_ == nullptr) return false;
  end_layer = best_goal_node_->layer;
  PathNodeRRTStarPtr node = best_goal_node_;
  while (node != nullptr) {
    if (node->parent == nullptr) {
      start_layer = node->layer;
      break;
    }
    node = node->parent;
  }
  return true;
}

int RrtStarPlanning::getPathLen() {
  if (!have_path_ || best_goal_node_ == nullptr) return -1;
  PathNodeRRTStarPtr temp = best_goal_node_;
  int len = 0;
  while (temp != nullptr) {
    ++len;
    temp = temp->parent;
  }
  return len;
}

void RrtStarPlanning::init(
    const std::vector<Eigen::VectorXd>& start_pt_list,
    const std::vector<Eigen::VectorXd>& end_pt_list) {
  reset();

  int start_num = start_pt_list.size();
  int end_num = end_pt_list.size();

  // 初始化采样空间边界
  for (int i = 0; i < mobile_base_dof_; ++i) {
    max_size_[i] = -1e6;
    min_size_[i] = 1e6;
  }

  c_best_ = 1e6;

  // 计算采样中心和边界
  sample_start_ = start_pt_list[0];
  sample_goal_ = end_pt_list[0];
  c_min_ = (sample_start_ - sample_goal_).norm();

  for (int i = 0; i < start_num; ++i) {
    for (int j = 0; j < end_num; ++j) {
      for (int k = 0; k < mobile_base_dof_; ++k) {
        max_size_[k] = std::max(start_pt_list[i][k] + std::fabs(end_pt_list[j][k] - start_pt_list[i][k]) + 20.0, max_size_[k]);
        min_size_[k] = std::min(start_pt_list[i][k] - std::fabs(end_pt_list[j][k] - start_pt_list[i][k]) - 20.0, min_size_[k]);
      }
    }
  }
}

void RrtStarPlanning::reset() {
  have_path_ = false;
  tree_count_ = 0;
  start_node_ = nullptr;
  goal_node_ = nullptr;
  best_goal_node_ = nullptr;
  c_best_ = 1e6;

  for (auto it = node_pool_.begin(); it != node_pool_.end(); ++it) {
    delete it->second;
  }
  node_pool_.clear();
}

void RrtStarPlanning::setParam(ros::NodeHandle& nh, const std::shared_ptr<MMConfig>& mm_config) {
  mm_config_ = mm_config;
  max_vel_ = mm_config_->getBaseMaxVel();

  nh.param("search/check_num", check_num_, 10);
  nh.param("search/max_sample_time", max_sample_time_, 0.5);
  nh.param("search/max_loop_num", max_loop_num_, 5000);
  nh.param("search/time_resolution", time_resolution_, 1.0);
  nh.param("optimization/self_safe_margin", self_safe_margin_, 0.1);
  nh.param("optimization/safe_margin_mani", safe_margin_mani_, 0.1);

  // RRT* 特有参数（可以从参数服务器读取，这里给默认值）
  nh.param("search/goal_tolerance", goal_tolerance_, 0.5);
  nh.param("search/goal_bias", goal_bias_, 0.4);
  nh.param("search/rewire_radius", rewire_radius_, 0.45);

  nh.param("mm/mobile_base_dof", mobile_base_dof_, 2);

  double dist_resolution;
  nh.param("search/dist_resolution", dist_resolution, 1.0);
  time_resolution_ = dist_resolution / max_vel_;

  std::vector<double> joint_pos_limit;
  nh.getParam("mm/manipulator_min_pos", joint_pos_limit);
  min_joint_pos_.resize(joint_pos_limit.size());
  for (unsigned int i = 0; i < joint_pos_limit.size(); i++) {
    min_joint_pos_(i) = joint_pos_limit[i];
  }

  joint_pos_limit.clear();
  nh.getParam("mm/manipulator_max_pos", joint_pos_limit);
  max_joint_pos_.resize(joint_pos_limit.size());
  for (unsigned int i = 0; i < joint_pos_limit.size(); i++) {
    max_joint_pos_(i) = joint_pos_limit[i];
  }

  nh.param("mm/manipulator_max_vel", max_joint_vel_, 1.0);
  nh.param("mm/manipulator_max_acc", max_joint_acc_, 1.0);
  nh.param("mm/manipulator_dof", manipulator_dof_, 6);
  nh.param("mm/manipulator_thickness", mani_thickness_, 0.1);

  dubins_curve_ = std::make_shared<ompl::base::DubinsStateSpace>(0.1);
  traj_dim_ = mobile_base_dof_ + manipulator_dof_;

  // 初始化采样空间边界
  max_size_.clear();
  min_size_.clear();
  for (int i = 0; i < mobile_base_dof_; ++i) {
    max_size_.push_back(0.0);
    min_size_.push_back(0.0);
  }
  for (int i = 0; i < manipulator_dof_; ++i) {
    max_size_.push_back(max_joint_pos_[i]);
    min_size_.push_back(min_joint_pos_[i]);
  }
  // yaw
  max_size_.push_back(M_PI);
  min_size_.push_back(-M_PI);

  // 可视化初始化
  nh.param("search/rrt_star_vis_enable", enable_vis_, true);
  nh.param("search/rrt_star_vis_interval", vis_interval_, 10);
  rrt_tree_pub_ = nh.advertise<visualization_msgs::Marker>("rrt_star_tree", 1);
  rrt_path_pub_ = nh.advertise<visualization_msgs::Marker>("rrt_star_path", 1);
  rrt_sample_pub_ = nh.advertise<visualization_msgs::Marker>("rrt_star_samples", 1);
  rrt_nodes_pub_ = nh.advertise<visualization_msgs::MarkerArray>("rrt_star_nodes", 1);
}

// ===================== Visualization Functions =====================

void RrtStarPlanning::visualizeTree() {
  if (!enable_vis_ || rrt_tree_pub_.getNumSubscribers() == 0) return;

  visualization_msgs::Marker tree_marker;
  tree_marker.header.frame_id = "world";
  tree_marker.header.stamp = ros::Time::now();
  tree_marker.ns = "rrt_star_tree";
  tree_marker.id = 0;
  tree_marker.type = visualization_msgs::Marker::LINE_LIST;
  tree_marker.action = visualization_msgs::Marker::ADD;
  tree_marker.pose.orientation.w = 1.0;
  tree_marker.scale.x = 0.02;  // 线宽
  tree_marker.color.r = 0.0;
  tree_marker.color.g = 0.8;
  tree_marker.color.b = 0.2;
  tree_marker.color.a = 0.6;

  // 遍历所有节点，绘制边
  for (auto& pair : node_pool_) {
    PathNodeRRTStarPtr node = pair.second;
    if (node->parent != nullptr && node->node_state == PathNodeRRTStar::RRT_IN_TREE) {
      geometry_msgs::Point p1, p2;
      p1.x = node->parent->state(0);
      p1.y = node->parent->state(1);
      p1.z = 0.05;
      p2.x = node->state(0);
      p2.y = node->state(1);
      p2.z = 0.05;
      tree_marker.points.push_back(p1);
      tree_marker.points.push_back(p2);
    }
  }

  rrt_tree_pub_.publish(tree_marker);

  // 同时发布节点（球体）
  visualization_msgs::MarkerArray nodes_array;
  visualization_msgs::Marker node_marker;
  node_marker.header.frame_id = "world";
  node_marker.header.stamp = ros::Time::now();
  node_marker.ns = "rrt_star_nodes";
  node_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  node_marker.action = visualization_msgs::Marker::ADD;
  node_marker.id = 0;
  node_marker.pose.orientation.w = 1.0;
  node_marker.scale.x = 0.08;
  node_marker.scale.y = 0.08;
  node_marker.scale.z = 0.08;
  node_marker.color.r = 0.2;
  node_marker.color.g = 0.6;
  node_marker.color.b = 1.0;
  node_marker.color.a = 0.8;

  for (auto& pair : node_pool_) {
    PathNodeRRTStarPtr node = pair.second;
    if (node->node_state == PathNodeRRTStar::RRT_IN_TREE) {
      geometry_msgs::Point p;
      p.x = node->state(0);
      p.y = node->state(1);
      p.z = 0.05;
      node_marker.points.push_back(p);
    }
  }

  nodes_array.markers.push_back(node_marker);

  // 起点标记（绿色）
  visualization_msgs::Marker start_marker;
  start_marker.header.frame_id = "world";
  start_marker.header.stamp = ros::Time::now();
  start_marker.ns = "rrt_star_start";
  start_marker.id = 1;
  start_marker.type = visualization_msgs::Marker::SPHERE;
  start_marker.action = visualization_msgs::Marker::ADD;
  start_marker.pose.position.x = start_node_->state(0);
  start_marker.pose.position.y = start_node_->state(1);
  start_marker.pose.position.z = 0.1;
  start_marker.pose.orientation.w = 1.0;
  start_marker.scale.x = 0.2;
  start_marker.scale.y = 0.2;
  start_marker.scale.z = 0.2;
  start_marker.color.r = 0.0;
  start_marker.color.g = 1.0;
  start_marker.color.b = 0.0;
  start_marker.color.a = 1.0;
  nodes_array.markers.push_back(start_marker);

  // 目标点标记（红色）
  visualization_msgs::Marker goal_marker;
  goal_marker.header.frame_id = "world";
  goal_marker.header.stamp = ros::Time::now();
  goal_marker.ns = "rrt_star_goal";
  goal_marker.id = 2;
  goal_marker.type = visualization_msgs::Marker::SPHERE;
  goal_marker.action = visualization_msgs::Marker::ADD;
  goal_marker.pose.position.x = goal_node_->state(0);
  goal_marker.pose.position.y = goal_node_->state(1);
  goal_marker.pose.position.z = 0.1;
  goal_marker.pose.orientation.w = 1.0;
  goal_marker.scale.x = 0.2;
  goal_marker.scale.y = 0.2;
  goal_marker.scale.z = 0.2;
  goal_marker.color.r = 1.0;
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 0.0;
  goal_marker.color.a = 1.0;
  nodes_array.markers.push_back(goal_marker);

  rrt_nodes_pub_.publish(nodes_array);
}

void RrtStarPlanning::visualizePath() {
  if (!enable_vis_ || rrt_path_pub_.getNumSubscribers() == 0) return;
  if (!have_path_ || best_goal_node_ == nullptr) return;

  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = "world";
  path_marker.header.stamp = ros::Time::now();
  path_marker.ns = "rrt_star_path";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = 0.06;  // 线宽
  path_marker.color.r = 1.0;
  path_marker.color.g = 0.5;
  path_marker.color.b = 0.0;
  path_marker.color.a = 1.0;

  // 从目标回溯到起点
  PathNodeRRTStarPtr node = best_goal_node_;
  while (node != nullptr) {
    geometry_msgs::Point p;
    p.x = node->state(0);
    p.y = node->state(1);
    p.z = 0.1;
    path_marker.points.push_back(p);
    node = node->parent;
  }

  rrt_path_pub_.publish(path_marker);
}

void RrtStarPlanning::visualizeSampledPoint(const Eigen::VectorXd& point, bool is_valid) {
  if (!enable_vis_ || rrt_sample_pub_.getNumSubscribers() == 0) return;

  static int sample_id = 0;
  visualization_msgs::Marker sample_marker;
  sample_marker.header.frame_id = "world";
  sample_marker.header.stamp = ros::Time::now();
  sample_marker.ns = "rrt_star_sample";
  sample_marker.id = sample_id++ % 100;  // 循环使用ID，避免marker过多
  sample_marker.type = visualization_msgs::Marker::SPHERE;
  sample_marker.action = visualization_msgs::Marker::ADD;
  sample_marker.pose.position.x = point(0);
  sample_marker.pose.position.y = point(1);
  sample_marker.pose.position.z = 0.02;
  sample_marker.pose.orientation.w = 1.0;
  sample_marker.scale.x = 0.05;
  sample_marker.scale.y = 0.05;
  sample_marker.scale.z = 0.05;
  sample_marker.lifetime = ros::Duration(1.0);  // 1秒后消失

  if (is_valid) {
    sample_marker.color.r = 0.0;
    sample_marker.color.g = 1.0;
    sample_marker.color.b = 1.0;
  } else {
    sample_marker.color.r = 1.0;
    sample_marker.color.g = 0.0;
    sample_marker.color.b = 1.0;
  }
  sample_marker.color.a = 0.7;

  rrt_sample_pub_.publish(sample_marker);
}

void RrtStarPlanning::clearVisualization() {
  if (!enable_vis_) return;

  visualization_msgs::Marker clear_marker;
  clear_marker.header.frame_id = "world";
  clear_marker.header.stamp = ros::Time::now();
  clear_marker.action = visualization_msgs::Marker::DELETEALL;

  rrt_tree_pub_.publish(clear_marker);
  rrt_path_pub_.publish(clear_marker);
  rrt_sample_pub_.publish(clear_marker);

  visualization_msgs::MarkerArray clear_array;
  clear_marker.ns = "rrt_star_nodes";
  clear_array.markers.push_back(clear_marker);
  rrt_nodes_pub_.publish(clear_array);
}

}  // namespace remani_planner