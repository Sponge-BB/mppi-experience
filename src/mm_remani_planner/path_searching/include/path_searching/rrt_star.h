#ifndef _MM_CAR_FULL_RRT_STAR_PLANNING_
#define _MM_CAR_FULL_RRT_STAR_PLANNING_

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <time.h>
#include <random>
#include <cstdint>
#include "plan_env/grid_map.h"
#include "mm_config/mm_config.hpp"
#include <fstream>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/ScopedState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace remani_planner {

class PathNodeRRTStar {
public:
  enum RRT_NODE_STATE {
    RRT_NOT_EXPAND,
    RRT_IN_TREE,
    RRT_COLLISION
  };

  int singul;
  int layer;
  std::string index;
  Eigen::VectorXd state;
  double yaw;
  PathNodeRRTStar* parent;
  double g_score;
  RRT_NODE_STATE node_state;
  std::map<std::string, PathNodeRRTStar*> children;

  PathNodeRRTStar();
  PathNodeRRTStar(int dof);
  ~PathNodeRRTStar();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef PathNodeRRTStar* PathNodeRRTStarPtr;

class RrtStarPlanning {
public:
  std::shared_ptr<MMConfig> mm_config_;

  RrtStarPlanning();
  ~RrtStarPlanning();

  int RRTStarSearchAndGetSimplePath(
    const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<double>& start_yaw_list,
    const std::vector<Eigen::VectorXd>& end_pt_list, const std::vector<double>& end_yaw_list,
    const std::vector<double>& start_g_score_list, const std::vector<int>& start_layer_list,
    const std::vector<double>& end_g_score_list, const std::vector<int>& end_layer_list,
    const std::vector<int>& start_singul_list, const std::vector<int>& end_singul_list,
    std::vector<Eigen::VectorXd>& path, std::vector<double>& yaw_list, std::vector<double>& t_list);

  bool search(
    const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<double>& start_yaw_list,
    const std::vector<Eigen::VectorXd>& end_pt_list, const std::vector<double>& end_yaw_list,
    const std::vector<double>& start_g_score_list, const std::vector<int>& start_layer_list,
    const std::vector<double>& end_g_score_list, const std::vector<int>& end_layer_list,
    const std::vector<int>& start_singul_list, const std::vector<int>& end_singul_list);

  void setParam(ros::NodeHandle& nh, const std::shared_ptr<MMConfig>& mm_config);
  void init(const std::vector<Eigen::VectorXd>& start_pt_list, const std::vector<Eigen::VectorXd>& end_pt_list);
  void reset();
  bool getTraj(std::vector<Eigen::VectorXd>& traj, std::vector<double>& yaw_list, std::vector<double>& t_list);
  void smoothPath(PathNodeRRTStarPtr end_node);
  int getPathLen();
  int getTreeNum();
  double getCost();
  bool getLayer(int& start_layer, int& end_layer);

  // Visualization methods
  void enableVisualization(bool enable) { enable_vis_ = enable; }
  void visualizeTree();
  void visualizePath();
  void visualizeSampledPoint(const Eigen::VectorXd& point, bool is_valid);
  void clearVisualization();

  typedef std::shared_ptr<RrtStarPlanning> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  PathNodeRRTStarPtr start_node_;
  PathNodeRRTStarPtr goal_node_;
  PathNodeRRTStarPtr best_goal_node_;
  std::map<std::string, PathNodeRRTStarPtr> node_pool_;

  bool have_path_;
  double best_cost_;

  Eigen::VectorXd min_joint_pos_, max_joint_pos_;
  double max_joint_vel_, max_joint_acc_;
  double max_vel_, max_acc_;
  double self_safe_margin_, safe_margin_mani_;
  int mobile_base_dof_, manipulator_dof_, traj_dim_;
  double mani_thickness_;

  double goal_tolerance_;
  double goal_bias_;
  double max_sample_time_;
  double rewire_radius_;
  int check_num_;
  int max_loop_num_;

  std::vector<double> max_size_, min_size_;
  Eigen::VectorXd sample_start_, sample_goal_;
  double c_best_;
  double c_min_;

  std::mt19937 random_gen_;
  std::uniform_real_distribution<double> random_dis_;
  std::normal_distribution<double> norm_dis_;

  ompl::base::StateSpacePtr dubins_curve_;
  double time_resolution_;

  int tree_count_;

  // Visualization members
  bool enable_vis_;
  int vis_interval_;  // visualize every N iterations
  ros::Publisher rrt_tree_pub_;
  ros::Publisher rrt_path_pub_;
  ros::Publisher rrt_sample_pub_;
  ros::Publisher rrt_nodes_pub_;

  bool checkCollision(PathNodeRRTStarPtr& cur_state, PathNodeRRTStarPtr& next_state);
  bool checkCollision(PathNodeRRTStarPtr& cur_state, const Eigen::VectorXd& next_state, const double next_yaw);
  bool checkCollision(PathNodeRRTStarPtr& cur_state);

  void propagateGscore(PathNodeRRTStarPtr q);
  double calAngleErr(double angle1, double angle2);
  double estimateHeuristic(PathNodeRRTStarPtr& x1, PathNodeRRTStarPtr& x2);
  double estimateHeuristic(const Eigen::VectorXd& s1, const double& yaw1, const Eigen::VectorXd& s2, const double& yaw2);

  PathNodeRRTStarPtr createNode(const Eigen::VectorXd& s, const double yaw);
  void sample(Eigen::VectorXd& s_state, double& s_yaw);
  void sampleInformed(Eigen::VectorXd& s_state, double& s_yaw);
  void sampleGoalBiased(Eigen::VectorXd& s_state, double& s_yaw);  // 目标偏置采样
  PathNodeRRTStarPtr nearest(const Eigen::VectorXd& s, const double yaw);
  std::vector<PathNodeRRTStarPtr> nearNeighbors(const Eigen::VectorXd& s, const double yaw, double radius_time);
  PathNodeRRTStarPtr steer(PathNodeRRTStarPtr& q_near, const Eigen::VectorXd& s, const double s_yaw, double step_time);
  void rewire(PathNodeRRTStarPtr q_new, double near_time);
  void linkNode(PathNodeRRTStarPtr& parent, PathNodeRRTStarPtr& child);
  double getTime(PathNodeRRTStarPtr& pre_node, PathNodeRRTStarPtr& cur_node);
  std::string calculateIndex(PathNodeRRTStarPtr& q);
  std::string calculateIndex(const Eigen::VectorXd& s, const double yaw);
  bool isGoalReached(PathNodeRRTStarPtr& node);
  void updateBestPath(PathNodeRRTStarPtr& node);
};

}  // namespace remani_planner

#endif