#ifndef FLOW_PLAN_SRC_FLOW_PLAN_APP_H_
#define FLOW_PLAN_SRC_FLOW_PLAN_APP_H_

// Copyright 2015 Jake Ware

// c system includes
#include <assert.h>

// cpp system includes
#include <string>

// external includes
#include <eigen3/Eigen/Dense>
#include <lcm/lcm.h>
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <lcmgl_utils/lcmgl_utils.hpp>

// project includes

// CLASS
class App {
 public:
  App();
  ~App();

  lcm_t * lcm;
  BotParam * param;
  lcmgl_utils::LcmglStore lstore;

  // setup
  bool quiet;
  bool verbose;
  bool debug;
  bool plot;
  bool publish;
  bool dump;
  bool rand;
  bool naive_flag;
  bool vel_a_cap_flag;
  bool filename_flag;
  bool test_sg;

  // config
  std::string filename;
  double alpha;
  double mass;
  double g;
  int rand_count;
  int path_num;
  double min_dist;

  int z_index;

  // plotting
  double flow_scale;
  int flow_increment;
  double drag_scale;
  double sg_scale;

  // start and goal
  Eigen::Vector4f start_state;
  Eigen::Vector3f start_pos;
  Eigen::Vector3i start_ixyz;

  Eigen::Vector4f goal_state;
  Eigen::Vector3f goal_pos;
  Eigen::Vector3i goal_ixyz;

  // bounds
  Eigen::Vector3i ixyz_min;  // planner bounds (not map bounds)
  Eigen::Vector3i ixyz_max;  // planner bounds (not map bounds)
  Eigen::Vector3f pos_min;
  Eigen::Vector3f pos_max;
  Eigen::Vector4f state_min;
  Eigen::Vector4f state_max;
  double vel_g_min;
  double vel_g_max;

  // discretizations
  int x_step;
  Eigen::VectorXi x_steps;
  int y_step;
  Eigen::VectorXi y_steps;
  int z_step;
  Eigen::VectorXi z_steps;
  double vel_g_step;
  Eigen::VectorXf vel_g_steps;

  Eigen::MatrixXf d_pos;
  Eigen::VectorXf d_speed;
  Eigen::MatrixXf d_state;

  // power profile
  Eigen::VectorXd pow_prof;

  // constraints
  double vel_a_max;

  // rand seed
  unsigned int seed;

  void getParams();
};

#endif  // FLOW_PLAN_SRC_FLOW_PLAN_APP_H_
