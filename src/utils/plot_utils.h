#ifndef FLOW_PLAN_SRC_PLANNER_PLOT_UTILS_H_
#define FLOW_PLAN_SRC_PLANNER_PLOT_UTILS_H_

// Copyright 2015 Jake Ware

// c system includes

// cpp system includes
#include <list>
#include <string>

// external includes

// project includes
#include "eigen3/Eigen/Dense"
#include "eigen_utils/eigen_utils.hpp"
#include "lcm/lcm.h"
#include "bot_lcmgl_client/lcmgl.h"
#include "bot_param/param_client.h"
#include "bot_param/param_util.h"
#include "lcmgl_utils/lcmgl_utils.hpp"
// #include "rrbt/drawing_defs.hpp"
#include "node/node.h"
#include "flow_plan_app/flow_plan_app.h"

void plotSG(Eigen::Vector3f start_pos, Eigen::Vector3f goal_pos);
void plotSGGlobal(Eigen::Vector3f start_pos, Eigen::Vector3f goal_pos);
void plotPos(Eigen::Vector3f pos, std::string type, App * app);
void plotBounds(Eigen::Vector3f pos_min, Eigen::Vector3f pos_max, App * app);
void bot_lcmgl_line_3d(bot_lcmgl_t * lcmgl, double x_start, double y_start,
                       double z_start, double x_end, double y_end,
                       double z_end);
void plotPathEdgeCost(std::list<Node*> * path, App * app);
void plotPathEdgeVel(std::list<Node*> * path, std::string path_name, App * app);
void plotPathNodePos(std::list<Node*> * path, std::string path_name, App * app);
void plotEdge(Eigen::Vector3f pos1, Eigen::Vector3f pos2, std::string type,
              App * app);
void plotPoint(Eigen::Vector3f point, std::string type);

#endif
