// Copyright 2015 Jake Ware

// c system includes

// cpp system includes

// external includes

// project includes
#include "utils/plot_utils.h"

void plotSG(Eigen::Vector3f start_pos, Eigen::Vector3f goal_pos) {
  lcm_t * lcm = bot_lcm_get_global(NULL);
  bot_lcmgl_t * lcmgl_sg = bot_lcmgl_init(lcm, "planner_sg");

  // start pos
  bot_lcmgl_color3f(lcmgl_sg, 0, 1, 0);
  double* start_pos_ptr = new double[3];
  start_pos_ptr[0] = 0.0;  // start_pos(0);
  start_pos_ptr[1] = 0.0;  // start_pos(1);
  start_pos_ptr[2] = start_pos(2);
  bot_lcmgl_disk(lcmgl_sg, start_pos_ptr, 2, 3);

  // goal pos
  bot_lcmgl_color3f(lcmgl_sg, 1, 0, 0);
  double* goal_pos_ptr = new double[3];
  goal_pos_ptr[0] = goal_pos(0) - start_pos(0);
  goal_pos_ptr[1] = goal_pos(1) - start_pos(1);
  goal_pos_ptr[2] = goal_pos(2);
  bot_lcmgl_disk(lcmgl_sg, goal_pos_ptr, 2, 3);

  bot_lcmgl_switch_buffer(lcmgl_sg);

  delete start_pos_ptr;
  delete goal_pos_ptr;
}

void plotSGGlobal(Eigen::Vector3f start_pos, Eigen::Vector3f goal_pos) {
  lcm_t * lcm = bot_lcm_get_global(NULL);
  bot_lcmgl_t * lcmgl_sg = bot_lcmgl_init(lcm, "planner_sg");

  // start pos
  bot_lcmgl_color3f(lcmgl_sg, 0, 1, 0);
  double* start_pos_ptr = new double[3];
  start_pos_ptr[0] = start_pos(0);
  start_pos_ptr[1] = start_pos(1);
  start_pos_ptr[2] = start_pos(2);
  bot_lcmgl_disk(lcmgl_sg, start_pos_ptr, 2, 3);

  // goal pos
  bot_lcmgl_color3f(lcmgl_sg, 1, 0, 0);
  double* goal_pos_ptr = new double[3];
  goal_pos_ptr[0] = goal_pos(0);
  goal_pos_ptr[1] = goal_pos(1);
  goal_pos_ptr[2] = goal_pos(2);
  bot_lcmgl_disk(lcmgl_sg, goal_pos_ptr, 2, 3);

  bot_lcmgl_switch_buffer(lcmgl_sg);

  delete start_pos_ptr;
  delete goal_pos_ptr;
}

void plotPos(Eigen::Vector3f pos, std::string type, App * app) {
  // offset by start_pos
  pos.segment(0, 2) -= app->start_pos.segment(0, 2);

  lcm_t * lcm = bot_lcm_get_global(NULL);
  std::string name = type + "_pos";
  bot_lcmgl_t * lcmgl_node_pos = bot_lcmgl_init(lcm, name.c_str());

  bot_lcmgl_point_size(lcmgl_node_pos, 5);
  bot_lcmgl_begin(lcmgl_node_pos, GL_POINTS);
  bot_lcmgl_color3f(lcmgl_node_pos, 0, 0, 0);
  bot_lcmgl_vertex3d(lcmgl_node_pos, static_cast<double>(pos(0)),
                     static_cast<double>(pos(1)), static_cast<double>(pos(2)));
  bot_lcmgl_end(lcmgl_node_pos);

  bot_lcmgl_switch_buffer(lcmgl_node_pos);
}

void plotEdge(Eigen::Vector3f pos1, Eigen::Vector3f pos2, std::string type,
              App * app) {
  lcm_t * lcm = bot_lcm_get_global(NULL);
  std::string name = type + "_edge";
  bot_lcmgl_t * lcmgl_edge_pos = bot_lcmgl_init(lcm, name.c_str());

  bot_lcmgl_line_width(lcmgl_edge_pos, 7);
  bot_lcmgl_begin(lcmgl_edge_pos, GL_LINES);

  bot_lcmgl_color3f(lcmgl_edge_pos, 1, 0, 0);
  bot_lcmgl_line_3d(lcmgl_edge_pos, static_cast<double>(pos1(0)),
                    static_cast<double>(pos1(1)), static_cast<double>(pos1(2)),
                    static_cast<double>(pos2(0)), static_cast<double>(pos2(1)),
                    static_cast<double>(pos2(2)));

  bot_lcmgl_end(lcmgl_edge_pos);

  bot_lcmgl_switch_buffer(lcmgl_edge_pos);

  return;
}

void plotBounds(Eigen::Vector3f pos_min, Eigen::Vector3f pos_max, App * app) {
  // offset by start_pos
  pos_min.segment(0, 2) -= app->start_pos.segment(0, 2);
  pos_max.segment(0, 2) -= app->start_pos.segment(0, 2);

  lcm_t * lcm = bot_lcm_get_global(NULL);
  bot_lcmgl_t * lcmgl_bounds = bot_lcmgl_init(lcm, "planner_bounds");

  bot_lcmgl_line_width(lcmgl_bounds, 7);
  bot_lcmgl_color3f(lcmgl_bounds, 0, 0, 0);
  bot_lcmgl_line_3d(lcmgl_bounds, static_cast<double>(pos_min(0)),
                    static_cast<double>(pos_min(1)),
                    static_cast<double>(pos_min(2)),
                    static_cast<double>(pos_max(0)),
                    static_cast<double>(pos_min(1)),
                    static_cast<double>(pos_min(2)));

  bot_lcmgl_line_3d(lcmgl_bounds, static_cast<double>(pos_min(0)),
                    static_cast<double>(pos_min(1)),
                    static_cast<double>(pos_min(2)),
                    static_cast<double>(pos_min(0)),
                    static_cast<double>(pos_max(1)),
                    static_cast<double>(pos_min(2)));

  bot_lcmgl_line_3d(lcmgl_bounds, static_cast<double>(pos_max(0)),
                    static_cast<double>(pos_max(1)),
                    static_cast<double>(pos_min(2)),
                    static_cast<double>(pos_max(0)),
                    static_cast<double>(pos_min(1)),
                    static_cast<double>(pos_min(2)));

  bot_lcmgl_line_3d(lcmgl_bounds, static_cast<double>(pos_max(0)),
                    static_cast<double>(pos_max(1)),
                    static_cast<double>(pos_min(2)),
                    static_cast<double>(pos_min(0)),
                    static_cast<double>(pos_max(1)),
                    static_cast<double>(pos_min(2)));
  bot_lcmgl_switch_buffer(lcmgl_bounds);

  return;
}

void bot_lcmgl_line_3d(bot_lcmgl_t * lcmgl, double x_start, double y_start,
                       double z_start, double x_end, double y_end,
                       double z_end) {
  lcmglBegin(LCMGL_LINES);
  lcmglVertex3d(x_start, y_start, z_start);
  lcmglVertex3d(x_end, y_end, z_end);
  lcmglEnd();
}

void plotPathEdgeCost(std::list<Node*>* path, App * app) {
  lcm_t * lcm = bot_lcm_get_global(NULL);
  bot_lcmgl_t * lcmgl_path_edges = bot_lcmgl_init(lcm, "planner_path_edges");

  // calc cost/dist for each segment
  Eigen::VectorXd cost_per_dist;
  cost_per_dist.setZero(path->size() - 1);
  int count = 0;
  for (std::list<Node*>::iterator it = path->begin(); it != path->end(); it++) {
    if (it == path->begin())
      continue;

    cost_per_dist(distance(path->begin(), it) - 1) = ((*it)->g
        - (*it)->parent->g) / ((*it)->dist - (*it)->parent->dist);
  }

  // find max cost per dist
  double max_cpd = 0;
  for (int i = 0; i < cost_per_dist.rows(); i++) {
    if (cost_per_dist(i) > max_cpd) {
      max_cpd = cost_per_dist(i);
    }
  }

  bot_lcmgl_line_width(lcmgl_path_edges, 7);
  bot_lcmgl_begin(lcmgl_path_edges, GL_LINES);
  for (std::list<Node*>::iterator it = path->begin(); it != path->end(); it++) {
    if (it == path->begin())
      continue;

    // get translated positions
    Eigen::Vector3f pos = (*it)->pos - app->start_pos;
    Eigen::Vector3f parent_pos = (*it)->parent->pos - app->start_pos;

    float * color = bot_color_util_jet(
        static_cast<double>(cost_per_dist(distance(path->begin(), it) - 1)
            / max_cpd));
    bot_lcmgl_color3f(lcmgl_path_edges, color[0], color[1], color[2]);
    bot_lcmgl_line_3d(lcmgl_path_edges, static_cast<double>(parent_pos(0)),
                      static_cast<double>(parent_pos(1)),
                      static_cast<double>(parent_pos(2)),
                      static_cast<double>(pos(0)), static_cast<double>(pos(1)),
                      static_cast<double>(pos(2)));
  }

  bot_lcmgl_end(lcmgl_path_edges);

  return;
}

void plotPathEdgeVel(std::list<Node*> * path, std::string path_name,
                     App * app) {
  lcm_t * lcm = bot_lcm_get_global(NULL);
  std::string name = "planner_" + path_name + "_vel";
  bot_lcmgl_t * lcmgl_path_edges = bot_lcmgl_init(lcm, name.c_str());

  bot_lcmgl_line_width(lcmgl_path_edges, 7);
  for (std::list<Node*>::iterator it = path->begin(); it != path->end(); it++) {
    if (it == path->begin())
      continue;

    std::list<Node*>::iterator it_last = it;
    it_last--;

    // get translated positions
    Eigen::Vector3f pos = (*it)->pos;
    pos.segment(0, 2) -= app->start_pos.segment(0, 2);
    Eigen::Vector3f last_pos = (*it_last)->pos;
    last_pos.segment(0, 2) -= app->start_pos.segment(0, 2);

    float * color = bot_color_util_jet((*it)->vel_g_mag / app->vel_g_max);
    bot_lcmgl_color3f(lcmgl_path_edges, color[0], color[1], color[2]);
    bot_lcmgl_line_3d(lcmgl_path_edges, static_cast<double>(last_pos(0)),
                      static_cast<double>(last_pos(1)),
                      static_cast<double>(last_pos(2)),
                      static_cast<double>(pos(0)), static_cast<double>(pos(1)),
                      static_cast<double>(pos(2)));
  }

  bot_lcmgl_switch_buffer(lcmgl_path_edges);

  return;
}

void plotPathNodePos(std::list<Node*> * path, std::string path_name,
                     App * app) {
  lcm_t * lcm = bot_lcm_get_global(NULL);
  std::string name = "planner_" + path_name + "_pos";
  bot_lcmgl_t * lcmgl_path_nodes = bot_lcmgl_init(lcm, name.c_str());

  bot_lcmgl_point_size(lcmgl_path_nodes, 5);
  bot_lcmgl_begin(lcmgl_path_nodes, GL_POINTS);
  bot_lcmgl_color3f(lcmgl_path_nodes, 0, 0, 0);
  for (std::list<Node*>::iterator it = path->begin(); it != path->end(); it++) {
    // get translated positions
    Eigen::Vector3f pos = (*it)->pos;
    pos.segment(0, 2) -= app->start_pos.segment(0, 2);

    bot_lcmgl_vertex3d(lcmgl_path_nodes, static_cast<double>(pos(0)),
                       static_cast<double>(pos(1)),
                       static_cast<double>(pos(2)));
  }
  bot_lcmgl_end(lcmgl_path_nodes);

  bot_lcmgl_switch_buffer(lcmgl_path_nodes);

  return;
}

void plotPoint(Eigen::Vector3f point, std::string type) {
  lcm_t * lcm = bot_lcm_get_global(NULL);
  std::string name = type + "_point";
  bot_lcmgl_t * lcmgl_point = bot_lcmgl_init(lcm, name.c_str());

  bot_lcmgl_point_size(lcmgl_point, 5);
  bot_lcmgl_begin(lcmgl_point, GL_POINTS);
  bot_lcmgl_color3f(lcmgl_point, 0, 0, 0);
  bot_lcmgl_vertex3d(lcmgl_point, static_cast<double>(point(0)),
                     static_cast<double>(point(1)),
                     static_cast<double>(point(2)));
  bot_lcmgl_end(lcmgl_point);

  bot_lcmgl_switch_buffer(lcmgl_point);
}
