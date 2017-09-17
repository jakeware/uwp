#ifndef FLOW_PLAN_SRC_PLANNER_H_
#define FLOW_PLAN_SRC_PLANNER_H_

// Copyright 2015 Jake Ware

// c system includes
#include <stdlib.h>
#include <stdio.h>

// cpp system includes
#include <queue>
#include <list>
#include <vector>
#include <algorithm>
#include <iostream>
#include <string>
#include <limits>

// external includes
#include "eigen3/Eigen/Dense"
#include "bot_lcmgl_client/lcmgl.h"
#include "eigen_utils/eigen_utils.hpp"

// project includes
#include "flow_data/flow_data.h"
#include "flow_plan_app/flow_plan_app.h"
#include "node/node.h"
#include "utils/plot_utils.h"
#include "astar/astar.h"
#include "dijkstra/dijkstra.h"

class Planner {
 public:
  Planner(FlowData * W, App * app);
  ~Planner();

  // general utilities
  void freeMem(std::vector<std::vector<std::vector<Node*> > > * N, FlowData* W,
               App * app);
  void initMap(FlowData * W, App * app);
  int checkPlannerBounds(Eigen::Vector3i ixyz, FlowData * W, App* app);

  // node utilities
  void getDState(App * app);
  void getSteps(App * app);
  void initNodes(std::vector<std::vector<std::vector<Node*> > > * N,
                 FlowData* W, App * app);
  Eigen::Vector3i getNodeInd(const Eigen::Vector4f state, App * app);
  Node* createNode(Node * _parent, Eigen::Vector3i _ixyz, double _vel_g_mag,
                   FlowData * W, App * app);

  // naive planner utils
  void checkPath(std::list<Node*> * path,
                 std::vector<std::vector<std::vector<Node*> > > * N,
                 FlowData* W, App* app);
  int checkDynamics(Node * n, Node * parent, FlowData * W, App * app);

  // path utils
  void getPath(std::list<Node*> * path,
               std::vector<std::vector<std::vector<Node*> > > * N, FlowData * W,
               App * app);
  void printPath(std::list<Node*> * path, App * app);
  void sampPathSimple(std::list<Node*> * path, std::list<Node*> * samp_path,
                      App * app);
  void sampPathEnergy(std::list<Node*> * path, std::list<Node*> * samp_path,
                      FlowData* W, App * app);
  void collisionCheckPath(std::list<Node*> * path, std::list<Node*> * samp_path,
                          std::list<Node*> * samp_path_col, FlowData* W,
                          App * app);
  // QuadPath getQuadPath(std::list<Node*> * path, App * app);
  // QuadTrajectory * getQuadTraj(std::list<Node*> * path, App * app);
  void dumpPath(std::list<Node*> * path, std::string path_name, App * app);
  // void publishPath(std::list<Node*> * path, App * app);
  double getEnergyDelta(Eigen::Vector4f state1, Eigen::Vector4f state2,
                        float dt, bool debug, FlowData * W, App * app);
  bool collisionCheck(Eigen::Vector3f pos1, Eigen::Vector3f pos2, bool debug,
                      FlowData * W, App * app);
  Eigen::VectorXd getSegmentTimes(std::list<Node*> * path,
                                  std::list<Node*> * path_samp_col, FlowData* W,
                                  App * app);

  std::list<Node*> path_;
};

#endif
