#ifndef FLOW_PLAN_SRC_DIJKSTRA_DIJKSTRA_H_
#define FLOW_PLAN_SRC_DIJKSTRA_DIJKSTRA_H_

// Copyright 2015 Jake Ware

// c system includes

// cpp system includes
#include <queue>
#include <list>
#include <vector>
#include <algorithm>
#include <limits>
#include <iostream>
#include <string>

// external includes
#include "eigen3/Eigen/Dense"
#include "eigen_utils/eigen_utils.hpp"

// project includes
#include "node/node.h"
#include "flow_data/flow_data.h"
#include "flow_plan_app/flow_plan_app.h"

class SearchD {
 public:
  explicit SearchD(std::vector<std::vector<std::vector<Node*> > > * _N);
  ~SearchD();

  // storage
  struct nodeCompare {
    bool operator()(const Node* n1, const Node* n2) const {
      return (n1->g > n2->g);
    }
  };

  // node queue
  std::priority_queue<Node*, std::vector<Node*>, nodeCompare> Q;

  // node storage array
  std::vector<std::vector<std::vector<Node*> > > * N;

  // utilities
  void initStart(FlowData* W, App * app);
  int checkPlannerBounds(Eigen::Vector3i ixyz, FlowData * W, App * app);
  Eigen::Vector3i getNodeInd(const Eigen::Vector4f state, App * app);
  std::vector<Node*> getNeighbors(Node* n, FlowData* W, App* app);

  // search
  int getGraph(FlowData * W, App * app);
};

#endif  // FLOW_PLAN_SRC_DIJKSTRA_DIJKSTRA_H_
