#ifndef ASTAR_H_
#define ASTAR_H_

// Copyright 2015 Jake Ware

// c system includes

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
#include "eigen_utils/eigen_utils.hpp"

// project includes
#include "node/node.h"
#include "flow_data/flow_data.h"
#include "flow_plan_app/flow_plan_app.h"

class SearchAS {
 public:
  explicit SearchAS(std::vector<std::vector<std::vector<Node*> > > * _N);
  ~SearchAS();

  // storage
  struct nodeCompare {
    bool operator()(const Node* n1, const Node* n2) const {
      return (n1->f > n2->f);
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
  std::vector<Node*> getNeighbors(Node * u, FlowData * W, App * app);

  // search
  int getGraph(FlowData * W, App * app);
};

#endif
