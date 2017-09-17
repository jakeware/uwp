#ifndef RRT_HPP_
#define RRT_HPP_

// Copyright 2015 Jake Ware

// INCLUDES
// c system includes
#include <string>
#include <queue>
#include <list>
#include <vector>
#include <algorithm>

// cpp system includes

// external library includes
#include <eigen3/Eigen/Dense>
#include <bot_lcmgl_client/lcmgl.h>
#include <eigen_utils/eigen_utils.hpp>
#include <flann/flann.hpp>
// #include <boost/multi_array.hpp>

// project includes
#include "flow_data/flow_data.h"
#include "flow_plan/flow_plan_app.h"

// NAMESPACES
using std::string;
using Eigen::Vector3i;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::MatrixXd;

class NodeRRT {
 public:
  NodeRRT(NodeRRT* _parent, Vector3i _ixyz, double _vel_g_mag, FlowData &W);
  ~NodeRRT();

  int treeID;
  NodeRRT* parent;
  Vector3i ixyz;
  Vector3d pos;  // state
  Vector3d vel_g;  // state
  Vector4d state;
  Vector3d vel_f;
  Vector3d vel_a;
  double vel_a_mag;
  double vel_g_mag;
  double vel_a_max;

  void getVels();
  void fillState();
  void node2Flann(flann::Matrix<double> &nf);
  void print();
};

class PlannerRRT {
 public:
  PlannerRRT(Vector3i _sxyz, Vector3i _gxyz, FlowData &W, App &app);
  ~PlannerRRT();

  Vector3i start_ixyz;
  Vector3i goal_ixyz;
  Vector3d start_pos;
  Vector3d goal_pos;
  Vector4d start_state;
  Vector4d goal_state;
  Vector3i ixyz_min;
  Vector3i ixyz_max;
  Vector3d pos_min;
  Vector3d pos_max;
  double vel_g_min;
  double vel_g_max;
  double vel_g_step;
  double vel_a_max;
  vector<NodeRRT*> V;  // store all created nodes
  list<NodeRRT*> path;  // store final solution path
  MatrixXd P;  // store visited locations

  NodeRRT* createNode(NodeRRT* _parent, Vector3i _ixyz, double _vel_g_mag,
                      FlowData &W);
  int checkEdge(Vector3d pos1, Vector3d pos2, FlowData &W);
  NodeRRT* randNode(FlowData &W);
  int findNearest(NodeRRT* n_rand, flann::Index<flann::L2<double> > &kdtree, FlowData &W, App &app);
  NodeRRT* newNode(NodeRRT* n_near, NodeRRT* n_rand, FlowData &W);
  void addNode(NodeRRT* n, flann::Index<flann::L2<double> > &kdtree);
  int checkFree(Vector3i pxyz, FlowData &W);
  int checkRand(Vector3i pxyz, FlowData &W);
  Vector3i pos2Index(Vector3d pos);
  int planPath(FlowData &W, App &app);
  void getPath(NodeRRT* n);

  // plotting
  void plotSG(App &app);
  void plotNodePos(NodeRRT* n, App &app);
  void plotRandPos(NodeRRT* n, App &app);
  void plotNearPos(NodeRRT* n, App &app);
  void plotNewPos(NodeRRT* n, App &app);
  void plotEdge(Vector3d pos1, Vector3d pos2, App &app);
  void plotGraph(App &app);
  void plotPath(App &app);
  void plotPathEdge(App &app);
  void plotPathNode(App &app);
};

#endif
