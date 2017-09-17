#ifndef RRTS_HPP_
#define RRTS_HPP_

// Copyright 2015 Jake Ware

// c cystem includes
#include <queue>
#include <list>
#include <vector>
#include <algorithm>
#include <string>

// cpp system includes

// external includes
#include <eigen3/Eigen/Dense>
#include <bot_lcmgl_client/lcmgl.h>
#include <eigen_utils/eigen_utils.hpp>
#include <flann/flann.hpp>
//#include <boost/multi_array.hpp>

// project includes
#include "flow_data/flow_data.h"
#include "flow_plan/flow_plan_app.h"

// NAMESPACES
using std::string;
using Eigen::Vector3i;
using Eigen::Vector3d;
using Eigen::Vector4d;

class NodeRRTS {
 public:
  NodeRRTS(NodeRRTS* _parent, Vector3i &_ixyz, double &_vel_g_mag, FlowData &W, double &_mass, double &_alpha, App &app);
  ~NodeRRTS();

  int treeID;  // index in kdtree
  NodeRRTS* parent;  // pointer to parent node

  // state variables
  Vector3i ixyz;  // grid index
  Vector3d pos;  // position [m]
  double vel_g_mag;  // speed along edge [m/s]
  Vector4d state;  // x,y,z,speed

  // cost variables
  Vector3d vel_f;  // u,v,w wind velocities [m/s]
  Vector3d vel_g;  // resulting ground speed [m/s]
  Vector3d vel_a;  // x,y,z air speed
  double vel_a_mag;  // magnitude of air speed [m/s]
  double energy;  // cost to come [J]
  double energy_pot;  // potential energy [J]
  double energy_sto;  // stored energy [J]
  double dist;  // edge distance [m]
  double cost;  // total cost

  // parameters
  double mass;
  double alpha;

  void setState();
  MatrixXd getVels(NodeRRTS* _parent);
  Vector3d getEnergy(NodeRRTS* _parent, FlowData &W, App &app);
  double getDist(NodeRRTS* _parent);
  double getCost(NodeRRTS* _parent, FlowData &W, App &app);
  void setCost(NodeRRTS* _parent, FlowData &W, App &app);
  void node2Flann(flann::Matrix<double> &nf);
  void print();

  // plotting
  void plotCostEdge(Vector3d &pos1, Vector3d &pos2, App &app);
  void plotCostPos(Vector3d pos, App &app);
};

class PlannerRRTS {
 public:
  PlannerRRTS(Vector3i &_sxyz, Vector3i &_gxyz, FlowData &W, App &app);
  ~PlannerRRTS();

  // start and goal
  Vector3i start_ixyz;
  Vector3i goal_ixyz;
  Vector3d start_pos;
  Vector3d goal_pos;
  Vector4d start_state;
  Vector4d goal_state;

  // bounds
  Vector3i ixyz_min;
  Vector3i ixyz_max;
  Vector3d pos_min;
  Vector3d pos_max;

  // constraints
  double vel_g_min;
  double vel_g_max;
  double vel_g_step;
  double vel_a_max;

  // parameters
  double mass;  // vehicle weight [kg]
  double alpha;  // cost weight parameter
  double gamma;  // kdtree search radius parameter for findNear
  double eta;  // kdtree max search radius parameter for findNear

  // storage
  vector<NodeRRTS*> N;  // store all created nodes
  list<NodeRRTS*> path1;  // store final solution path
  list<NodeRRTS*> path2;  // store final solution path

  // flann
  flann::SearchParams search_params;
  flann::SearchParams radius_search_params;

  // node utilities
  NodeRRTS* createNode(NodeRRTS* _parent, Vector3i _ixyz, double _vel_g_mag,
                      FlowData &W, App &app);
  Vector4d randState(FlowData &W, App &app);
  NodeRRTS* newNode(NodeRRTS* n_near, Vector4d &state_rand, FlowData &W, App &app);
  void addNode(NodeRRTS* n, flann::Index<flann::L2_3D<double> > &kdtree);

  // collision checking
  int checkPoint(Vector3i &pxyz, FlowData &W);
  int checkEdge(Vector3d &pos1, Vector3d &pos2, FlowData &W);

  // kdtree utilities
  NodeRRTS* findNearest(Vector4d &state_rand, flann::Index<flann::L2_3D<double> > &kdtree, App &app);
  vector<NodeRRTS*> findNear(NodeRRTS* n_new, flann::Index<flann::L2_3D<double> > &kdtree, App &app);
  void connectNode(NodeRRTS* n_new, vector<NodeRRTS*> &N_near, FlowData &W, App &app);
  void rewireTree(NodeRRTS* n_new, vector<NodeRRTS*> &N_near, FlowData &W, App &app);

  // path utilities
  int planPath(FlowData &W, App &app);
  void getPath1();
  void getPath2();
  void dumpPath(list<NodeRRTS*> &path, App &app);

  // plotting
  void plotSG(App &app);
  void plotNodePos(NodeRRTS* n, App &app);
  void plotRandPos(Vector4d &state, App &app);
  void plotNearestPos(NodeRRTS* n, App &app);
  void plotNearPos(vector<NodeRRTS*> N_near, App &app);
  void plotNewPos(NodeRRTS* n, App &app);
  void plotColEdge(Vector3d pos1, Vector3d pos2, App &app);
  void plotGraph(App &app);
  void plotPaths(App &app);
  void plotPathEdge(list<NodeRRTS*> &path, App &app);
  void plotPathNode(list<NodeRRTS*> &path, App &app);
  void plotNearArea(Vector3d pos, double rad, App &app);
  void plotConnPos(NodeRRTS* n, App &app, int update_flag);
  void plotRewPos(NodeRRTS* n, App &app, int update_flag);
  void clearBuffersEnd(App &app);
  void clearBuffersMid(App &app);
};

#endif
