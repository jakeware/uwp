#ifndef FLOW_PLAN_SRC_PLANNER_NODE_H_
#define FLOW_PLAN_SRC_PLANNER_NODE_H_

// Copyright 2015 Jake Ware

// c system includes

// cpp system includes
#include <eigen3/Eigen/Dense>
#include <eigen_utils/eigen_utils.hpp>

// external includes
#include <string>

// project includes
#include "flow_data/flow_data.h"
#include "flow_plan_app/flow_plan_app.h"

class Node {
 public:
  Node(Node* _parent, const Eigen::Vector3i _ixyz, double _vel_g_mag,
       FlowData* W, App* app);
  Node(Node* _parent, const Eigen::Vector3i _ixyz, double _vel_g_mag,
         FlowData* W);
  ~Node();

  bool visited;
  Node* parent;  // pointer to parent node

  // state variables
  Eigen::Vector3i ixyz;  // grid index
  Eigen::Vector3f pos;  // position [m]
  double vel_g_mag;  // speed along edge [m/s]
  Eigen::Vector4f state;  // x,y,z,speed

  // cost variables
  Eigen::Vector3f vel_f;  // u,v,w wind velocities [m/s]
  Eigen::Vector3f vel_g;  // resulting ground speed [m/s]
  Eigen::Vector3f vel_a;  // x,y,z air speed
  double vel_a_mag;  // magnitude of air speed [m/s]
  double energy;  // cost to come [J]
  double energy_pot;  // potential energy [J]
  double energy_sto;  // stored energy [J]
  double dist;  // edge distance [m]
  double g;  // total cost to come
  double h;  // heuristic (euclidean distance to goal)
  double f;  // total cost (g + h)

  // book keeping variables
  Eigen::Vector3f vel_a_true;  // x,y,z air speed
  double vel_a_mag_true;
  double energy_true;  // cost to come [J]
  double energy_pot_true;  // potential energy [J]
  double energy_sto_true;  // stored energy [J]
  double g_true;  // total cost (g + h)

  // parameters
  double mass;
  double alpha;

  // actions
  Eigen::VectorXi valid;  // flags  // dx, dy, dz, dspeed

  Eigen::MatrixXf getVels(const Node * _parent, bool naive, FlowData * W);
  double getVelAMagTrue(Node* _parent, FlowData * W);
  Eigen::Vector3f getEnergy(const Node * _parent, bool naive, FlowData * W,
                            App* app);
  double getStored(const double _vel_a_mag, App * app);
  double getDist(const Node * _parent);
  double getCost(const Node * _parent, FlowData* W, App * app);
  void setNode(const Node * _parent, FlowData* W, App * app);
  void getActions(FlowData * W, const Eigen::Vector4f state_min,
                  const Eigen::Vector4f state_max,
                  const Eigen::MatrixXf d_state);
  int checkState(const Eigen::Vector4f _state, const Eigen::Vector4f state_min,
                 const Eigen::Vector4f state_max, FlowData * W);
  void print();
};

#endif
