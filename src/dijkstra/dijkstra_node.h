#ifndef FLOW_PLAN_SRC_DIJKSTRA_DIJKSTRA_NODE_H_
#define FLOW_PLAN_SRC_DIJKSTRA_DIJKSTRA_NODE_H_

// Copyright 2015 Jake Ware

// INCLUDES
// c system includes

// cpp system includes
#include <eigen3/Eigen/Dense>
#include <eigen_utils/eigen_utils.hpp>

// external includes
#include <string>

// project includes
#include "flow_plan/flow_data.h"
#include "flow_plan/flow_plan_app.h"

class NodeD {
 public:
  NodeD(NodeD* _parent, const Eigen::Vector3i _ixyz, double _vel_g_mag,
        FlowData* W, double _mass, double _alpha,
        const Eigen::Vector4f state_min, const Eigen::Vector4f state_max,
        const Eigen::MatrixXf d_state, App* app);
  ~NodeD();

  bool visited;
  NodeD* parent;  // pointer to parent node

  // state variables
  Eigen::Vector3i ixyz;  // grid index
  Eigen::Vector3f pos;  // position [m]
  double vel_g_mag;  // speed along edge [m/s]
  Eigen::Vector4f state;  // x,y,z,speed

  // cost variables
  Eigen::Vector3f vel_f;  // u,v,w wind velocities [m/s]
  Eigen::Vector3f var_f;  // u,v,w wind variance
  Eigen::Vector3f vel_g;  // resulting ground speed [m/s]
  Eigen::Vector3f vel_a;  // x,y,z air speed
  double vel_a_mag;  // magnitude of air speed [m/s]
  double energy;  // cost to come [J]
  double energy_pot;  // potential energy [J]
  double energy_sto;  // stored energy [J]
  double dist;  // edge distance [m]
  double cost;  // total cost
  double delta;  // distance between node and parent

  // book keeping variables
  Eigen::Vector3f vel_g_true;  // resulting ground speed [m/s]
  Eigen::Vector3f vel_a_true;  // x,y,z air speed
  double vel_a_mag_true;
  double energy_true;  // cost to come [J]
  double energy_pot_true;  // potential energy [J]
  double energy_sto_true;  // stored energy [J]
  double cost_true;  // total cost

  // parameters
  double mass;
  double alpha;

  // actions
  Eigen::VectorXi valid;  // flags  // dx, dy, dz, dspeed

  void setState();
  Eigen::MatrixXf getVels(const NodeD* _parent, App* app);
  double getVelAMagTrue(NodeD* _parent);
  Eigen::Vector3f getEnergy(const NodeD* _parent, FlowData* W, App* app);
  Eigen::Vector3f getEnergyTrue(const NodeD* _parent, FlowData* W,
                                App* app);
  double getStored(const double _vel_a_mag, App* app);
  double getDist(const NodeD* _parent);
  double getCost(const NodeD* _parent, FlowData* W, App* app);
  void setCost(const NodeD* _parent, FlowData* W, App* app);
  void setCostTrue(const NodeD* _parent, FlowData* W, App* app);
  void getActions(FlowData* W, const Eigen::Vector4f state_min,
                  const Eigen::Vector4f state_max,
                  const Eigen::MatrixXf d_state);
  int checkState(const Eigen::Vector4f _state, const Eigen::Vector4f state_min,
                 const Eigen::Vector4f state_max, FlowData* W);
  void print();
};

#endif // FLOW_PLAN_SRC_DIJKSTRA_DIJKSTRA_NODE_H_
