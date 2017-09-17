// Copyright 2015 Jake Ware

// INCLUDES
// c system includes

// cpp system includes

// external includes

// project includes
#include "./dijkstra_node.h"

NodeD::NodeD(NodeD* _parent, const Eigen::Vector3i _ixyz,
             const double _vel_g_mag, FlowData* W, const double _mass,
             const double _alpha, const Eigen::Vector4f state_min,
             const Eigen::Vector4f state_max, const Eigen::MatrixXf d_state,
             App* app) {
  // get parent
  visited = false;
  parent = _parent;

  // get parameters
  mass = _mass;
  alpha = _alpha;

  // get values
  ixyz = _ixyz;
  pos = W->getXYZ(ixyz);
  vel_g_mag = _vel_g_mag;
  vel_f = W->getUVW(ixyz);
  //var_f = W->getUVWVar(ixyz);
  setState();
  getActions(W, state_min, state_max, d_state);

  // set velocities and costs
  setCost(parent, W, app);
  if (app->naive_flag) {
    setCostTrue(parent, W, app);
  }
}

NodeD::~NodeD() {
  // Nothing
}

int NodeD::checkState(const Eigen::Vector4f _state,
                      const Eigen::Vector4f state_min,
                      const Eigen::Vector4f state_max, FlowData* W) {
  Eigen::Vector3f _pos = _state.segment(0, 3);
  Eigen::Vector3i _ixyz = W->getInd(_pos);

  // check boundaries
  for (int i = 0; i < _state.rows(); i++) {
    if (_state(i) < state_min(i) || _state(i) > state_max(i)) {
      return 0;
    }
  }

  // check occ and obs
  if (W->getObs2D(_ixyz) == 1) {
    return 0;
  }

  return 1;
}

void NodeD::getActions(FlowData* W, const Eigen::Vector4f state_min,
                       const Eigen::Vector4f state_max,
                       const Eigen::MatrixXf d_state) {
  valid.setOnes(d_state.cols());

  // loop through d_states
  for (int i = 0; i < d_state.cols(); i++) {
    Eigen::Vector4f test_state = state + d_state.col(i);
    valid(i) = checkState(test_state, state_min, state_max, W);
  }

  return;
}

void NodeD::setState() {
  state(0) = pos(0);
  state(1) = pos(1);
  state(2) = pos(2);
  state(3) = vel_g_mag;

  return;
}

Eigen::MatrixXf NodeD::getVels(const NodeD* _parent, App* app) {
  Eigen::MatrixXf vels;
  vels.setZero(3, 2);

  if (_parent != NULL) {
    // calc ground speed
    vels.col(0) = ((pos - _parent->pos).normalized()) * vel_g_mag;
    // calc air speed
    if (!app->naive_flag) {
      vels.col(1) = vels.col(0) - vel_f;
    } else {
      vels.col(1) = vels.col(0);
    }
  } else {
    vels.col(1) = -vel_f;
  }

  return vels;
}

double NodeD::getVelAMagTrue(NodeD* _parent) {
  double _vel_a_mag_true = 0;
  Eigen::MatrixXf vels;
  vels.setZero(3, 2);

  if (_parent != NULL) {
    // calc ground speed
    vels.col(0) = ((pos - _parent->pos).normalized()) * vel_g_mag;
    // calc air speed
    vels.col(1) = vels.col(0) - vel_f;

  } else {
    vels.col(1) = -vel_f;
  }

  _vel_a_mag_true = vels.col(1).norm();

  return _vel_a_mag_true;
}

double NodeD::getStored(const double _vel_a_mag, App* app) {
//  double _energy_sto = -0.003044 * powf(_vel_a_mag, 4)
//      + 0.3193 * powf(_vel_a_mag, 3) - 0.9984 * powf(_vel_a_mag, 2)
//      - 2.956 * _vel_a_mag + 109.2;

  double _energy_sto = app->pow_prof(0) * powf(_vel_a_mag, 4)
      + app->pow_prof(1) * powf(_vel_a_mag, 3)
      + app->pow_prof(2) * powf(_vel_a_mag, 2) + app->pow_prof(3) * _vel_a_mag
      + app->pow_prof(4);

  return _energy_sto;
}

Eigen::Vector3f NodeD::getEnergy(const NodeD* _parent, FlowData* W, App* app) {
  Eigen::Vector3f _energy = Eigen::Vector3f::Zero();

  // is this the start node?
  if (_parent != NULL) {
    // calculate velocities for this step
    Eigen::MatrixXf vels = getVels(_parent, app);
    Eigen::Vector3f _vel_g = vels.col(0);
    Eigen::Vector3f _vel_a = vels.col(1);
    double _vel_a_mag = _vel_a.norm();
    double _vel_g_mag = _vel_g.norm();

    // potential energy
    double _energy_pot = 0;  // mass * 9.81 * (pos(2) - _parent->pos(2));

    // stored energy
    // horizontal flight
    double _energy_sto = getStored(_vel_a_mag, app);

    // convert from watts to joules
    double edge_dist = (pos - _parent->pos).norm();
    double edge_time = edge_dist / _vel_g_mag;

    _energy_sto *= edge_time;

    _energy(0) = _parent->energy;
    _energy(1) = _energy_pot;
    _energy(2) = _energy_sto;
  }

  return _energy;
}

Eigen::Vector3f NodeD::getEnergyTrue(const NodeD* _parent, FlowData* W,
                                     App* app) {
  Eigen::Vector3f _energy = Eigen::Vector3f::Zero();

  // is this the start node?
  if (_parent != NULL) {
    // calculate velocities for this step
    // calc ground speed
    Eigen::Vector3f _vel_g = ((pos - _parent->pos).normalized()) * vel_g_mag;
    // calc air speed
    Eigen::Vector3f _vel_a = _vel_g - vel_f;

    double _vel_a_mag = _vel_a.norm();
    double _vel_g_mag = _vel_g.norm();

    // potential energy
    double _energy_pot = mass * 9.81 * (pos(2) - _parent->pos(2));

    // stored energy
    // horizontal flight
    double _energy_sto = getStored(_vel_a_mag, app);

    // convert from watts to joules
    double edge_dist = (pos - _parent->pos).norm();
    double edge_time = edge_dist / _vel_g_mag;

    _energy_sto *= edge_time;

    _energy(0) = _parent->energy_true;
    _energy(1) = _energy_pot;
    _energy(2) = _energy_sto;
  }

  return _energy;
}

double NodeD::getDist(const NodeD* _parent) {
  double _dist = 0;

  if (_parent != NULL) {
    _dist = _parent->dist + (_parent->pos - pos).norm();
  }

  return _dist;
}

double NodeD::getCost(const NodeD* _parent, FlowData* W, App* app) {
  // fprintf(stderr, "getCost\n");
  // calc energy consumption
  Eigen::Vector3f _energy = getEnergy(_parent, W, app);

  // calc distance
  double _dist = getDist(_parent);

  double _cost = alpha * _energy.sum() + (1 - alpha) * _dist;

  return _cost;
}

void NodeD::setCost(const NodeD* _parent, FlowData* W, App* app) {
  // fprintf(stderr, "setCost\n");
  // calc velocities
  Eigen::MatrixXf vels = getVels(_parent, app);
  vel_g = vels.col(0);
  vel_a = vels.col(1);
  vel_a_mag = vel_a.norm();

  // calc energy consumption
  Eigen::Vector3f _energy = getEnergy(_parent, W, app);
  energy_pot = _energy(1);
  energy_sto = _energy(2);
  energy = _energy.sum();

  // calc distance
  dist = getDist(_parent);

  cost = alpha * energy + (1 - alpha) * dist;

  return;
}

void NodeD::setCostTrue(const NodeD* _parent, FlowData* W, App* app) {
  // fprintf(stderr, "setCost\n");
  // calc velocities
  Eigen::MatrixXf vels;
  vels.setZero(3, 2);

  if (_parent != NULL) {
    // calc ground speed
    vels.col(0) = ((pos - _parent->pos).normalized()) * vel_g_mag;
    // calc air speed
    vels.col(1) = vels.col(0) - vel_f;
  } else {
    vels.col(1) = -vel_f;
  }

  vel_g_true = vels.col(0);
  vel_a_true = vels.col(1);
  vel_a_mag_true = vel_a_true.norm();

  // calc energy consumption
  Eigen::Vector3f _energy = getEnergyTrue(_parent, W, app);
  energy_pot_true = _energy(1);
  energy_sto_true = _energy(2);
  energy_true = _energy.sum();

  // calc distance
  cost_true = alpha * energy_true + (1 - alpha) * dist;

  return;
}

void NodeD::print() {
  fprintf(stderr, "\nnode:\n");
  // cout << "n_ptr: " << this << "\n";
  // cout << "p_ptr: " << this->parent << "\n";
  fprintf(stderr, "ixyz: %i %i %i\n", ixyz(0), ixyz(1), ixyz(2));
  fprintf(stderr, "pos: %f %f %f\n", pos(0), pos(1), pos(2));
  fprintf(stderr, "vel_g_mag: %f\n", vel_g_mag);
  fprintf(stderr, "vel_a_mag: %f\n", vel_a_mag);
  fprintf(stderr, "vel_f: %f %f %f\n", vel_f(0), vel_f(1), vel_f(2));
  fprintf(stderr, "vel_g: %f %f %f\n", vel_g(0), vel_g(1), vel_g(2));
  fprintf(stderr, "vel_a: %f %f %f\n", vel_a(0), vel_a(1), vel_a(2));
  fprintf(stderr, "potential energy: %f\n", energy_pot);
  fprintf(stderr, "stored energy: %f\n", energy_sto);
  fprintf(stderr, "total energy: %f\n", energy);
  fprintf(stderr, "dist: %f\n", dist);
  fprintf(stderr, "cost: %f\n", cost);
  fprintf(stderr, "visited: %i\n", visited);
  fprintf(stderr, "\n");
}
