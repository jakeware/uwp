// Copyright 2015 Jake Ware

// c system includes

// cpp system includes

// external includes

// project includes
#include "node/node.h"

Node::Node(Node* _parent, const Eigen::Vector3i _ixyz, const double _vel_g_mag,
           FlowData * W, App * app) {
  // get parent
  visited = false;
  parent = _parent;

  // get parameters
  mass = app->mass;
  alpha = app->alpha;

  // get values
  ixyz = _ixyz;
  pos = W->getXYZ(ixyz);
  vel_g_mag = _vel_g_mag;

  state(0) = pos(0);
  state(1) = pos(1);
  state(2) = pos(2);
  state(3) = vel_g_mag;

  getActions(W, app->state_min, app->state_max, app->d_state);

  // set velocities and costs
  setNode(parent, W, app);
  h = (app->goal_state.segment(0, 3) - pos).norm();
  f = g + h;
}

Node::Node(Node* _parent, const Eigen::Vector3i _ixyz, const double _vel_g_mag,
           FlowData * W) {
  // get parent
  visited = false;
  parent = _parent;

  // get parameters
  mass = 0;
  alpha = 0;

  // get values
  ixyz = _ixyz;
  pos = W->getXYZ(ixyz);
  vel_g_mag = _vel_g_mag;

  state(0) = pos(0);
  state(1) = pos(1);
  state(2) = pos(2);
  state(3) = vel_g_mag;

  vel_a_mag = 0;
  vel_a_mag_true = 0;

  energy_pot = 0;
  energy_pot_true = 0;
  energy_sto = 0;
  energy_sto_true = 0;
  energy = 0;
  energy_true = 0;

  dist = 0;

  g = 0;
  g_true = 0;
  h = 0;
  f = 0;
}

Node::~Node() {
  // Nothing
}

int Node::checkState(const Eigen::Vector4f _state,
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
  if (W->getObs3D(_ixyz) == 1) {
    return 0;
  }

  return 1;
}

void Node::getActions(FlowData* W, const Eigen::Vector4f state_min,
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

Eigen::MatrixXf Node::getVels(const Node* _parent, bool naive, FlowData * W) {
  Eigen::MatrixXf vels;
  vels.setZero(3, 3);

  if (_parent != NULL) {
    // calc ground speed
    vels.col(0) = ((pos - _parent->pos).normalized()) * vel_g_mag;

    // get wind speed
    vels.col(1) = (W->getUVW(ixyz) + W->getUVW(_parent->ixyz))/2.0;

    // calc air speed
    if (!naive) {
      vels.col(2) = vels.col(0) - vels.col(1);
    } else {
      vels.col(2) = vels.col(0);
    }
  } else {
    vels.col(2) = -vels.col(1);
  }

  return vels;
}

double Node::getVelAMagTrue(Node* _parent, FlowData * W) {
  // calculate velocities for this step
  Eigen::MatrixXf vels = getVels(_parent, false, W);
  Eigen::Vector3f _vel_a = vels.col(2);
  double _vel_a_mag = _vel_a.norm();

  return _vel_a_mag;
}

double Node::getStored(const double _vel_a_mag, App * app) {
  double _energy_sto = app->pow_prof(0) * powf(_vel_a_mag, 4)
      + app->pow_prof(1) * powf(_vel_a_mag, 3)
      + app->pow_prof(2) * powf(_vel_a_mag, 2) + app->pow_prof(3) * _vel_a_mag
      + app->pow_prof(4);

  return _energy_sto;
}

Eigen::Vector3f Node::getEnergy(const Node* _parent, bool naive, FlowData* W,
                                App* app) {
  Eigen::Vector3f _energy = Eigen::Vector3f::Zero();

  // is this the start node?
  if (_parent != NULL) {
    // calculate velocities for this step
    Eigen::MatrixXf vels = getVels(_parent, naive, W);
    Eigen::Vector3f _vel_g = vels.col(0);
    Eigen::Vector3f _vel_f = vels.col(1);
    Eigen::Vector3f _vel_a = vels.col(2);
    double _vel_a_mag = _vel_a.norm();
    double _vel_g_mag = _vel_g.norm();

    // potential energy
    double _energy_pot = mass * app->g * (pos(2) - _parent->pos(2));

    // stored energy
    // horizontal flight
    double _energy_sto = getStored(_vel_a_mag, app);

    // convert from watts to joules
    double edge_dist = (pos - _parent->pos).norm();
    double edge_time = edge_dist / _vel_g_mag;

    _energy_sto *= edge_time;

    if (!naive) {
      _energy(0) = _parent->energy_true;
    } else {
      _energy(0) = _parent->energy;
    }
    _energy(1) = _energy_pot;
    _energy(2) = _energy_sto;
  }

  return _energy;
}

double Node::getDist(const Node* _parent) {
  double _dist = 0;

  if (_parent != NULL) {
    _dist = _parent->dist + (_parent->pos - pos).norm();
  }

  return _dist;
}

double Node::getCost(const Node* _parent, FlowData* W, App* app) {
  // fprintf(stderr, "getCost\n");
  // calc energy consumption
  Eigen::Vector3f _energy = getEnergy(_parent, app->naive_flag, W, app);

  // calc distance
  double _dist = getDist(_parent);

  double _cost = alpha * _energy.sum() + (1 - alpha) * _dist;

  return _cost;
}

void Node::setNode(const Node* _parent, FlowData* W, App* app) {
  // set values that are planned over
  // calc velocities for initial planning
  Eigen::MatrixXf vels = getVels(_parent, app->naive_flag, W);
  vel_g = vels.col(0);
  vel_f = vels.col(1);
  vel_a = vels.col(2);
  vel_a_mag = vel_a.norm();

  // calc true underlying velocities
  Eigen::MatrixXf vels_true = getVels(_parent, false, W);
  vel_a_true = vels_true.col(2);
  vel_a_mag_true = vel_a_true.norm();

  // calc energy consumption for planning
  Eigen::Vector3f _energy = getEnergy(_parent, app->naive_flag, W, app);
  energy_pot = _energy(1);
  energy_sto = _energy(2);
  energy = _energy.sum();

  // calc true energy consumption
  Eigen::Vector3f _energy_true = getEnergy(_parent, false, W, app);
  energy_pot_true = _energy_true(1);
  energy_sto_true = _energy_true(2);
  energy_true = _energy_true.sum();

  // calc distance
  dist = getDist(_parent);

  // set costs
  g = energy;
  g_true = energy_true;
  h = (pos - app->goal_state.segment(0, 3)).norm();
  f = g + h;

  return;
}

void Node::print() {
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
  fprintf(stderr, "cost: %f\n", g);
  fprintf(stderr, "visited: %i\n", visited);
  fprintf(stderr, "\n");
}
