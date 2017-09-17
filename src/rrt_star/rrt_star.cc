// Copyright 2015 Jake Ware

// INCLUDES
// c system includes

// cpp system includes

// external includes

// project includes
#include "rrt_star.h"

NodeRRTS::NodeRRTS(NodeRRTS* _parent, Vector3i &_ixyz, double &_vel_g_mag,
                   FlowData &W, double &_mass, double &_alpha, App &app) {
  // get parent
  parent = _parent;

  // get parameters
  mass = _mass;
  alpha = _alpha;

  // get values
  ixyz = _ixyz;
  pos = W.getXYZ(ixyz);
  vel_g_mag = _vel_g_mag;
  vel_f = W.getUVW(ixyz);
  setState();

  // set velocities and costs
  setCost(parent, W, app);
}

NodeRRTS::~NodeRRTS() {
  // Nothing
}

void NodeRRTS::setState() {
  state(0) = pos(0);
  state(1) = pos(1);
  state(2) = pos(2);
  state(3) = vel_g_mag;

  return;
}

MatrixXd NodeRRTS::getVels(NodeRRTS* _parent) {
  MatrixXd vels;
  vels.setZero(3, 2);

  if (_parent != NULL) {
    // calc ground speed
    vels.col(0) = ((pos - _parent->pos).normalized()) * vel_g_mag;
    // calc air speed
    vels.col(1) = vels.col(0) - vel_f;
  } else {
    vels.col(1) = -vel_f;
  }

  return vels;
}

void NodeRRTS::plotCostEdge(Vector3d &pos1, Vector3d &pos2, App &app) {
  bot_lcmgl_line_width(app.lcmgl_cost_edge, 2);
  bot_lcmgl_begin(app.lcmgl_cost_edge, GL_LINES);
  bot_lcmgl_color3f(app.lcmgl_cost_edge, 0, 0.8, 0.2);
  bot_lcmgl_line3d(app.lcmgl_cost_edge, double(pos1(0)), double(pos1(1)),
                   double(pos1(2)), double(pos2(0)), double(pos2(1)),
                   double(pos2(2)));
  bot_lcmgl_end(app.lcmgl_cost_edge);
  bot_lcmgl_switch_buffer(app.lcmgl_cost_edge);
}

void NodeRRTS::plotCostPos(Vector3d pos, App &app) {
  bot_lcmgl_point_size(app.lcmgl_cost_pos, 5);
  bot_lcmgl_begin(app.lcmgl_cost_pos, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_cost_pos, 0, 0, 1);
  bot_lcmgl_vertex3d(app.lcmgl_cost_pos, double(pos(0)), double(pos(1)),
                     double(pos(2)));
  bot_lcmgl_end(app.lcmgl_cost_pos);
}

Vector3d NodeRRTS::getEnergy(NodeRRTS* _parent, FlowData &W, App &app) {
  if (app.debug) {
    fprintf(stderr, "getEnergy\n");
  }
  Vector3d _energy = Vector3d::Zero();

  // is this the start node?
  if (_parent != NULL) {
    if (app.debug) {
      plotCostEdge(_parent->pos, pos, app);
    }
    // calculate number of steps
    // get line between the two
    Vector3d edge_dir = (pos - _parent->pos).normalized();
    double edge_dist = (pos - _parent->pos).norm();
    int edge_tot_steps = floor(edge_dist);
    Vector3d edge_step = edge_dist * edge_dir / double(edge_tot_steps);

    // ground velocity
    Vector3d _vel_g = edge_dir * vel_g_mag;

    // declarations
    Vector3d _vel_a = Vector3d::Zero();
    Vector3d _vel_f = Vector3d::Zero();
    Vector3i _ixyz = Vector3i::Zero();
    Vector3d _pos = _parent->pos;  // will get assigned to _pos_last on first iteration
    Vector3d _pos_last = Vector3d::Zero();
    double _vel_a_mag = 0;
    double _energy_pot = 0;
    double _energy_sto = 0;
    double _energy_pot_tot = 0;
    double _energy_sto_tot = 0;

//    eigen_dump(pos);
//    eigen_dump(_parent->pos);
//    eigen_dump(edge_dir);
//    fprintf(stderr, "edge_dist: %f\n", edge_dist);
//    fprintf(stderr, "edge_tot_steps: %i\n", edge_tot_steps);
//    eigen_dump(edge_step);
//    eigen_dump(_vel_g);

    // step through edge start 1 step away
    for (int i = 1; i <= edge_tot_steps; i++) {
      // get location for step
      _pos_last = _pos;
      _pos = _parent->pos + double(i) * edge_step;
      _ixyz = W.getInd(_pos);

      if (app.debug) {
        plotCostPos(W.getXYZ(_ixyz), app);
      }

      // calculate velocities for this step
      // calc air speed
      _vel_f = W.getUVW(_ixyz);
      _vel_a = _vel_g - _vel_f;
      _vel_a_mag = _vel_a.norm();

      // potential energy
      _energy_pot = mass * 9.81 * (_pos(2) - _pos_last(2));

      // stored energy
      // horizontal flight
      //fprintf(stderr, "vel_a_h: %f  vel_a_v: %f\n" , vel_a.segment(0, 2).norm(), fabs(double(vel_a(2))));
      if (_vel_a.segment(0, 2).norm() >= fabs(double(_vel_a(2)))) {
        _energy_sto = 0.008171 * powf(_vel_a_mag, 4)
            + 0.1757 * powf(_vel_a_mag, 3) - 0.846 * powf(_vel_a_mag, 2)
            - 0.6665 * _vel_a_mag + 105.8;
      }
      // vertical flight
      else if (_vel_a.segment(0, 2).norm() < fabs(double(_vel_a(2)))) {
        double Vh = 5.2942;  // induced velocity at hover [m/s]
        double T = 0.1378 * powf(_vel_a_mag, 2) + mass * 9.81;
        // normal
        if (_vel_a_mag / Vh >= 0) {
          double Vi = -_vel_a_mag / 2
              + powf(powf(_vel_a_mag / 2, 2) + powf(Vh, 2), 0.5);
          _energy_sto = T * (_vel_a_mag + Vi);
        }
        // descent
        else if (_vel_a_mag / Vh < 0) {
          double Vi = -_vel_a_mag / 2
              - powf(powf(_vel_a_mag / 2, 2) - powf(Vh, 2), 0.5);
          _energy_sto = T * (_vel_a_mag + Vi);
        }
        // otherwise
        else {
          fprintf(
              stderr,
              "Error: Unhandled vertical flight state in Node::setEnergy\n");
        }
      }
      // otherwise
      else {
        fprintf(stderr, "Error: Unhandled flight state in Node::setEnergy\n");
      }

      // convert from watts to joules
      _energy_sto *= (_pos - _pos_last).norm() / vel_g_mag;

      // add to totals
      _energy_pot_tot += _energy_pot;
      _energy_sto_tot += _energy_sto;

//      eigen_dump(_ixyz);
//      eigen_dump(_pos_last);
//      eigen_dump(_pos);
//      eigen_dump(_vel_f);
//      eigen_dump(_vel_a);
//      fprintf(stderr, "_vel_a_mag: %f\n", _vel_a_mag);
//      fprintf(stderr, "_energy_pot: %f\n", _energy_pot);
//      fprintf(stderr, "_energy_sto: %f\n", _energy_sto);
//      fprintf(stderr, "_energy_pot_tot: %f\n", _energy_pot_tot);
//      fprintf(stderr, "_energy_sto_tot: %f\n", _energy_sto_tot);
    }

    _energy(0) = _parent->energy;
    _energy(1) = _energy_pot_tot;
    _energy(2) = _energy_sto_tot;

    if (app.debug) {
      bot_lcmgl_switch_buffer(app.lcmgl_cost_pos);
    }

    //eigen_dump(_energy);
  }

  return _energy;
}

double NodeRRTS::getDist(NodeRRTS* _parent) {
  double _dist = 0;

  if (_parent != NULL) {
    _dist = _parent->dist + (_parent->pos - pos).norm();
  }

  return _dist;
}

double NodeRRTS::getCost(NodeRRTS* _parent, FlowData &W, App &app) {
  //fprintf(stderr, "getCost\n");
  Vector3d _energy = Vector3d::Zero();
  double _dist = 0;
  double _cost = 0;

  // calc velocities
  MatrixXd vels = getVels(_parent);

  // calc energy consumption
  if (alpha > 0) {
    _energy = getEnergy(_parent, W, app);
  }

  // calc distance
  _dist = getDist(_parent);

  _cost = alpha * _energy.sum() + (1 - alpha) * _dist;

  return _cost;
}

void NodeRRTS::setCost(NodeRRTS* _parent, FlowData &W, App &app) {
  //fprintf(stderr, "setCost\n");
  Vector3d _energy = Vector3d::Zero();
  energy = 0;
  energy_pot = 0;
  energy_sto = 0;
  dist = 0;
  cost = 0;

  // calc velocities
  MatrixXd vels = getVels(_parent);
  vel_g = vels.col(0);
  vel_a = vels.col(1);
  vel_a_mag = vel_a.norm();

  // calc energy consumption
  if (alpha > 0) {
    _energy = getEnergy(_parent, W, app);
    energy_pot = _energy(1);
    energy_sto = _energy(2);
    energy = _energy.sum();
  }

  // calc distance
  dist = getDist(_parent);

  cost = alpha * energy + (1 - alpha) * dist;

  return;
}

void NodeRRTS::print() {
  fprintf(stderr, "\nnode:\n");
  //cout << "n_ptr: " << this << "\n";
  //cout << "p_ptr: " << this->parent << "\n";
  fprintf(stderr, "ixyz: %i %i %i\n", ixyz(0), ixyz(1), ixyz(2));
  fprintf(stderr, "pos: %f %f %f\n", pos(0), pos(1), pos(2));
  fprintf(stderr, "vel_g_mag: %f\n", vel_g_mag);
  fprintf(stderr, "vel_a_mag: %f\n", vel_a_mag);
  fprintf(stderr, "vel_f: %f %f %f\n", vel_f(0), vel_f(1), vel_f(2));
  fprintf(stderr, "vel_g: %f %f %f\n", vel_g(0), vel_g(1), vel_g(2));
  fprintf(stderr, "vel_a: %f %f %f\n", vel_a(0), vel_a(1), vel_a(2));
  fprintf(stderr, "\n");
}

void NodeRRTS::node2Flann(flann::Matrix<double> &nf) {
  nf[0][0] = this->pos(0);
  nf[0][1] = this->pos(1);
  nf[0][2] = this->pos(2);

  return;
}

PlannerRRTS::PlannerRRTS(Vector3i &_sxyz, Vector3i &_gxyz, FlowData &W,
                         App &app) {
  fprintf(stderr, "Starting planner...\n");

  // seed random number generator
  srand(time(NULL));

  // initialize start and goal states
  start_ixyz = _sxyz;
  start_pos = W.getXYZ(start_ixyz);
  start_state.segment(0, 3) = start_pos;
  start_state(3) = 0;
  goal_ixyz = _gxyz;
  goal_pos = W.getXYZ(goal_ixyz);
  goal_state.segment(0, 3) = goal_pos;
  goal_state(3) = 0;

  // bounds
  ixyz_min << 120, 150, 10;  //0, 0, 2;
  ixyz_max << 280, 280, 10;  //W.dim_x, W.dim_y, 15;
  pos_min = W.getXYZ(ixyz_min);
  pos_max = W.getXYZ(ixyz_max);

  // constraints
  vel_g_min = 0.5;  // minimum ground speed [m/s]
  vel_g_max = 10;  // maximum ground speed [m/s]
  vel_g_step = 0.5;  // speed discretization [m/s]
  vel_a_max = 15;  // maximum air speed [m/s]

  // parameters
  mass = 1.4;
  alpha = 0;  // 0 is pure distance, previous: 0.02
  gamma = 20;  // rad scale
  eta = 20;  // max rad

  fprintf(stderr, "Start: %f %f %f\n", start_pos(0), start_pos(1),
          start_pos(2));
  fprintf(stderr, "Goal: %f %f %f\n", goal_pos(0), goal_pos(1), goal_pos(2));

  // nearest search
  search_params.checks = flann::FLANN_CHECKS_UNLIMITED;  // check an unlimited number of leaves
  search_params.cores = 0;  // use 4 cores
  search_params.eps = 0;  // don't use eps-approximated neighbors
  search_params.sorted = true;  // sort results
  search_params.use_heap = flann::FLANN_False;  // don't use heap

  // near (radius) search
  radius_search_params.checks = flann::FLANN_CHECKS_UNLIMITED;  // check an unlimited number of leaves
  radius_search_params.cores = 0;  // use 4 cores
  radius_search_params.eps = 0;  // don't use eps-approximated neighbors
  radius_search_params.sorted = false;  // don't sort results
  radius_search_params.use_heap = flann::FLANN_False;  // don't use heap

  // plot start and goal nodes
  plotSG(app);

  // find path
  fprintf(stderr, "Searching...\n");
  if (planPath(W, app)) {
    fprintf(stderr, "Failure...\n");
  } else {
    fprintf(stderr, "Success...\n");
  }

  plotGraph(app);

  // get paths
  getPath1();
  getPath2();
  plotPaths(app);
  dumpPath(path1, app);
  dumpPath(path2, app);
}

PlannerRRTS::~PlannerRRTS() {
  // Nothing
}

int PlannerRRTS::checkPoint(Vector3i &ixyz, FlowData &W) {
  // check occ
  if (W.getObs(ixyz) == 1 || W.getOcc(ixyz) == 1) {
    return 0;
  }

  return 1;
}

Vector4d PlannerRRTS::randState(FlowData &W, App &app) {
  if (app.debug) {
    fprintf(stderr, "randState\n");
  }
  // get a sample in Cfree
  Vector3i samp_ixyz;
  int free = 0;

  // sample grid until point is in Cfree
  while (!free) {
    // probabilistically add goal node
    if (rand() % 100 < 1) {
      samp_ixyz = goal_ixyz;
    }
    // otherwise sample normally
    else {
      for (int i = 0; i < samp_ixyz.size(); i++) {
        // check if constraints are equal
        if (ixyz_max(i) == ixyz_min(i)) {
          samp_ixyz(i) = ixyz_max(i);
        } else {
          samp_ixyz(i) = (rand() % (ixyz_max(i) - ixyz_min(i))) + ixyz_min(i);
        }
      }
    }

    free = checkPoint(samp_ixyz, W);
  }

  // sample velocity
  //int vels = int((vel_g_max - vel_g_min) / vel_g_step);
  //double vel_samp = (double(rand() % vels) * vel_g_step) + vel_g_min;
  double vel_samp = 5;

  Vector4d samp_state = Vector4d::Zero();
  samp_state.segment(0, 3) = W.getXYZ(samp_ixyz);
  samp_state(3) = vel_samp;

  return samp_state;
}

NodeRRTS* PlannerRRTS::createNode(NodeRRTS* parent, Vector3i ixyz,
                                  double vel_g_mag, FlowData &W, App &app) {
  NodeRRTS* n = new NodeRRTS(parent, ixyz, vel_g_mag, W, mass, alpha, app);

  return n;
}

int PlannerRRTS::checkEdge(Vector3d &pos1, Vector3d &pos2, FlowData &W) {
  // get line between the two
  Vector3d dir = (pos2 - pos1).normalized();
  double dist = (pos2 - pos1).norm();
  int tot_steps = (int) floor(dist);
  Vector3d step = dist * dir / double(tot_steps);
  Vector3d test;
  Vector3i ind;

  // test along line
  for (int i = 0; i < tot_steps + 2; i++) {
    test = pos1 + double(i) * step;
    ind = W.getInd(test);

    if (checkPoint(ind, W) == 0) {
      return 0;
    }
  }

  return 1;
}

vector<NodeRRTS*> PlannerRRTS::findNear(
    NodeRRTS* n_new, flann::Index<flann::L2_3D<double> > &kdtree, App &app) {
  if (app.debug) {
    fprintf(stderr, "findNear\n");
  }
  // find nearest neighbors within a radius
  flann::Matrix<double> query(new double[3], 1, 3);
  n_new->node2Flann(query);
  //fprintf(stderr, "query: %f %f %f\n", query[0][0], query[0][1], query[0][2]);

  float rad = fmin(
      gamma
          * powf(log(double(N.size())) / double(N.size()),
                 (1 / double(n_new->state.size()))),
      eta);


  std::vector<std::vector<int> > indices;
  std::vector<std::vector<double> > dists;
  // need to scale search radius for some reason
  int near_count = kdtree.radiusSearch(query, indices, dists, rad * 15,
                                       radius_search_params);
  delete[] query.ptr();

  if (app.debug)  {
    fprintf(stderr, "near count: %i\n", near_count);
    fprintf(stderr, "near rad: %f\n", rad);
    plotNearArea(n_new->pos, rad, app);
  }

  vector<NodeRRTS*> N_near;
  int treeID = 0;
  for (vector<vector<int> >::iterator it1 = indices.begin();
      it1 != indices.end(); it1++) {
    for (vector<int>::iterator it2 = (*it1).begin(); it2 != (*it1).end();
        it2++) {
      treeID = *it2;

      // skip identical point
      if ((N[treeID]->pos - n_new->pos).squaredNorm() > 0.1) {
        N_near.push_back(N[treeID]);
      }
    }
  }

  return N_near;
}

NodeRRTS* PlannerRRTS::findNearest(Vector4d &state_rand,
                                   flann::Index<flann::L2_3D<double> > &kdtree,
                                   App &app) {
  //fprintf(stderr, "findNearest\n");
  // find nearest neighbors
  flann::Matrix<double> query(new double[3], 1, 3);
  query[0][0] = state_rand(0);
  query[0][1] = state_rand(1);
  query[0][2] = state_rand(2);

  int nn = 5;
  std::vector<std::vector<int> > indices;
  std::vector<std::vector<double> > dists;
  kdtree.knnSearch(query, indices, dists, nn, search_params);
  delete[] query.ptr();

  // check points
  NodeRRTS *n_near;
  int treeID = 0;
  for (vector<vector<int> >::iterator it1 = indices.begin();
      it1 != indices.end(); it1++) {
    for (vector<int>::iterator it2 = (*it1).begin(); it2 != (*it1).end();
        it2++) {
      treeID = *it2;

      // skip identical point
      if ((N[treeID]->pos - state_rand.segment(0, 3)).squaredNorm() > 0.1) {
        return N[treeID];
      }
    }
  }

  return NULL;
}

// find lowest cost node to connect with n_new
void PlannerRRTS::connectNode(NodeRRTS* n_new, vector<NodeRRTS*> &N_near,
                              FlowData &W, App &app) {
  if (app.debug) {
    fprintf(stderr, "connectNode\n");
  }
  // iterate over vector of nodes
  for (vector<NodeRRTS*>::iterator it = N_near.begin(); it != N_near.end();
      it++) {
    NodeRRTS* n_near = (*it);

    // check for collisions and cost
    //double cost = n_near->cost + getEdgeCost(n_near, n_new, W);
    double cost = n_new->getCost(n_near, W, app);
    if (checkEdge(n_near->pos, n_new->pos, W) && cost < n_new->cost) {
      n_new->parent = n_near;
      n_new->setCost(n_near, W, app);
      if (app.debug) {
        plotConnPos(n_near, app, 1);
      }
    } else {
      if (app.debug) {
        plotConnPos(n_near, app, 0);
      }
    }

    if (app.debug) {
      cin.ignore();
    }
  }

  return;
}

// replace the parents of near nodes with n_new if cost is lower
void PlannerRRTS::rewireTree(NodeRRTS* n_new, vector<NodeRRTS*> &N_near,
                             FlowData &W, App &app) {
  if (app.debug) {
    fprintf(stderr, "rewireNode\n");
  }
  // iterate over vector of nodes
  for (vector<NodeRRTS*>::iterator it = N_near.begin(); it != N_near.end();
      it++) {
    NodeRRTS* n_near = (*it);

    // check for collisions and cost
    double cost = n_near->getCost(n_new, W, app);
    if (checkEdge(n_near->pos, n_new->pos, W) && cost < n_near->cost) {
      n_near->parent = n_new;
      n_near->setCost(n_new, W, app);
      if (app.debug) {
        plotRewPos(n_near, app, 1);
      }
    } else {
      if (app.debug) {
        plotRewPos(n_near, app, 0);
      }
    }

    if (app.debug) {
      cin.ignore();
    }
  }

  return;
}

NodeRRTS * PlannerRRTS::newNode(NodeRRTS* n_nearest, Vector4d &state_rand,
                                FlowData &W, App &app) {
  if (app.debug) {
    fprintf(stderr, "newNode\n");
  }
  // get new node position
  Vector3d pos_dir = (state_rand.segment(0, 3) - n_nearest->pos).normalized();
  double pos_dist = 5;
  Vector3d pos = n_nearest->pos + pos_dist * pos_dir;
  //double speed_dist = fabs(n_rand->vel_g_mag - n_nearest->vel_g_mag);
  //double speed_dir = (n_rand->vel_g_mag - n_nearest->vel_g_mag) / speed_dist;
  //double speed = n_nearest->vel_g_mag + 0.5 * speed_dist * speed_dir;

  // create new node
  NodeRRTS* n_new = createNode(n_nearest, W.getInd(pos), double(state_rand(3)),
                               W, app);

  return n_new;
}

void PlannerRRTS::addNode(NodeRRTS* n,
                          flann::Index<flann::L2_3D<double> > &kdtree) {
  //fprintf(stderr, "addNode\n");
  // add to node list
  N.push_back(n);
  n->treeID = N.size();

  // add to kdtree
  flann::Matrix<double> p(new double[3], 1, 3);
  n->node2Flann(p);
  kdtree.addPoints(p, 4);

  return;
}

void PlannerRRTS::plotSG(App &app) {
  bot_lcmgl_point_size(app.lcmgl_sg, 10);
  bot_lcmgl_begin(app.lcmgl_sg, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_sg, 0, 1, 0);
  bot_lcmgl_vertex3d(app.lcmgl_sg, double(start_pos(0)), double(start_pos(1)),
                     double(start_pos(2)));
  bot_lcmgl_color3f(app.lcmgl_sg, 1, 0, 0);
  bot_lcmgl_vertex3d(app.lcmgl_sg, double(goal_pos(0)), double(goal_pos(1)),
                     double(goal_pos(2)));
  bot_lcmgl_end(app.lcmgl_sg);
  bot_lcmgl_switch_buffer(app.lcmgl_sg);
}

void PlannerRRTS::plotNodePos(NodeRRTS* n, App &app) {
  bot_lcmgl_point_size(app.lcmgl_node_pos, 5);
  bot_lcmgl_begin(app.lcmgl_node_pos, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_node_pos, 0, 0, 0);
  bot_lcmgl_vertex3d(app.lcmgl_node_pos, double(n->pos(0)), double(n->pos(1)),
                     double(n->pos(2)));
  bot_lcmgl_end(app.lcmgl_node_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_node_pos);
}

void PlannerRRTS::plotRandPos(Vector4d &state, App &app) {
  bot_lcmgl_point_size(app.lcmgl_rand_pos, 7);
  bot_lcmgl_begin(app.lcmgl_rand_pos, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_rand_pos, 0, 0, 0);
  bot_lcmgl_vertex3d(app.lcmgl_rand_pos, double(state(0)), double(state(1)),
                     double(state(2)));
  bot_lcmgl_end(app.lcmgl_rand_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_rand_pos);
}

void PlannerRRTS::plotNearestPos(NodeRRTS* n, App &app) {
  bot_lcmgl_point_size(app.lcmgl_nearest_pos, 7);
  bot_lcmgl_begin(app.lcmgl_nearest_pos, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_nearest_pos, 0, 1, 1);
  bot_lcmgl_vertex3d(app.lcmgl_nearest_pos, double(n->pos(0)),
                     double(n->pos(1)), double(n->pos(2)));
  bot_lcmgl_end(app.lcmgl_nearest_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_nearest_pos);
}

void PlannerRRTS::plotNearPos(vector<NodeRRTS*> N_near, App &app) {
  bot_lcmgl_point_size(app.lcmgl_near_pos, 7);
  bot_lcmgl_begin(app.lcmgl_near_pos, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_near_pos, 0.8, 0.2, 0.8);
  for (vector<NodeRRTS*>::iterator it = N_near.begin(); it != N_near.end();
      it++) {
    bot_lcmgl_vertex3d(app.lcmgl_near_pos, double((*it)->pos(0)),
                       double((*it)->pos(1)), double((*it)->pos(2)));
  }
  bot_lcmgl_end(app.lcmgl_near_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_near_pos);
}

void PlannerRRTS::plotConnPos(NodeRRTS* n, App &app, int update_flag) {
  bot_lcmgl_point_size(app.lcmgl_conn_pos, 7);
  bot_lcmgl_begin(app.lcmgl_conn_pos, GL_POINTS);
  if (update_flag) {
    bot_lcmgl_color3f(app.lcmgl_conn_pos, 0, 1, 0);
  } else {
    bot_lcmgl_color3f(app.lcmgl_conn_pos, 1, 0, 0);
  }

  bot_lcmgl_vertex3d(app.lcmgl_conn_pos, double(n->pos(0)), double(n->pos(1)),
                     double(n->pos(2)));
  bot_lcmgl_end(app.lcmgl_conn_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_conn_pos);
}

void PlannerRRTS::plotRewPos(NodeRRTS* n, App &app, int update_flag) {
  bot_lcmgl_point_size(app.lcmgl_rew_pos, 7);
  bot_lcmgl_begin(app.lcmgl_rew_pos, GL_POINTS);
  if (update_flag) {
    bot_lcmgl_color3f(app.lcmgl_rew_pos, 0, 1, 0);
  } else {
    bot_lcmgl_color3f(app.lcmgl_rew_pos, 1, 0, 0);
  }
  bot_lcmgl_vertex3d(app.lcmgl_rew_pos, double(n->pos(0)), double(n->pos(1)),
                     double(n->pos(2)));
  bot_lcmgl_end(app.lcmgl_rew_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_rew_pos);
}

void PlannerRRTS::plotNewPos(NodeRRTS* n, App &app) {
  bot_lcmgl_point_size(app.lcmgl_new_pos, 7);
  bot_lcmgl_begin(app.lcmgl_new_pos, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_new_pos, 0, 0, 0);
  bot_lcmgl_vertex3d(app.lcmgl_new_pos, double(n->pos(0)), double(n->pos(1)),
                     double(n->pos(2)));
  bot_lcmgl_end(app.lcmgl_new_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_new_pos);
}

void PlannerRRTS::plotColEdge(Vector3d pos1, Vector3d pos2, App &app) {
  bot_lcmgl_line_width(app.lcmgl_col_edge, 2);
  bot_lcmgl_begin(app.lcmgl_col_edge, GL_LINES);
  bot_lcmgl_color3f(app.lcmgl_col_edge, 0, 0, 0.5);
  bot_lcmgl_line3d(app.lcmgl_col_edge, double(pos1(0)), double(pos1(1)),
                   double(pos1(2)), double(pos2(0)), double(pos2(1)),
                   double(pos2(2)));
  bot_lcmgl_end(app.lcmgl_col_edge);
  bot_lcmgl_switch_buffer(app.lcmgl_col_edge);
}

void PlannerRRTS::plotGraph(App &app) {
  bot_lcmgl_line_width(app.lcmgl_graph_edges, 2);
  bot_lcmgl_begin(app.lcmgl_graph_edges, GL_LINES);
  bot_lcmgl_color3f(app.lcmgl_graph_edges, 0, 0.5, 0.5);
  for (vector<NodeRRTS*>::iterator it = N.begin() + 2; it != N.end(); it++) {
    bot_lcmgl_line3d(app.lcmgl_graph_edges, double((*it)->parent->pos(0)),
                     double((*it)->parent->pos(1)),
                     double((*it)->parent->pos(2)), double((*it)->pos(0)),
                     double((*it)->pos(1)), double((*it)->pos(2)));
  }
  bot_lcmgl_end(app.lcmgl_graph_edges);
  bot_lcmgl_switch_buffer(app.lcmgl_graph_edges);

  bot_lcmgl_point_size(app.lcmgl_graph_nodes, 3);
  bot_lcmgl_begin(app.lcmgl_graph_nodes, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_graph_nodes, 1, 0, 0);
  for (vector<NodeRRTS*>::iterator it = N.begin() + 1; it != N.end(); it++) {
    bot_lcmgl_vertex3d(app.lcmgl_graph_nodes, double((*it)->pos(0)),
                       double((*it)->pos(1)), double((*it)->pos(2)));
  }
  bot_lcmgl_end(app.lcmgl_graph_nodes);
  bot_lcmgl_switch_buffer(app.lcmgl_graph_nodes);

  return;
}

void PlannerRRTS::plotPathNode(list<NodeRRTS*> &path, App &app) {
  bot_lcmgl_point_size(app.lcmgl_path_nodes, 5);
  bot_lcmgl_begin(app.lcmgl_path_nodes, GL_POINTS);
  for (list<NodeRRTS*>::iterator it = path.begin(); it != path.end(); it++) {
    bot_lcmgl_color3f(app.lcmgl_path_nodes, 0, 0, 0);
    bot_lcmgl_vertex3d(app.lcmgl_path_nodes, double((*it)->pos(0)),
                       double((*it)->pos(1)), double((*it)->pos(2)));
  }
  bot_lcmgl_end(app.lcmgl_path_nodes);

  return;
}

void PlannerRRTS::plotNearArea(Vector3d pos, double rad, App &app) {
  bot_lcmgl_begin(app.lcmgl_near_area, GL_LINES);
  bot_lcmgl_line_width(app.lcmgl_path_edges, 7);
  bot_lcmgl_color3f(app.lcmgl_near_area, 1, 0, 0);
  double xyz[3];
  xyz[0] = pos(0);
  xyz[1] = pos(1);
  xyz[2] = pos(2);
  bot_lcmgl_circle(app.lcmgl_near_area, xyz, rad);
  bot_lcmgl_end(app.lcmgl_near_area);
  bot_lcmgl_switch_buffer(app.lcmgl_near_area);

  return;
}

void PlannerRRTS::plotPathEdge(list<NodeRRTS*> &path, App &app) {
  // calc cost/dist for each segment
  VectorXd cost_per_dist;
  cost_per_dist.setZero(path.size() - 1);
  for (list<NodeRRTS*>::iterator it = path.begin(); it != path.end(); it++) {
    if (it == path.begin())
      continue;

    cost_per_dist(distance(path.begin(), it) - 1) = ((*it)->cost
        - (*it)->parent->cost) / ((*it)->dist - (*it)->parent->dist);
  }

  // find max cost per dist
  double max_cpd = 0;
  for (int i = 0; i < cost_per_dist.size(); i++) {
    if (cost_per_dist(i) > max_cpd) {
      max_cpd = cost_per_dist(i);
    }
  }

  bot_lcmgl_line_width(app.lcmgl_path_edges, 7);
  bot_lcmgl_begin(app.lcmgl_path_edges, GL_LINES);
  for (list<NodeRRTS*>::iterator it = path.begin(); it != path.end(); it++) {
    if (it == path.begin())
      continue;

    float * color = bot_color_util_jet(
        double(cost_per_dist(distance(path.begin(), it) - 1) / max_cpd));
    bot_lcmgl_color3f(app.lcmgl_path_edges, color[0], color[1], color[2]);
    bot_lcmgl_line3d(app.lcmgl_path_edges, double((*it)->parent->pos(0)),
                     double((*it)->parent->pos(1)),
                     double((*it)->parent->pos(2)), double((*it)->pos(0)),
                     double((*it)->pos(1)), double((*it)->pos(2)));
  }
  bot_lcmgl_end(app.lcmgl_path_edges);

  return;
}

void PlannerRRTS::clearBuffersEnd(App &app) {
  bot_lcmgl_switch_buffer(app.lcmgl_node_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_nearest_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_near_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_conn_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_rew_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_new_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_col_edge);
  bot_lcmgl_switch_buffer(app.lcmgl_near_area);
  bot_lcmgl_switch_buffer(app.lcmgl_cost_edge);
  bot_lcmgl_switch_buffer(app.lcmgl_cost_pos);

  return;
}

void PlannerRRTS::clearBuffersMid(App &app) {
  bot_lcmgl_switch_buffer(app.lcmgl_node_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_conn_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_rew_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_new_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_col_edge);
  bot_lcmgl_switch_buffer(app.lcmgl_cost_edge);
  bot_lcmgl_switch_buffer(app.lcmgl_cost_pos);

  return;
}

void PlannerRRTS::getPath1() {
  fprintf(stderr, "Getting path1...\n");

  // find node in goal region
  NodeRRTS* n = NULL;
  vector<NodeRRTS*>::iterator it;
  for (it = N.begin() + 1; it != N.end(); it++) {
    n = (*it);

    if ((n->pos - goal_pos).squaredNorm() < 5.0
        && n->pos(0) < n->parent->pos(0)) {
      break;
    }
  }
  //n->print();

  // check if we found a start node
  if (it != N.end()) {
    while (n != NULL) {
      //n.print();
      path1.push_front(n);
      n = n->parent;
    }
  } else {
    fprintf(stderr,
            "Warning: No suitable start node in PlannerRRTS::getPath1\n");
  }

  return;
}

void PlannerRRTS::getPath2() {
  fprintf(stderr, "Getting path2...\n");

  // find node in goal region
  NodeRRTS* n = NULL;
  vector<NodeRRTS*>::iterator it;
  for (it = N.begin() + 1; it != N.end(); it++) {
    n = (*it);

    if ((n->pos - goal_pos).squaredNorm() < 5.0
        && n->pos(0) > n->parent->pos(0)) {
      break;
    }
  }
  //n->print();

  // check if we found a start node
  if (it != N.end()) {
    while (n != NULL) {
      //n.print();
      path2.push_front(n);
      n = n->parent;
    }
  } else {
    fprintf(stderr,
            "Warning: No suitable start node in PlannerRRTS::getPath2\n");
  }

  return;
}

void PlannerRRTS::plotPaths(App &app) {
  fprintf(stderr, "Plotting path...\n");

  // path1
  if (path1.size() > 0) {
    plotPathNode(path1, app);
    plotPathEdge(path1, app);
  }

  // path2
  if (path2.size()) {
    plotPathNode(path2, app);
    plotPathEdge(path2, app);
  }

  bot_lcmgl_switch_buffer(app.lcmgl_path_nodes);
  bot_lcmgl_switch_buffer(app.lcmgl_path_edges);

  return;
}

void PlannerRRTS::dumpPath(list<NodeRRTS*> &path, App &app) {
  fprintf(stderr, "Dumping data...\n");

  MatrixXd dump;
  dump.setZero(path.size(), 22);

  int i = 0;
  for (list<NodeRRTS*>::iterator it = path.begin(); it != path.end(); it++) {
    // 3 vectors
    for (int j = 0; j < 3; j++) {
      dump(i, j) = (*it)->ixyz(j);
      dump(i, j + 3) = (*it)->pos(j);
      dump(i, j + 6) = (*it)->vel_f(j);
      dump(i, j + 9) = (*it)->vel_g(j);
      dump(i, j + 12) = (*it)->vel_a(j);
    }

    // cost terms
    dump(i, 15) = (*it)->vel_a_mag;
    dump(i, 16) = (*it)->vel_g_mag;
    dump(i, 17) = (*it)->energy_pot;
    dump(i, 18) = (*it)->energy_sto;
    dump(i, 19) = (*it)->energy;
    dump(i, 20) = (*it)->dist;
    dump(i, 21) = (*it)->cost;

    i++;
  }

  eigen_matlab_dump(dump);

  return;
}

int PlannerRRTS::planPath(FlowData &W, App &app) {
  // create start node
  NodeRRTS* n_start = createNode(NULL, start_ixyz, 0, W, app);
  //n_start->parent = n_start;
  flann::Matrix<double> p_start(new double[3], 1, 3);
  n_start->node2Flann(p_start);
  N.push_back(n_start);

  // create tree
  flann::Index<flann::L2_3D<double> > kdtree(p_start,
                                             flann::KDTreeSingleIndexParams(4));
  kdtree.removePoint(0);
  addNode(n_start, kdtree);

  int i_max = 10000;
  for (int i = 0; i < i_max; i++) {
    if (app.debug) {
      fprintf(stderr, "i: %i, N: %lu, t: %lu\n", i, N.size() - 1, kdtree.size());
    }

    if (i % (i_max/10) == 0) {
      fprintf(stderr, "i: %f percent\n", double(i)/double(i_max)*100);
    }

    // random node in Cfree
    Vector4d state_rand = randState(W, app);
    if (app.debug) {
      plotRandPos(state_rand, app);
    }

    // find nearest node using FLANN, reject identical point
    NodeRRTS* n_nearest = findNearest(state_rand, kdtree, app);

    // did we find a node?
    if (n_nearest != NULL) {
      //fprintf(stderr, "found nearest\n");
      if (app.debug) {
        plotNearestPos(n_nearest, app);
      }

      // create node using some function and check for collisions
      NodeRRTS* n_new = newNode(n_nearest, state_rand, W, app);
      //n_new->print();
      if (app.debug) {
        plotNewPos(n_new, app);
      }

      // check for collisions
      if (checkEdge(n_nearest->pos, n_new->pos, W)) {
        //fprintf(stderr, "no collisions\n");
        // find near nodes
        vector<NodeRRTS*> N_near = findNear(n_new, kdtree, app);

        // did we get any nodes?
        if (N_near.size() > 0) {
          if (app.debug) {
            plotNearPos(N_near, app);
          }
          //fprintf(stderr, "found near\n");
          // add new node
          addNode(n_new, kdtree);
          if (app.debug) {
            plotGraph(app);
            cin.ignore();
            clearBuffersMid(app);
          }

          // connect along minimum cost path
          connectNode(n_new, N_near, W, app);
          if (app.debug) {
            plotGraph(app);
            cin.ignore();
            clearBuffersMid(app);
          }

          // rewire the tree
          rewireTree(n_new, N_near, W, app);
          if (app.debug) {
            plotGraph(app);
            cin.ignore();
            clearBuffersMid(app);
          }
        } else {
          //fprintf(stderr, "Warning: findNear returned nothing for i: %i\n", i);
        }
      }
    } else {
      //fprintf(stderr, "Warning: findNearest returned nothing for i: %i\n", i);
    }

    if (app.debug) {
      cin.ignore();
      clearBuffersEnd(app);
    }
  }

  return 0;
}
