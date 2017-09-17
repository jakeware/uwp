// Copyright 2015 Jake Ware

// c system includes

// cpp system includes

// external includes

// project includes
#include "planner/planner.h"

Planner::Planner(FlowData * W, App * app) {
  if (!app->quiet && app->verbose) {
    fprintf(stderr, "Starting planner...\n");
  }

  initMap(W, app);

  if (!app->test_sg) {
    // min planner bounds
    app->pos_min = W->getXYZ(app->ixyz_min);
    app->state_min.segment(0, 3) = app->pos_min;
    app->state_min(3) = app->vel_g_min;

    // max planner bounds
    app->pos_max = W->getXYZ(app->ixyz_max);
    app->state_max.segment(0, 3) = app->pos_max;
    app->state_max(3) = app->vel_g_max;

    // initialize start states
    app->start_pos = app->start_state.segment(0, 3);
    app->start_ixyz = W->getInd(app->start_pos);

    // initialize goal states
    app->goal_pos = app->goal_state.segment(0, 3);
    app->goal_ixyz = W->getInd(app->goal_pos);

    // discretizations
    app->x_step = W->res;
    app->y_step = W->res;
    app->z_step = W->res;
    getSteps(app);
    getDState(app);

    if (app->verbose) {
      fprintf(stderr, "Start: x: %0.1f y: %0.1f z: %0.1f vel: %0.1f\n",
              app->start_state(0), app->start_state(1), app->start_state(2),
              app->start_state(3));
      fprintf(stderr, "Goal1: x: %0.1f y: %0.1f z: %0.1f vel: %0.1f\n",
              app->goal_state(0), app->goal_state(1), app->goal_state(2),
              app->goal_state(3));
    }

    // node storage array
    std::vector<std::vector<std::vector<Node*> > > N;

    // initialize nodes
    initNodes(&N, W, app);

    // get graph
    // TODO(jakeware): make this a param
    SearchAS G(&N);
    // SearchD G(&N);

    if (!G.getGraph(W, app)) {
      if (!app->quiet) {
        fprintf(stderr, "Success: Found goal\n");
      }

      // find optimal path in graph
      std::list<Node*> path;
      getPath(&path, &N, W, app);

      // find path given naive planner and dynamics constraints
      if (app->vel_a_cap_flag && app->naive_flag) {
        checkPath(&path, &N, W, app);
      }

      path_ = path;

      // plot initial path
      if (app->plot) {
        plotPathEdgeVel(&path, "path", app);
        plotPathNodePos(&path, "path", app);
      }

      // dump path to file
      if (app->dump) {
        // printPath(&path, app);
        dumpPath(&path, "path", app);
      }

      // sample path
      // std::list<Node*> path_samp;
      // sampPathEnergy(&path, &path_samp, W, app);

      // plot sampled path
      if (app->plot) {
        // plotPathEdgeVel(&path_samp, "path_samp", app);
        // plotPathNodePos(&path_samp, "path_samp", app);
      }

      // dump path to file
      if (app->dump) {
        // dumpPath(&path_samp, "path_samp", app);
      }

      // std::list<Node*> path_samp_col;
      // collisionCheckPath(&path, &path_samp, &path_samp_col, W, app);

      // plot sampled path with collision checking
      if (app->plot) {
        // plotPathEdgeVel(&path_samp_col, "path_samp_col", app);
        // plotPathNodePos(&path_samp_col, "path_samp_col", app);
      }

      // dump path to file
      if (app->dump) {
        // dumpPath(&path_samp_col, "path_samp_col", app);
      }

      // publish path to lcm
      if (app->publish) {
        // publishPath(&path, app);
      }
    } else {
      fprintf(stderr, "Error: Could not find goal\n");
    }

    freeMem(&N, W, app);
  }
}

Planner::~Planner() {
  // Nothing
}

Eigen::VectorXd Planner::getSegmentTimes(std::list<Node*> * path,
                                         std::list<Node*> * path_samp_col,
                                         FlowData* W, App * app) {
  if (!app->quiet && app->verbose) {
    fprintf(stderr, "getSegmentTimes... ");
  }

  Eigen::VectorXd segment_times;
  segment_times.setZero(path_samp_col->size() - 1);

  // iterate over path_samp_col
  for (std::list<Node*>::iterator it = ++path_samp_col->begin();
      it != path_samp_col->end(); it++) {
    int i = std::distance(path_samp_col->begin(), it) - 1;

    std::list<Node*>::iterator it_last = it;
    it_last--;

    // find both it and it_last in path
    std::list<Node*>::iterator it_tail;
    std::list<Node*>::iterator it_tip;
    for (std::list<Node*>::iterator it_path = path->begin();
        it_path != path->end(); it_path++) {
      // did we find the segment end node?
      if ((*it_path)->ixyz == (*it)->ixyz) {
        it_tip = it_path;
      }

      // did we find the segment start node?
      if ((*it_path)->ixyz == (*it_last)->ixyz) {
        it_tail = it_path;
      }
    }

    // calculate the total distance
    std::list<Node*>::iterator it_end = it_tip;
    it_end++;
    for (std::list<Node*>::iterator it_path = ++it_tail; it_path != it_end;
        it_path++) {
      std::list<Node*>::iterator it_path_last = it_path;
      it_path_last--;

      double dist = ((*it_path)->pos - (*it_path_last)->pos).norm();
      double speed = (*it_path)->vel_g_mag;

      segment_times(i) += dist / speed;
    }
  }

  if (!app->quiet && app->verbose) {
    fprintf(stderr, "Done\n");
  }

  return segment_times;
}

// QuadTrajectory * Planner::getQuadTraj(std::list<Node*> * path, App * app) {
//   if (!app->quiet && app->verbose) {
//     fprintf(stderr, "getQuadTraj... ");
//   }

//   // create QuadTrajectory
//   QuadTrajectory * quad_traj = new QuadTrajectory(app->param, app->lcm);

//   // time segments (taus)
//   Eigen::VectorXd segment_times;
//   segment_times.setZero(path->size() - 1);

//   // derivatives at each path node
//   std::list<Node*>::iterator it = path->begin();
//   Eigen::VectorXd * start_state = new Eigen::VectorXd;
//   start_state->setZero(20);
//   (*start_state)(0) = (*it)->pos(0);
//   (*start_state)(1) = (*it)->pos(1);
//   (*start_state)(2) = (*it)->pos(2);
//   (*start_state)(3) = 0.0;
//   (*start_state)(4) = (*it)->vel_g(0);
//   (*start_state)(5) = (*it)->vel_g(1);
//   (*start_state)(6) = (*it)->vel_g(2);
//   (*start_state)(7) = 0.0;
//   quad_traj->vector_vert_list.push_back(start_state);

//   // get segment times and derivatives
//   for (std::list<Node*>::iterator it = ++path->begin(); it != path->end();
//       it++) {
//     int i = std::distance(++path->begin(), it);

//     // get last node in path
//     std::list<Node*>::iterator it_last = it;
//     it_last--;

//     // get taus
//     double edge_dist = ((*it)->pos - (*it_last)->pos).norm();
//     double vel_g_avg = ((*it)->vel_g_mag + (*it_last)->vel_g_mag) / 2.0;
//     double edge_time = edge_dist / vel_g_avg;
//     segment_times(i) = edge_time;

//     // get yaw
//     Eigen::Vector3f diff = ((*it)->pos - (*it_last)->pos).normalized();
//     double psi = atan2(static_cast<double>(diff(1)),
//                        static_cast<double>(diff(0)));

//     // get state (x, y, z, yaw, xdot, ydot, zdot, yawdot, xddot, ...)
//     Eigen::VectorXd * state = new Eigen::VectorXd;
//     state->setZero(20);
//     (*state)(0) = (*it)->pos(0);
//     (*state)(1) = (*it)->pos(1);
//     (*state)(2) = (*it)->pos(2);
//     (*state)(3) = psi;
//     (*state)(4) = (*it)->vel_g(0);
//     (*state)(5) = (*it)->vel_g(1);
//     (*state)(6) = (*it)->vel_g(2);
//     (*state)(7) = 0.0;

//     quad_traj->vector_vert_list.push_back(state);
//   }

//   app->lstore["poly_path"];
//   app->lstore["traj_axes"];
//   app->lstore["vel_vec"];
//   app->lstore["accel_vec"];
//   app->lstore.switchAllBuffers();
//   quad_traj->poly_path = app->lstore["poly_path"];
//   quad_traj->traj_axes = app->lstore["traj_axes"];
//   quad_traj->vel_vec = app->lstore["vel_vec"];
//   quad_traj->accel_vec = app->lstore["accel_vec"];

//   // TODO(jakeware): free state list memory

//   if (!app->quiet && app->verbose) {
//     fprintf(stderr, "Done\n");
//   }

//   return quad_traj;
// }

// QuadPath Planner::getQuadPath(std::list<Node*> * path, App * app) {
//   if (!app->quiet && app->verbose) {
//     fprintf(stderr, "getQuadPath... ");
//   }

//   // time segments (taus)
//   Eigen::VectorXd segment_times;
//   segment_times.setZero(path->size() - 1);

//   // derivatives at each path node
//   std::list<Eigen::VectorXd *> state_list;
//   std::list<Node*>::iterator it = path->begin();
//   Eigen::VectorXd * start_state = new Eigen::VectorXd;
//   start_state->setZero(20);
//   (*start_state)(0) = (*it)->pos(0);
//   (*start_state)(1) = (*it)->pos(1);
//   (*start_state)(2) = (*it)->pos(2);
//   (*start_state)(3) = 0.0;
//   (*start_state)(4) = (*it)->vel_g(0);
//   (*start_state)(5) = (*it)->vel_g(1);
//   (*start_state)(6) = (*it)->vel_g(2);
//   (*start_state)(7) = 0.0;
//   state_list.push_back(start_state);

//   // get segment times and derivatives
//   for (std::list<Node*>::iterator it = ++path->begin(); it != path->end();
//       it++) {
//     int i = std::distance(++path->begin(), it);

//     // get last node in path
//     std::list<Node*>::iterator it_last = it;
//     it_last--;

//     // get taus
//     double edge_dist = ((*it)->pos - (*it_last)->pos).norm();
//     double vel_g_avg = ((*it)->vel_g_mag + (*it_last)->vel_g_mag) / 2.0;
//     double edge_time = edge_dist / vel_g_avg;

//     segment_times(i) = edge_time;

//     // get state (x, y, z, yaw, xdot, ydot, zdot, yawdot, xddot, ...)

//     Eigen::Vector3f diff = ((*it)->pos - (*it_last)->pos).normalized();
//     double psi = atan2(static_cast<double>(diff(1)),
//                        static_cast<double>(diff(0)));

//     Eigen::VectorXd * state = new Eigen::VectorXd;
//     state->setZero(20);
//     (*state)(0) = (*it)->pos(0);
//     (*state)(1) = (*it)->pos(1);
//     (*state)(2) = (*it)->pos(2);
//     (*state)(3) = psi;
//     (*state)(4) = (*it)->vel_g(0);
//     (*state)(5) = (*it)->vel_g(1);
//     (*state)(6) = (*it)->vel_g(2);
//     (*state)(7) = 0.0;

//     state_list.push_back(state);
//   }

//   QuadPath quad_path(state_list, segment_times);
//   // quad_path.segment_times = quad_path.getSegmentTimes();
//   quad_path.segment_times = segment_times;

//   // free memory
//   for (std::list<Eigen::VectorXd *>::iterator it = state_list.begin();
//       it != state_list.end(); it++) {
//     free(*it);
//   }

//   if (!app->quiet && app->verbose) {
//     fprintf(stderr, "Done\n");
//   }

//   return quad_path;
// }

void Planner::sampPathSimple(std::list<Node*> * path,
                             std::list<Node*> * samp_path, App * app) {
  if (!app->quiet && app->verbose) {
    fprintf(stderr, "sampPathSimple... ");
  }

  // create array to mark nodes for removal
  Eigen::VectorXi remove;
  remove.setZero(path->size());

  // iterate over path and mark for removal
  for (std::list<Node*>::iterator it = ++path->begin(); it != path->end();
      it++) {
    if (((*it)->vel_g - (*it)->parent->vel_g).norm() == 0) {
      remove(std::distance(path->begin(), it)) = 1;
    }
  }

  fprintf(stderr, "path size: %lu\n", path->size());

  // iterate over path and copy selected nodes
  for (std::list<Node*>::iterator it = path->begin(); it != path->end(); it++) {
    if (!remove(std::distance(path->begin(), it))) {
      samp_path->push_back(*it);
    }
  }

  fprintf(stderr, "samp_path size: %lu\n", samp_path->size());

  if (!app->quiet && app->verbose) {
    fprintf(stderr, "Done\n");
  }

  return;
}

void Planner::collisionCheckPath(std::list<Node*> * path,
                                 std::list<Node*> * samp_path,
                                 std::list<Node*> * samp_path_col, FlowData * W,
                                 App * app) {
  if (!app->quiet && app->verbose) {
    fprintf(stderr, "collisionCheckPath... ");
  }

  // add start node
  std::list<Node*>::iterator it_start = samp_path->begin();
  samp_path_col->push_back(*it_start);

  // iterate over list looking for collision
  for (std::list<Node*>::iterator it = ++samp_path->begin();
      it != samp_path->end(); it++) {
    // fprintf(stderr, "node: %lu\n", std::distance(samp_path->begin(), it));

    std::list<Node*>::iterator it_last = it;
    it_last--;
    bool collision = collisionCheck((*it_last)->pos, (*it)->pos, false, W, app);

    // did we get a collision?
    if (collision) {
      // plotEdge((*it_last)->pos, (*it)->pos, "collision");

      // find first edge node in original path
      std::list<Node*>::iterator it_tail;
      for (std::list<Node*>::iterator it_path = ++path->begin();
          it_path != path->end(); it_path++) {
        if ((*it_last)->state == (*it_path)->state) {
          it_tail = it_path;
          break;
        }
      }

      // look for first collision point between it_tail and it_tip
      std::list<Node*>::iterator it_tip = it_tail;
      it_tip++;

      // Note: compare indices because iterators are from different paths
      while ((*it_tip)->ixyz != (*it)->ixyz) {
        // collision check
        bool collision_test = collisionCheck((*it_tail)->pos, (*it_tip)->pos,
                                             false, W, app);

        if (collision_test) {
          it_tail = it_tip;
          it_tail--;

          samp_path_col->push_back(*it_tail);
        } else {
          it_tip++;
        }

        // std::cin.get();
      }

      samp_path_col->push_back(*it);
    } else {
      // no collision, push node
      samp_path_col->push_back(*it);
    }
  }

  fprintf(stderr, "samp_path_col size: %lu\n", samp_path_col->size());

  if (!app->quiet && app->verbose) {
    fprintf(stderr, "Done\n");
  }

  return;
}

bool Planner::collisionCheck(Eigen::Vector3f pos1, Eigen::Vector3f pos2,
                             bool debug, FlowData * W, App * app) {
  Eigen::Vector3f dir = (pos2 - pos1).normalized();
  float step_size = 0.1;  // [m]
  float total_dist = (pos2 - pos1).norm();
  int steps = ceil(total_dist / step_size);

  float current_dist = 0.0;
  float last_dist = 0.0;
  bool collision = false;

  for (int i = 1; i <= steps; i++) {
    // get current time
    last_dist = current_dist;
    current_dist = static_cast<float>(i) * step_size;

    // check if time is greater than total time
    if (current_dist > total_dist) {
      // fprintf(stderr, "total time:%f\n", total_time);
      current_dist = total_dist;
    }

    // get current state
    Eigen::Vector3f pos = pos1 + current_dist * dir;

    // get index
    Eigen::Vector3i ixyz = W->getInd(pos);

    // check for collision
    if (W->getObs3D(ixyz) > 0) {
      collision = true;
    }

    if (debug) {
      plotPos(pos, "step", app);
      std::cin.get();
    }
  }

  return collision;
}

double Planner::getEnergyDelta(Eigen::Vector4f state1, Eigen::Vector4f state2,
                               float dt, bool debug, FlowData * W, App * app) {
  Eigen::Vector3f pos1 = state1.segment(0, 3);
  Eigen::Vector3f pos2 = state2.segment(0, 3);
  float vel_g_mag = state2(3);
  Eigen::Vector3f dir = (pos2 - pos1).normalized();
  Eigen::Vector3f vel_g = dir * vel_g_mag;

  double total_dist = (pos2 - pos1).norm();
  double total_time = total_dist / vel_g_mag;
  int steps = ceil(total_time / dt);

  float total_energy = 0.0;
  float current_time = 0.0;
  float last_time = 0.0;
  float energy_sto = 0.0;
  float vel_a_mag = 0.0;

  for (int i = 1; i <= steps; i++) {
    // get current time
    last_time = current_time;
    current_time = static_cast<float>(i) * dt;

    // check if time is greater than total time
    if (current_time > total_time) {
      // fprintf(stderr, "total time:%f\n", total_time);
      current_time = total_time;
    }

    // get current state
    Eigen::Vector3f pos = pos1 + current_time * vel_g;

    // get index
    Eigen::Vector3i ixyz = W->getInd(pos);

    // get wind at state2
    Eigen::Vector3f vel_f = W->getUVW(ixyz);

    // get air speed
    Eigen::Vector3f vel_a = vel_g - vel_f;
    vel_a_mag = vel_a.norm();

    // get stored energy
    energy_sto = app->pow_prof(0) * powf(vel_a_mag, 4)
        + app->pow_prof(1) * powf(vel_a_mag, 3)
        + app->pow_prof(2) * powf(vel_a_mag, 2) + app->pow_prof(3) * vel_a_mag
        + app->pow_prof(4);
    energy_sto *= current_time - last_time;

    total_energy += energy_sto;

    if (debug) {
      fprintf(stderr, "i:%i\n", i);
      fprintf(stderr, "current time:%f\n", current_time);
      plotPos(pos, "step", app);
      eigen_dump(ixyz);
      eigen_dump(vel_f);
      eigen_dump(vel_a_mag);
      std::cin.get();
    }
  }

  // get potential energy
  double energy_pot = (pos2(3) - pos1(3)) * app->mass * app->g;

  total_energy += energy_pot;

  return total_energy;
}

void Planner::sampPathEnergy(std::list<Node*> * path,
                             std::list<Node*> * samp_path, FlowData * W,
                             App * app) {
  if (!app->quiet && app->verbose) {
    fprintf(stderr, "sampPathEnergy... ");
  }

  fprintf(stderr, "\npath size: %lu\n", path->size());

  // print total energy consumption for original path
  std::list<Node*>::iterator it = --path->end();
  fprintf(stderr, "path_total_energy:%f\n", (*it)->g);

  // create array to mark nodes for removal
  Eigen::VectorXi remove;
  remove.setZero(path->size());

  // energy ratio threshold
  double energy_ratio_threshold = 0.02;

  // iterate over path and mark for removal
  std::list<Node*>::iterator it_tail = path->begin();
  std::list<Node*>::iterator it_tip = ++path->begin();
  std::list<Node*>::iterator it_tip_last;

  // iterate over path
  int count = 0;
  float total_energy = 0.0;
  while (it_tip != path->end()) {
    // get energy from original path between tail and tip
    double energy_diff_orig = (*it_tip)->g - (*it_tail)->g;

    // integrate energy between tail and tip with new velocity
    double energy_diff = getEnergyDelta((*it_tail)->state, (*it_tip)->state,
                                        0.1, false, W, app);
    double energy_ratio = fabs(energy_diff - energy_diff_orig)
        / energy_diff_orig;

    // check difference in energy
    // TODO(jakeware): fix potential infinite loop here if can't meet constraint
    if (energy_ratio > energy_ratio_threshold) {
      // check to see if we are stuck in a single step
      if (std::distance(it_tail, it_tip) < 2) {
        fprintf(stderr, "Stuck on single step.  Relaxing.  Energy ratio: %f\n",
                energy_ratio);

        eigen_dump(energy_diff_orig);
        eigen_dump(energy_diff);

        double temp = getEnergyDelta((*it_tail)->state, (*it_tip_last)->state,
                                     0.1, true, W, app);
        fprintf(stderr, "return\n");

        plotPos((*it_tail)->pos, "tail", app);
        plotEdge((*it_tail)->pos, (*it_tip)->pos, "tail2tip", app);
        Eigen::Vector3i ixyz_tail = W->getInd((*it_tail)->pos);
        Eigen::Vector3i ixyz_tip = W->getInd((*it_tip)->pos);
        plotPos(W->getXYZ(ixyz_tail), "tail_test", app);
        plotPos(W->getXYZ(ixyz_tip), "tip_test", app);

        std::cin.get();
        energy_ratio_threshold = 0.05;
      }

      // mark tip node for retention
      remove(std::distance(path->begin(), it_tip)) = 0;

      // get last tip node
      it_tip_last = it_tip;
      it_tip_last--;

      total_energy += getEnergyDelta((*it_tail)->state, (*it_tip_last)->state,
                                     0.1, false, W, app);
      // plotEdgePos((*it_tail)->pos, (*it_tip_last)->pos);

      // mark previous node for retention
      remove(std::distance(path->begin(), it_tip_last)) = 0;

      // move tail to last tip
      it_tail = it_tip_last;
    } else {
      // reset threshold
      energy_ratio_threshold = 0.02;

      // mark node for removal
      remove(std::distance(path->begin(), it_tip)) = 1;

      // increment tip
      it_tip++;
    }

    // std::cin.get();

    count++;
  }

  // add energy from final segment
  it_tip_last = it_tip;
  it_tip_last--;

  total_energy += getEnergyDelta((*it_tail)->state, (*it_tip_last)->state, 0.1,
                                 false, W, app);
  // plotEdgePos((*it_tail)->pos, (*it_tip_last)->pos);

  // keep goal
  remove(path->size() - 1) = 0;
  // eigen_dump(remove);

  // iterate over path and copy selected nodes (ignore cost)

  // push start node
  std::list<Node*>::iterator it_start = path->begin();
  Node * n_start = createNode((*it_start)->parent, (*it_start)->ixyz,
                              (*it_start)->vel_g_mag, W, app);
  samp_path->push_back(n_start);

  for (std::list<Node*>::iterator it = ++path->begin(); it != path->end();
      it++) {
    //    fprintf(stderr, "i:%lu\n", std::distance(path->begin(), it));

    if (!remove(std::distance(path->begin(), it))) {
      //    eigen_dump((*it)->ixyz);
      //    eigen_dump((*it)->vel_g_mag);

      std::list<Node*>::iterator it_last = --samp_path->end();

      // create new node
      Node * n = createNode((*it_last), (*it)->ixyz, (*it)->vel_g_mag, W, app);

      samp_path->push_back(n);
    }
  }

  fprintf(stderr, "samp_path size: %lu\n", samp_path->size());
  fprintf(stderr, "samp_path_total_energy:%f\n", total_energy);

  if (!app->quiet && app->verbose) {
    fprintf(stderr, "Done\n");
  }

  return;
}

int Planner::checkDynamics(Node * n, Node * parent, FlowData* W, App * app) {
  // check air velocity
  if (n->getVelAMagTrue(parent, W) > app->vel_a_max && app->vel_a_cap_flag
      && app->naive_flag) {
    return 0;
  }

  return 1;
}

void Planner::checkPath(std::list<Node*> * path,
                        std::vector<std::vector<std::vector<Node*> > > * N,
                        FlowData * W, App * app) {
  // fprintf(stderr, "checkPath\n");

  Node* n = *(path->begin());
  // loop over path from start + 1 to goal
  int count = 1;
  bool fail = false;
  bool once = true;
  for (std::list<Node*>::iterator it = ++path->begin(); it != path->end();
      it++) {
    // fprintf(stderr, "i: %i\n", count++);
    (*it)->parent = n;
    (*it)->setNode(n, W, app);

    // check air speed
    int valid = checkDynamics((*it), (*it)->parent, W, app);

    // if valid, assign and move on
    if (valid) {
      n = (*it);
    } else {  // otherwise decrement ground speed
      // reduce air speed until valid or return failure
      Eigen::Vector3i ind = getNodeInd((*it)->state, app);
      while (!valid) {
        // fprintf(stderr, "not valid\n");
        // check if ground speed can be lowered
        if (ind(2) > 0) {
          // decrement ground speed
          ind(2) = ind(2) - 1;
          n = (*N)[ind(0)][ind(1)][ind(2)];

          // check validity
          valid = checkDynamics(n, (*it)->parent, W, app);
        } else {
          if (once) {
            once = false;
            fprintf(
                stderr,
                "Warning: checkPath found an infeasible naive trajectory\n");
          }

          fail = true;
          valid = 1;
        }
      }

      // assign new node
      n->parent = (*it)->parent;
      n->setNode((*it)->parent, W, app);
    }

    (*it) = n;

    // has a past node failed already?
    if (fail) {
      n->g_true = std::numeric_limits<double>::infinity();
    }
  }

  return;
}

void Planner::initMap(FlowData * W, App * app) {
  // get start pos and index
  app->start_pos = app->start_state.segment(0, 3);
  app->start_ixyz = W->getInd(app->start_pos);

  // check if start_pos is valid
  // TODO(jakeware): fix segfault that occurs after this is done
  Eigen::Vector3f start_pos_test = Eigen::Vector3f::Zero();
  start_pos_test = W->getXYZ(app->start_ixyz);
  if (app->start_pos != start_pos_test) {
    fprintf(stderr, "Warning: Invalid start_pos not on grid.\n");
    app->start_pos = start_pos_test;
    fprintf(stderr, "Resetting to: %f, %f, %f\n", app->start_pos(0),
            app->start_pos(1), app->start_pos(2));
  }

  // get goal pos and index
  app->goal_pos = app->goal_state.segment(0, 3);
  app->goal_ixyz = W->getInd(app->goal_pos);

  // check if goal_pos is valid
  Eigen::Vector3f goal_pos_test = Eigen::Vector3f::Zero();
  goal_pos_test = W->getXYZ(app->goal_ixyz);
  if (app->goal_pos != goal_pos_test) {
    fprintf(stderr, "Warning: Invalid goal_pos not on grid.\n");
    app->goal_pos = goal_pos_test;
    fprintf(stderr, "Resetting to: %f, %f, %f\n", app->goal_pos(0),
            app->goal_pos(1), app->goal_pos(2));
  }

  // check if start and goal locations are in obstacle space
  if (W->getObs3D(app->start_ixyz)) {
    fprintf(stderr, "Start Location in Obstacle Space\n");
  }

  if (W->getObs3D(app->goal_ixyz)) {
    fprintf(stderr, "Goal Location in Obstacle Space\n");
  }

  // plotting
  if (app->plot) {
    plotSG(app->start_state.segment(0, 3), app->goal_state.segment(0, 3));
    plotBounds(W->getXYZ(app->ixyz_min), W->getXYZ(app->ixyz_max), app);
  }

  // check bounds
  if (W->checkMapBounds(app->ixyz_min)) {
    fprintf(stderr, "Error: Invalid min bounds.  Using map min bound.\n");
    app->ixyz_min = W->ixyz_min;
  }
  if (W->checkMapBounds(app->ixyz_max)) {
    fprintf(stderr, "Error: Invalid max bounds.  Using map max bound.\n");
    app->ixyz_max = W->ixyz_max;
  }

  return;
}

void Planner::freeMem(std::vector<std::vector<std::vector<Node*> > > * N,
                      FlowData* W, App * app) {
  // nodes
  // x
  for (int i = 0; i < app->x_steps.rows(); i++) {
    // y
    for (int j = 0; j < app->y_steps.rows(); j++) {
      // vel
      for (int k = 0; k < app->vel_g_steps.rows(); k++) {
        Eigen::Vector3i ixyz = Eigen::Vector3i::Zero();
        ixyz(0) = app->x_steps(i);
        ixyz(1) = app->y_steps(j);
        ixyz(2) = app->start_ixyz(2);
        // don't create node if in obstacle space
        if (W->getObs3D(ixyz)) {
          continue;
        }

        delete (*N)[i][j][k];
      }
    }
  }

  return;
}

int Planner::checkPlannerBounds(Eigen::Vector3i ixyz, FlowData * W, App * app) {
  int ret = 0;

  // loop over indicies
  for (int i = 0; i < 3; i++) {
    if (ixyz(i) < app->ixyz_min(i) || ixyz(i) > app->ixyz_max(i)) {
      ret = 1;
    }
  }

  if (W->checkMapBounds(ixyz)) {
    ret = 1;
  }

  return ret;
}

Node* Planner::createNode(Node* parent, const Eigen::Vector3i ixyz,
                          const double vel_g_mag, FlowData* W, App* app) {
  Node* n = new Node(parent, ixyz, vel_g_mag, W, app);
  return n;
}

Eigen::Vector3i Planner::getNodeInd(const Eigen::Vector4f state, App * app) {
  Eigen::Vector3i ind = Eigen::Vector3i::Zero();
  ind(0) = (state(0) - app->state_min(0)) / app->x_step;
  ind(1) = (state(1) - app->state_min(1)) / app->y_step;
  ind(2) = (state(3) - app->state_min(3)) / app->vel_g_step;

  return ind;
}

void Planner::getDState(App * app) {
  app->d_pos.setZero(3, 8);
  app->d_pos.col(0) << 1, 0, 0;  // forward
  app->d_pos.col(1) << -1, 0, 0;  // backward
  app->d_pos.col(2) << 0, 1, 0;  // right
  app->d_pos.col(3) << 0, -1, 0;  // left
  app->d_pos.col(4) << 1, 1, 0;  // forward-right
  app->d_pos.col(5) << 1, -1, 0;  // forward-left
  app->d_pos.col(6) << -1, 1, 0;  // backward-right
  app->d_pos.col(7) << -1, -1, 0;  // backward-left

  app->d_speed.setZero(3);
  app->d_speed << -0.5, 0, 0.5;  // possible speed deltas

  // x,y
  //  d_state.setZero(4, d_pos.cols());
  //  for (int i = 0; i < d_pos.cols(); i++) {
  //    d_state.col(i).segment(0, 3) = d_pos.col(i);
  //    d_state(3, i) = 0;
  //  }

  // x,y,vel
  app->d_state.setZero(4, app->d_pos.cols() * app->d_speed.rows());
  for (int i = 0; i < app->d_pos.cols(); i++) {
    for (int j = 0; j < app->d_speed.rows(); j++) {
      app->d_state.col(i * app->d_speed.rows() + j).segment(0, 3) = app->d_pos
          .col(i);
      app->d_state(3, i * app->d_speed.rows() + j) = app->d_speed(j);
    }
  }

  // eigen_dump(d_state);
  // fprintf(stderr, "d_state: %lu\n", d_state.cols());

  return;
}

void Planner::getSteps(App * app) {
  // x
  int x_size = (app->ixyz_max(0) - app->ixyz_min(0)) / app->x_step;
  app->x_steps.setZero(x_size + 1);
  for (int i = 0; i < app->x_steps.rows(); i++) {
    app->x_steps(i) = app->ixyz_min(0) + i * app->x_step;
  }

  // y
  int y_size = (app->ixyz_max(1) - app->ixyz_min(1)) / app->y_step;
  app->y_steps.setZero(y_size + 1);
  for (int i = 0; i < app->y_steps.rows(); i++) {
    app->y_steps(i) = app->ixyz_min(1) + i * app->y_step;
  }

  // z
  int z_size = (app->ixyz_max(2) - app->ixyz_min(2)) / app->z_step;
  app->z_steps.setZero(z_size + 1);
  for (int i = 0; i < app->z_steps.rows(); i++) {
    app->z_steps(i) = app->ixyz_min(2) + i * app->z_step;
  }

  // vel
  int vel_size = (app->vel_g_max - app->vel_g_min) / app->vel_g_step;
  app->vel_g_steps.setZero(vel_size + 1);
  for (int i = 0; i < app->vel_g_steps.rows(); i++) {
    app->vel_g_steps(i) = app->vel_g_min + i * app->vel_g_step;
  }

  return;
}

void Planner::initNodes(std::vector<std::vector<std::vector<Node*> > > * N,
                        FlowData* W, App* app) {
  if (!app->quiet && app->verbose) {
    fprintf(stderr, "initNodes... ");
  }

  // init node array
  (*N).resize(app->x_steps.rows());
  for (int i = 0; i < app->x_steps.rows(); ++i) {
    (*N)[i].resize(app->y_steps.rows());

    for (int j = 0; j < app->y_steps.rows(); ++j)
      (*N)[i][j].resize(app->vel_g_steps.rows());
  }

  // eigen_dump(z_steps);
  // fprintf(stderr, "z: %i\n", z_steps(start_ixyz(2)));

  Eigen::Vector3i ixyz = Eigen::Vector3i::Zero();
  double speed = 0;
  int node_count = 0;
  // x
  for (int i = 0; i < app->x_steps.rows(); i++) {
    // y
    for (int j = 0; j < app->y_steps.rows(); j++) {
      // vel
      for (int k = 0; k < app->vel_g_steps.rows(); k++) {
        ixyz(0) = app->x_steps(i);
        ixyz(1) = app->y_steps(j);
        ixyz(2) = app->start_ixyz(2);
        speed = app->vel_g_steps(k);

        // don't create node if in obstacle space
        if (W->getObs3D(ixyz)) {
          continue;
        }

        node_count++;

        Node* n = createNode(NULL, ixyz, speed, W, app);

        n->g = std::numeric_limits<double>::infinity();
        n->dist = std::numeric_limits<double>::infinity();
        n->energy = std::numeric_limits<double>::infinity();
        n->energy_sto = std::numeric_limits<double>::infinity();
        n->energy_pot = std::numeric_limits<double>::infinity();
        (*N)[i][j][k] = n;
        // Q.push(n);
      }
    }
  }

  // fprintf(stderr, "Total Nodes: %i\n", node_count);
  if (!app->quiet && app->verbose) {
    fprintf(stderr, "Done\n");
  }

  return;
}

void Planner::getPath(std::list<Node*> * path,
                      std::vector<std::vector<std::vector<Node*> > > * N,
                      FlowData* W, App* app) {
  if (!app->quiet && app->verbose) {
    fprintf(stderr, "Getting path... ");
  }

  // reset start node
  Eigen::Vector3i start_ind = getNodeInd(app->start_state, app);
  Node* n_start = (*N)[start_ind(0)][start_ind(1)][start_ind(2)];
  n_start->parent = NULL;
  n_start->g = 0;

  // get goal node
  Eigen::Vector3i goal_ind = getNodeInd(app->goal_state, app);
  Node* n = (*N)[goal_ind(0)][goal_ind(1)][goal_ind(2)];

  while (n != NULL) {
    // n->print();
    path->push_front(n);
    n = n->parent;

    if (n->state == app->start_state) {
      path->push_front(n);
      break;
    }
  }

  // fprintf(stderr, "path size: %lu\n", P.size());
  if (!app->quiet && app->verbose) {
    fprintf(stderr, "Done\n");
  }

  return;
}

void Planner::printPath(std::list<Node*> * path, App * app) {
  std::list<Node*>::iterator it = --path->end();

  fprintf(stderr, "Total Path Energy: %f\n", (*it)->energy_true);
  fprintf(stderr, "Total Path Length: %f\n", (*it)->dist);

  return;
}

void Planner::dumpPath(std::list<Node*> * path, std::string path_name,
                       App * app) {
  Eigen::MatrixXd dump;
  dump.setZero(path->size(), 22);

  int i = 0;
  for (std::list<Node*>::iterator it = path->begin(); it != path->end(); it++) {
    if (!app->naive_flag) {
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
      dump(i, 21) = (*it)->g;
    } else {
      // 3 vectors
      for (int j = 0; j < 3; j++) {
        dump(i, j) = (*it)->ixyz(j);
        dump(i, j + 3) = (*it)->pos(j);
        dump(i, j + 6) = (*it)->vel_f(j);
        dump(i, j + 9) = (*it)->vel_g(j);
        dump(i, j + 12) = (*it)->vel_a_true(j);
      }

      // cost terms
      dump(i, 15) = (*it)->vel_a_mag_true;
      dump(i, 16) = (*it)->vel_g_mag;
      dump(i, 17) = (*it)->energy_pot_true;
      dump(i, 18) = (*it)->energy_sto_true;
      dump(i, 19) = (*it)->energy_true;
      dump(i, 20) = (*it)->dist;
      dump(i, 21) = (*it)->g_true;
    }

    i++;
  }

  std::ofstream myfile;
  char path_num_cstr[6];
  snprintf(path_num_cstr, sizeof(path_num_cstr), "%05i", app->path_num);

  std::string wind_str = "data";
  if (app->filename_flag) {
    int start_ind = app->filename.find("/", 32) + 1;
    int end_ind = app->filename.find("/", start_ind);
    // fprintf(stderr, "start_ind: %i\n", start_ind);
    // fprintf(stderr, "end_ind: %i\n", end_ind);
    // fprintf(stderr, "%s\n",
    // app->filename.substr(start_ind, end_ind-start_ind).c_str());
    wind_str = app->filename.substr(start_ind, end_ind - start_ind);
  }

  char naive_flag_cstr[5];
  snprintf(naive_flag_cstr, sizeof(naive_flag_cstr), "%i", !app->naive_flag);

  char vela_flag_cstr[5];
  snprintf(vela_flag_cstr, sizeof(vela_flag_cstr), "%i", app->vel_a_cap_flag);

  std::string filename = wind_str + "_w"
      + std::string(naive_flag_cstr) + "_a" + std::string(vela_flag_cstr) + "_"
      + std::string(path_num_cstr) + ".m";
  std::cout << filename + "\n";
  myfile.open(filename.c_str());
  myfile << "path" << "=[" << (dump) << "];\n";
  myfile.close();

  // eigen_matlab_dump(dump);

  return;
}

// void Planner::publishPath(std::list<Node*> * path, App * app) {
//   srand(time(NULL));

//   quad_waypoint_list_t list_msg;
//   list_msg.utime = bot_timestamp_now();
//   list_msg.num_waypoints = path->size() + 1;
//   list_msg.waypoints = new quad_waypoint_t[path->size() + 1];

//   double psi = 0;  // heading
//   double psi_last = 0;
//   Eigen::Vector3f pos_last = Eigen::Vector3f::Zero();
//   for (std::list<Node*>::iterator it = path->begin(); it != path->end(); it++) {
//     Eigen::Vector3f diff = ((*it)->pos - pos_last).normalized();
//     psi = atan2(static_cast<double>(diff(1)), static_cast<double>(diff(0)));

//     quad_waypoint_t waypoint_msg;
//     waypoint_msg.utime = bot_timestamp_now();
//     waypoint_msg.xyzt[0] = (*it)->pos(0) - app->start_pos(0);
//     waypoint_msg.xyzt[1] = (*it)->pos(1) - app->start_pos(1);
//     waypoint_msg.xyzt[2] = (*it)->pos(2);
//     waypoint_msg.xyzt[3] = psi;

//     waypoint_msg.xyzt_dot[0] = (*it)->vel_g(0);
//     waypoint_msg.xyzt_dot[1] = (*it)->vel_g(1);
//     waypoint_msg.xyzt_dot[2] = (*it)->vel_g(2);
//     waypoint_msg.xyzt_dot[3] = 0;

//     waypoint_msg.waypt_type = QUAD_WAYPOINT_T_TYPE_WAYPT;
//     waypoint_msg.sender = QUAD_WAYPOINT_T_SENDER_MISSION_PLANNER;
//     waypoint_msg.nonce = rand_r(&app->seed);

//     list_msg.waypoints[distance(path->begin(), it)] = waypoint_msg;

//     pos_last = (*it)->pos;
//     psi_last = psi;
//   }

//   // append zero velocity waypoint at goal
//   quad_waypoint_t waypoint_msg;
//   waypoint_msg.utime = bot_timestamp_now();
//   waypoint_msg.xyzt[0] = pos_last(0) - app->start_pos(0);
//   waypoint_msg.xyzt[1] = pos_last(1) - app->start_pos(1);
//   waypoint_msg.xyzt[2] = pos_last(2);
//   waypoint_msg.xyzt[3] = psi_last;

//   waypoint_msg.xyzt_dot[0] = 0;
//   waypoint_msg.xyzt_dot[1] = 0;
//   waypoint_msg.xyzt_dot[2] = 0;
//   waypoint_msg.xyzt_dot[3] = 0;

//   waypoint_msg.waypt_type = QUAD_WAYPOINT_T_TYPE_WAYPT;
//   waypoint_msg.sender = QUAD_WAYPOINT_T_SENDER_MISSION_PLANNER;
//   waypoint_msg.nonce = rand_r(&app->seed);

//   list_msg.waypoints[path->size()] = waypoint_msg;

//   quad_waypoint_list_t_publish(app->lcm, "PLANNED_WAYPOINTS", &list_msg);

//   delete list_msg.waypoints;

//   return;
// }
