// Copyright 2015 Jake Ware

// c system includes

// cpp system includes

// external includes

// project includes
#include "astar/astar.h"

SearchAS::SearchAS(std::vector<std::vector<std::vector<Node*> > > * _N) {
  // copy node array pointer
  N = _N;
}

SearchAS::~SearchAS() {
  // Nothing
}

void SearchAS::initStart(FlowData * W, App * app) {
  if (!app->quiet && app->verbose) {
    fprintf(stderr, "initStart... ");
  }

  // get node index in node array
  Eigen::Vector3i start_ind = getNodeInd(app->start_state, app);

  // get start ixyz from wind field
  Eigen::Vector3f start_pos = app->start_state.segment(0, 3);
  Eigen::Vector3i start_ixyz = W->getInd(start_pos);

  // check if start location is free
  if (W->getObs3D(start_ixyz)
      || checkPlannerBounds(start_ixyz, W, app)) {
    fprintf(stderr, "Error: Invalid Start Position\n");
    eigen_dump(start_ixyz);
  }

  Node* n_start = (*N)[start_ind(0)][start_ind(1)][start_ind(2)];

  n_start->parent = NULL;
  n_start->g = 0;
  n_start->h = (app->goal_state.segment(0, 3) - n_start->pos).norm();
  n_start->f = n_start->g + n_start->h;
  n_start->dist = 0;
  n_start->energy = 0;
  n_start->energy_pot = 0;
  n_start->energy_sto = 0;
  Q.push(n_start);

  if (!app->quiet && app->verbose) {
    fprintf(stderr, " Done\n");
  }

  return;
}

int SearchAS::checkPlannerBounds(Eigen::Vector3i ixyz, FlowData * W,
                                  App * app) {
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

std::vector<Node*> SearchAS::getNeighbors(Node* u, FlowData* W, App* app) {
  // fprintf(stderr, "getNeighbors\n");

  std::vector<Node*> L;
  Eigen::Vector4f state = Eigen::Vector4f::Zero();
  Eigen::Vector3i ind = Eigen::Vector3i::Zero();
  for (int i = 0; i < u->valid.rows(); i++) {
    // fprintf(stderr, "i: %i\n", i);

    if (u->valid(i) == 1) {
      state = u->state + app->d_state.col(i);
      // eigen_dump(state);

      ind = getNodeInd(state, app);
      // eigen_dump(ind);

      Node* n = (*N)[ind(0)][ind(1)][ind(2)];
      // n->print();

      // check constraints on new node
      if (!app->naive_flag && n->getVelAMagTrue(u, W) > app->vel_a_max) {
        u->valid(i) = 0;
      }

      if (!n->visited && u->valid(i)) {
        L.push_back(n);
      }

      // fprintf(stderr, "L: %lu\n", L.rows());
    }
  }

  return L;
}

// TODO(jakeware): Change this so that we can handle non-uniform grid spacing
// in z-axis
Eigen::Vector3i SearchAS::getNodeInd(const Eigen::Vector4f state, App * app) {
  Eigen::Vector3i ind = Eigen::Vector3i::Zero();
  ind(0) = (state(0) - app->state_min(0)) / app->x_step;
  ind(1) = (state(1) - app->state_min(1)) / app->y_step;
  ind(2) = (state(3) - app->state_min(3)) / app->vel_g_step;

  return ind;
}

int SearchAS::getGraph(FlowData * W, App * app) {
  if (!app->quiet && app->verbose) {
    fprintf(stderr, "Getting graph...\n");
  }

  // initialize nodes
  initStart(W, app);

  int count = 0;
  while (!Q.empty()) {
    // fprintf(stderr, "count: %i\n", count);
    Node * u = Q.top();  // get node with lowest f
    Q.pop();  // remove node

    // u->print();
    // plotPos(u->pos, "node_pos", app);
    // std::cin.ignore();

    // are we in goal region?
    if ((app->goal_state - u->state).norm() < 0.1) {
      return 0;
    } else {
      std::vector<Node*> L = getNeighbors(u, W, app);
      // fprintf(stderr, "neighbors: %lu\n", L.size());
      for (int i = 0; i < L.size(); i++) {
        Node* v = L[i];
        // v->print();
        v->parent = u;
        v->setNode(u, W, app);
        Q.push(v);
        v->visited = true;
      }
    }

    count++;
  }

  return 1;
}
