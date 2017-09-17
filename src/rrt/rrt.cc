// Copyright 2015 Jake Ware

// INCLUDES
#include "rrt.h"

NodeRRT::NodeRRT(NodeRRT* _parent, Vector3i _ixyz, double _vel_g_mag,
                 FlowData &W) {
  // get parent
  parent = _parent;

  // get values
  ixyz = _ixyz;
  pos = W.getXYZ(ixyz);
  vel_g_mag = _vel_g_mag;
  vel_f = W.getUVW(ixyz);

  getVels();
  fillState();
}

NodeRRT::~NodeRRT() {
  // Nothing
}

void NodeRRT::fillState() {
  state(0) = pos(0);
  state(1) = pos(1);
  state(2) = pos(2);
  state(3) = vel_g_mag;

  return;
}

void NodeRRT::getVels() {
  if (parent != NULL) {
    // get direction
    vel_g = ((pos - parent->pos).normalized()) * vel_g_mag;
    vel_a = vel_g - vel_f;
    vel_a_mag = vel_a.norm();
  } else {
    vel_g.setZero();
    vel_a = -vel_f;
    vel_a_mag = vel_f.norm();
  }

  return;
}

void NodeRRT::print() {
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

void NodeRRT::node2Flann(flann::Matrix<double> &nf) {
  nf[0][0] = this->pos(0);
  nf[0][1] = this->pos(1);
  nf[0][2] = this->pos(2);

  return;
}

PlannerRRT::PlannerRRT(Vector3i _sxyz, Vector3i _gxyz, FlowData &W, App &app) {
  fprintf(stderr, "Starting planner...\n");

  // seed random number generator
  srand(time(NULL));

  // initialize start and goal states
  start_ixyz = _sxyz;
  start_pos = W.getXYZ(start_ixyz);
  start_state.segment(0,3) = start_pos;
  start_state(3) = 0;
  goal_ixyz = _gxyz;
  goal_pos = W.getXYZ(goal_ixyz);
  goal_state.segment(0,3) = goal_pos;
  goal_state(3) = 0;

  ixyz_min << 120,150,10;  //0, 0, 2;
  ixyz_max << 280,280,10;  //W.dim_x, W.dim_y, 15;
  vel_g_min = 0.5;  // minimum ground speed [m/s]
  vel_g_max = 10;  // maximum ground speed [m/s]
  vel_g_step = 0.5;  // speed discretization [m/s]
  vel_a_max = 15;  // maximum air speed [m/s]
  P.setZero(W.dim_x, W.dim_y);

  fprintf(stderr, "Start: %f %f %f\n", start_pos(0), start_pos(1), start_pos(2));
  fprintf(stderr, "Goal: %f %f %f\n", goal_pos(0), goal_pos(1), goal_pos(2));

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
  plotPath(app);
  //dumpPath(app);  // dump path data
}

PlannerRRT::~PlannerRRT() {
  // Nothing
}

int PlannerRRT::checkFree(Vector3i ixyz, FlowData &W) {
  // check occ
  if (W.getObs(ixyz) == 1 || W.getOcc(ixyz) == 1) {
    return 0;
  }

  return 1;
}

int PlannerRRT::checkRand(Vector3i ixyz, FlowData &W) {
  // check occ
  if (W.getObs(ixyz) == 1 || W.getOcc(ixyz) == 1 || P(ixyz(0),ixyz(1)) == 1) {
    return 0;
  }

  return 1;
}

NodeRRT* PlannerRRT::randNode(FlowData &W) {
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

    free = checkRand(samp_ixyz, W);
  }

  // sample velocity
  int vels = int((vel_g_max - vel_g_min) / vel_g_step);
  double vel_samp = (double(rand() % vels) * vel_g_step) + vel_g_min;

  // create node
  NodeRRT* n = createNode(NULL, samp_ixyz, vel_samp, W);

  return n;
}

NodeRRT* PlannerRRT::createNode(NodeRRT* parent, Vector3i ixyz,
                                double vel_g_mag, FlowData &W) {
  NodeRRT* n = new NodeRRT(parent, ixyz, vel_g_mag, W);

  return n;
}

Vector3i PlannerRRT::pos2Index(Vector3d pos) {
  Vector3i ind;
  ind(0) = (int)floor(double(pos(0)));
  ind(1) = (int)floor(double(pos(1)));
  ind(2) = (int)ceil(double(pos(2)));

  return ind;
}

int PlannerRRT::checkEdge(Vector3d pos1, Vector3d pos2, FlowData &W) {
  // get line between the two
  Vector3d dir = (pos2 - pos1).normalized();
  double dist = (pos2 - pos1).norm();
  int dist_steps = (int)floor(dist);
  Vector3d test;
  Vector3i ind;

  // test along line
  for (int i = 0; i < dist_steps + 2; i++) {
    test = pos1 + double(i)/double(dist_steps)*dist*dir;
    ind = pos2Index(test);

    if (checkFree(ind, W) == 0) {
      return 0;
    }
  }

  return 1;
}

int PlannerRRT::findNearest(NodeRRT* n_rand,
                                 flann::Index<flann::L2<double> > &kdtree, FlowData &W, App &app) {
  // find nearest neighbors
  flann::Matrix<double> query(new double[3], 1, 3);
  n_rand->node2Flann(query);

  int nn = 5;
  std::vector<std::vector<int> > indices;
  std::vector<std::vector<double> > dists;
  kdtree.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
  delete [] query.ptr();

  // check points
  NodeRRT *n_near;
  int treeID = 0;
  Vector4d state;
  for (vector<vector<int> >::iterator it1 = indices.begin(); it1 != indices.end(); it1++) {
    for (vector<int>::iterator it2 = (*it1).begin(); it2 != (*it1).end(); it2++) {
      treeID = *it2;
      //plotEdge(n_rand->pos, V[treeID]->pos, app);

      // check for collision
      if (checkEdge(n_rand->pos, V[treeID]->pos, W)) {
        //fprintf(stderr, "no collisions\n");
        return treeID;
      }
    }
  }

  return -1;
}

NodeRRT * PlannerRRT::newNode(NodeRRT* n_near, NodeRRT* n_rand, FlowData &W) {
  Vector3d pos_dir = (n_rand->pos - n_near->pos).normalized();
  double pos_dist = (n_rand->pos - n_near->pos).norm();
  Vector3d pos = n_near->pos + 0.5*pos_dist*pos_dir;
  double speed_dist = fabs(n_rand->vel_g_mag - n_near->vel_g_mag);
  double speed_dir = (n_rand->vel_g_mag - n_near->vel_g_mag)/speed_dist;
  double speed = n_near->vel_g_mag + 0.5*speed_dist*speed_dir;

  NodeRRT* n_new = createNode(n_near, pos2Index(pos), speed, W);

  return n_new;
}

void PlannerRRT::addNode(NodeRRT* n, flann::Index<flann::L2<double> > &kdtree) {
  // add to node list
  V.push_back(n);
  n->treeID = V.size();

  // add to kdtree
  flann::Matrix<double> p(new double[3], 1, 3);
  n->node2Flann(p);
  kdtree.addPoints(p);

  // mark visited location
  P(n->ixyz(0), n->ixyz(1)) = 1;

  //delete [] p.ptr();

  return;
}

void PlannerRRT::plotSG(App &app) {
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

void PlannerRRT::plotNodePos(NodeRRT* n, App &app) {
  bot_lcmgl_point_size(app.lcmgl_node_pos, 5);
  bot_lcmgl_begin(app.lcmgl_node_pos, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_node_pos, 0, 0, 0);
  bot_lcmgl_vertex3d(app.lcmgl_node_pos, double(n->pos(0)), double(n->pos(1)),
                     double(n->pos(2)));
  bot_lcmgl_end(app.lcmgl_node_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_node_pos);
}

void PlannerRRT::plotRandPos(NodeRRT* n, App &app) {
  bot_lcmgl_point_size(app.lcmgl_rand_pos, 5);
  bot_lcmgl_begin(app.lcmgl_rand_pos, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_rand_pos, 1, 1, 0);
  bot_lcmgl_vertex3d(app.lcmgl_rand_pos, double(n->pos(0)), double(n->pos(1)),
                     double(n->pos(2)));
  bot_lcmgl_end(app.lcmgl_rand_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_rand_pos);
}

void PlannerRRT::plotNearPos(NodeRRT* n, App &app) {
  bot_lcmgl_point_size(app.lcmgl_node_pos, 5);
  bot_lcmgl_begin(app.lcmgl_node_pos, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_node_pos, 0, 1, 1);
  bot_lcmgl_vertex3d(app.lcmgl_node_pos, double(n->pos(0)), double(n->pos(1)),
                     double(n->pos(2)));
  bot_lcmgl_end(app.lcmgl_node_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_node_pos);
}

void PlannerRRT::plotNewPos(NodeRRT* n, App &app) {
  bot_lcmgl_point_size(app.lcmgl_node_pos, 5);
  bot_lcmgl_begin(app.lcmgl_node_pos, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_node_pos, 1, 0, 1);
  bot_lcmgl_vertex3d(app.lcmgl_node_pos, double(n->pos(0)), double(n->pos(1)),
                     double(n->pos(2)));
  bot_lcmgl_end(app.lcmgl_node_pos);
  bot_lcmgl_switch_buffer(app.lcmgl_node_pos);
}

void PlannerRRT::plotEdge(Vector3d pos1, Vector3d pos2, App &app) {
  bot_lcmgl_line_width(app.lcmgl_col_edge, 2);
  bot_lcmgl_begin(app.lcmgl_col_edge, GL_LINES);
  bot_lcmgl_color3f(app.lcmgl_col_edge, 0, 0, 0.5);
  bot_lcmgl_line3d(app.lcmgl_col_edge, double(pos1(0)), double(pos1(1)), double(pos1(2)), double(pos2(0)), double(pos2(1)), double(pos2(2)));
  bot_lcmgl_end(app.lcmgl_col_edge);
  bot_lcmgl_switch_buffer(app.lcmgl_col_edge);
}

void PlannerRRT::plotGraph(App &app) {
  bot_lcmgl_line_width(app.lcmgl_graph_edges, 2);
  bot_lcmgl_begin(app.lcmgl_graph_edges, GL_LINES);
  bot_lcmgl_color3f(app.lcmgl_graph_edges, 0, 0.5, 0.5);
  for (vector<NodeRRT*>::iterator it = V.begin()+2; it != V.end(); it++) {
    bot_lcmgl_line3d(app.lcmgl_graph_edges, double((*it)->parent->pos(0)), double((*it)->parent->pos(1)), double((*it)->parent->pos(2)), double((*it)->pos(0)), double((*it)->pos(1)), double((*it)->pos(2)));
  }
  bot_lcmgl_end(app.lcmgl_graph_edges);
  bot_lcmgl_switch_buffer(app.lcmgl_graph_edges);

  bot_lcmgl_point_size(app.lcmgl_graph_nodes, 3);
  bot_lcmgl_begin(app.lcmgl_graph_nodes, GL_POINTS);
  bot_lcmgl_color3f(app.lcmgl_graph_edges, 0, 0.5, 0.5);
  for (vector<NodeRRT*>::iterator it = V.begin()+1; it != V.end(); it++) {
    bot_lcmgl_vertex3d(app.lcmgl_graph_nodes, double((*it)->pos(0)), double((*it)->pos(1)), double((*it)->pos(2)));
  }
  bot_lcmgl_end(app.lcmgl_graph_nodes);
  bot_lcmgl_switch_buffer(app.lcmgl_graph_nodes);

  return;
}

void PlannerRRT::plotPathNode(App &app) {
  bot_lcmgl_point_size(app.lcmgl_path_nodes, 3);
  bot_lcmgl_begin(app.lcmgl_path_nodes, GL_POINTS);
  for (list<NodeRRT*>::iterator it = path.begin(); it != path.end(); it++) {
    bot_lcmgl_color3f(app.lcmgl_path_nodes, 0, 0, 0);
    bot_lcmgl_vertex3d(app.lcmgl_path_nodes, double((*it)->pos(0)), double((*it)->pos(1)), double((*it)->pos(2)));
  }
  bot_lcmgl_end(app.lcmgl_path_nodes);
  bot_lcmgl_switch_buffer(app.lcmgl_path_nodes);
}

void PlannerRRT::plotPathEdge(App &app) {
  bot_lcmgl_line_width(app.lcmgl_path_edges, 2);
  bot_lcmgl_begin(app.lcmgl_path_edges, GL_LINES);
  for (list<NodeRRT*>::iterator it = path.begin(); it != path.end(); it++) {
    if (it == path.begin()) continue;

    bot_lcmgl_color3f(app.lcmgl_path_edges, 0, 1, 0);
    bot_lcmgl_line3d(app.lcmgl_path_edges, double((*it)->parent->pos(0)), double((*it)->parent->pos(1)), double((*it)->parent->pos(2)), double((*it)->pos(0)), double((*it)->pos(1)), double((*it)->pos(2)));
  }
  bot_lcmgl_end(app.lcmgl_path_edges);
  bot_lcmgl_switch_buffer(app.lcmgl_path_edges);
}

void PlannerRRT::getPath(NodeRRT* n) {
  fprintf(stderr, "Getting path...\n");
  //n->print();

  while (n != NULL) {
    //n.print();
    path.push_front(n);
    n = n->parent;
  }

  return;
}

void PlannerRRT::plotPath(App &app) {
  fprintf(stderr, "Plotting path...\n");

  plotPathNode(app);
  plotPathEdge(app);

  return;
}

int PlannerRRT::planPath(FlowData &W, App &app) {
  // create start node
  NodeRRT* n_start = createNode(NULL, start_ixyz, 0, W);
  flann::Matrix<double> p_start(new double[3], 1, 3);
  n_start->node2Flann(p_start);
  V.push_back(n_start);

  // create tree
  flann::Index<flann::L2<double> > kdtree(p_start, flann::KDTreeSingleIndexParams(4));
  kdtree.removePoint(0);
  addNode(n_start, kdtree);

  for (int i = 0; i < 100000; i++) {
    // random node
    NodeRRT* n_rand = randNode(W);
    //plotRandPos(n_rand, app);

    // find nearest node using FLANN
    int treeID = findNearest(n_rand, kdtree, W, app);

    // did we find any valid points?
    if (treeID >= 0) {
      NodeRRT* n_near = V[treeID];
      //plotNearPos(n_near, app);

      // new node
      //NodeRRT* n_new = newNode(n_near, n_rand, W);  // create node using some function
      NodeRRT* n_new = createNode(n_near, n_rand->ixyz, n_rand->vel_g_mag, W);  // just take random sample
      addNode(n_new, kdtree);
      //plotNewPos(n_new, app);

      if ((goal_pos - n_new->pos).norm() < 2) {  // did we find the goal?
        fprintf(stderr, "Found goal...\n");
        getPath(n_new);

        return 0;
      }
    }
  }

  return 1;
}
