// Copyright 2015 Jake Ware

// c system includes
#include <getopt.h>

// cpp system includes
#include <string>
#include <list>

// external library includes
#include "eigen3/Eigen/Dense"
#include "bot_lcmgl_client/lcmgl.h"
#include "eigen_utils/eigen_utils.hpp"

// project includes
#include "flow_data/flow_data.h"
#include "utils/plot_utils.h"
#include "node/node.h"

// TODO(jakeware) on Startup
// Set map bounds
// Set start and goal points
// Set wind flag
// Set air speed cap flag

void usage(const char *progname) {
  char *basename = g_path_get_basename(progname);
  printf("Usage: %s [options]\n"
         "\n"
         "Options:\n"
         "\n"
         "    -h, --help                Shows this help text and exits\n"
         "    -v, --verbose             Enables additional text output\n"
         "    -q, --quiet               Suppresses text output\n"
         "    -t, --traj                Specifies path of trajectory files "
         "(./path/to/files/)\n"
         "    -w, --wind                Specifies path of wind field files "
         "(./path/to/files/)\n"
         "\n",
         basename);
  free(basename);
  exit(1);
}

FlowData* getFlowField(std::string dataset_str, std::string path_str,
                       bool quiet, bool verbose) {
  if (!quiet && verbose) {
    fprintf(stderr, "Getting Wind Field... \n");
  }

  // get data
  const char *path_str_c = path_str.c_str();

  // get datasets
  const char *dataset_str_c = dataset_str.c_str();

  // fprintf(stderr, "Done loading data\n");
  FlowData* W = new FlowData(path_str_c, dataset_str_c, quiet, verbose);

  if (!quiet && verbose) {
    fprintf(stderr, "Done\n");
  }

  return W;
}

std::list<Node*> readPath(FlowData * W, std::string trajpath,
                          Eigen::Vector3i *start_ixyz,
                          Eigen::Vector3i *goal_ixyz) {
  //  node data structure:
  //  1-3: index
  //  4-6: pos
  //  7-9: vel_f
  //  10-12: vel_g
  //  13-15: vel_a
  //  16: vel_a_mag
  //  17: vel_g_mag
  //  18: energy_pot
  //  19: energy_sto
  //  20: energy
  //  21: dist
  //  22: cost

  std::string line;
  std::ifstream myfile(trajpath.c_str());
  std::string delimiter = " ";
  std::list<Node*> path;
  if (myfile.is_open()) {
    // count number of lines
    std::string unused;
    int line_count = 0;
    while (std::getline(myfile, unused))
      ++line_count;

    // clear flag and go back to beginning
    myfile.clear();
    myfile.seekg(0, std::ios::beg);

    // read data

    while (getline(myfile, line)) {
      Node* n = new Node(NULL, Eigen::Vector3i::Zero(), 0.0, W);

      size_t pos = 0;
      std::string token;
      int i = 0;
      while ((pos = line.find(delimiter)) != std::string::npos) {
        token = line.substr(0, pos);
        if (!token.empty() && token.compare("path=[")) {
          double val = atof(token.c_str());

          switch (i) {
            case 0:  // x index
              n->ixyz(0) = val;
              break;
            case 1:  // y index
              n->ixyz(1) = val;
              break;
            case 2:  // z index
              n->ixyz(2) = val;
              break;
            case 3:  // x pos
              n->pos(0) = val;
              break;
            case 4:  // y pos
              n->pos(1) = val;
              break;
            case 5:  // z pos
              n->pos(2) = val;
              break;
            case 6:  // x vel_f
              n->vel_f(0) = val;
              break;
            case 7:  // y vel_f
              n->vel_f(1) = val;
              break;
            case 8:  // z vel_f
              n->vel_f(2) = val;
              break;
            case 9:  // x vel_g
              n->vel_g(0) = val;
              break;
            case 10:  // y vel_g
              n->vel_g(1) = val;
              break;
            case 11:  // z vel_g
              n->vel_g(2) = val;
              break;
            case 12:  // x vel_a
              n->vel_a(0) = val;
              break;
            case 13:  // y vel_a
              n->vel_a(1) = val;
              break;
            case 14:  // z vel_a
              n->vel_a(2) = val;
              break;
            case 15:  // vel_a_mag
              n->vel_a_mag = val;
              break;
            case 16:  // vel_g_mag
              n->vel_g_mag = val;
              break;
            case 17:  // energy_pot
              n->energy_pot = val;
              break;
            case 18:  // energy_sto
              n->energy_sto = val;
              break;
            case 19:  // energy
              n->energy = val;
              break;
            case 20:  // dist
              n->dist = val;
              break;
            case 21:  // cost
              n->g = val;
              break;
            default:
              fprintf(stderr, "Invalid value while reading path: %i\n", i);
          }

          i++;  // increment value index
        }
        line.erase(0, pos + delimiter.length());
      }
      path.push_back(n);
    }
    myfile.close();
  } else {
    std::cout << "Unable to open file";
  }

  // get start and goal
  std::list<Node*>::iterator it;
  it = path.begin();
  (*start_ixyz) = (*it)->ixyz;
  it = --path.end();
  (*goal_ixyz) = (*it)->ixyz;

  return path;
}

void plotPath(std::list<Node*> path, FlowData * W, Eigen::Vector3f start_pos,
              bool quiet, bool verbose) {
  if (!quiet && verbose) {
    fprintf(stderr, "Plotting Path... \n");
  }

  lcm_t * lcm = bot_lcm_get_global(NULL);
  std::string name = "test_path";
  bot_lcmgl_t * lcmgl_path_edges = bot_lcmgl_init(lcm, name.c_str());

  bot_lcmgl_line_width(lcmgl_path_edges, 7);
  bot_lcmgl_begin(lcmgl_path_edges, GL_LINES);
  for (std::list<Node*>::iterator it = path.begin(); it != path.end(); it++) {
    if (it == path.begin())
      continue;

    std::list<Node*>::iterator it_last = it;
    it_last--;

    // get translated positions
    Eigen::Vector3f pos = (*it)->pos;
    pos.segment(0, 2) -= start_pos.segment(0, 2);
    Eigen::Vector3f last_pos = (*it_last)->pos;
    last_pos.segment(0, 2) -= start_pos.segment(0, 2);

    // TODO(jakeware): don't hardcode vel_g_mag_max
    float * color = bot_color_util_jet((*it)->vel_g_mag / 8.0);
    bot_lcmgl_color3f(lcmgl_path_edges, color[0], color[1], color[2]);
    bot_lcmgl_line_3d(lcmgl_path_edges, static_cast<double>(last_pos(0)),
                      static_cast<double>(last_pos(1)),
                      static_cast<double>(last_pos(2)),
                      static_cast<double>(pos(0)), static_cast<double>(pos(1)),
                      static_cast<double>(pos(2)));
  }

  bot_lcmgl_end(lcmgl_path_edges);

  bot_lcmgl_switch_buffer(lcmgl_path_edges);

  if (!quiet && verbose) {
    fprintf(stderr, "Done\n");
  }
  return;
}

void plotPathGlobal(std::list<Node*> path, FlowData * W, bool quiet,
                    bool verbose) {
  if (!quiet && verbose) {
    fprintf(stderr, "Plotting Path... \n");
  }

  lcm_t * lcm = bot_lcm_get_global(NULL);
  std::string name = "test_path";
  bot_lcmgl_t * lcmgl_path_edges = bot_lcmgl_init(lcm, name.c_str());

  bot_lcmgl_line_width(lcmgl_path_edges, 7);
  bot_lcmgl_begin(lcmgl_path_edges, GL_LINES);
  for (std::list<Node*>::iterator it = path.begin(); it != path.end(); it++) {
    if (it == path.begin())
      continue;

    std::list<Node*>::iterator it_last = it;
    it_last--;

    // get translated positions
    Eigen::Vector3f pos = (*it)->pos;
    Eigen::Vector3f last_pos = (*it_last)->pos;

    // TODO(jakeware): don't hardcode vel_g_mag_max
    float * color = bot_color_util_jet((*it)->vel_g_mag / 8.0);
    bot_lcmgl_color3f(lcmgl_path_edges, color[0], color[1], color[2]);
    bot_lcmgl_line_3d(lcmgl_path_edges, static_cast<double>(last_pos(0)),
                      static_cast<double>(last_pos(1)),
                      static_cast<double>(last_pos(2)),
                      static_cast<double>(pos(0)), static_cast<double>(pos(1)),
                      static_cast<double>(pos(2)));
  }

  bot_lcmgl_end(lcmgl_path_edges);

  bot_lcmgl_switch_buffer(lcmgl_path_edges);

  if (!quiet && verbose) {
    fprintf(stderr, "Done\n");
  }
  return;
}

void plotWind(FlowData * W, Eigen::Vector3i start_ixyz, bool quiet,
              bool verbose) {
  if (!quiet && verbose) {
    fprintf(stderr, "Plotting Wind... \n");
  }

  if (W->data_flags(7)) {
    // W->plotOcc(app->start_state);
    W->plotObs2D();
    // W->plotObs3D();
  }

  if (W->data_flags(4) && W->data_flags(5) && W->data_flags(6)) {
    fprintf(stderr, "Plotting Wind\n");
    W->plotUVGlobal(static_cast<int>(start_ixyz(2)), 1.0, 1);
  }

  if (!quiet && verbose) {
    fprintf(stderr, "Done\n");
  }

  return;
}

int main(int argc, char **argv) {
  bool quiet;
  bool verbose;
  Eigen::Vector3i start_ixyz;
  Eigen::Vector3f start_pos;
  Eigen::Vector3i goal_ixyz;
  Eigen::Vector3f goal_pos;

  // set defaults
  std::string traj_path_str = "./data/";
  std::string wind_path_str = "./data/";
  std::string dataset_str = "DSM";
  bool dump_flag = false;
  bool test_flag = false;

  // parse options
  const char *optstring = "hvqt:w:dp";
  struct option long_opts[] = { { "help", no_argument, 0, 'h' }, { "verbose",
  no_argument, 0, 'v' }, { "quiet", no_argument, 0, 'q' }, { "traj",
  required_argument, 0, 't' }, { "wind", required_argument, 0, 'w' }, { "dump",
  no_argument, 0, 'd' }, { "point",
  no_argument, 0, 'p' }, { 0, 0, 0, 0 } };
  int c;
  while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
    switch (c) {
      case 'h':
        usage(argv[0]);
        break;
      case 'v':
        verbose = true;
        break;
      case 'q':
        quiet = true;
        break;
      case 't':
        if (optarg != NULL) {
          traj_path_str = std::string(optarg);
        }
        break;
      case 'w':
        if (optarg != NULL) {
          wind_path_str = std::string(optarg);
        }
        break;
      case 'd':
        dump_flag = true;
        break;
      case 'p':
        test_flag = true;
        break;
      default:
        usage(argv[0]);
        break;
    }
  }

  // invalid options
  if (optind < argc - 1) {
    usage(argv[0]);
  }

  // get flow field
  FlowData * W;
  W = getFlowField(dataset_str, wind_path_str, quiet, verbose);

  // check for failed load
  if (W != NULL) {
    // read and plot path data
    std::list<Node*> path = readPath(W, traj_path_str, &start_ixyz, &goal_ixyz);
    eigen_dump(start_ixyz);
    eigen_dump(goal_ixyz);

    start_pos = W->getXYZ(start_ixyz);
    goal_pos = W->getXYZ(goal_ixyz);
    eigen_dump(start_pos);
    eigen_dump(goal_pos);

    plotSGGlobal(start_pos, goal_pos);
    plotPathGlobal(path, W, quiet, verbose);

    // plot wind field at fixed height
    plotWind(W, start_ixyz, quiet, verbose);

    if (test_flag) {
      fprintf(stderr, "Plotting Test Point\n");
      int z = 3;
      Eigen::Vector3i ixyz;
      // ixyz << 710, 770, z;  // NC4
      // ixyz << NaN,NaN,NaN;  // NC12
      // ixyz << 787, 763, z;  // NC3
      // ixyz << 780, 680, z;  // NC1
      // ixyz << 715, 670, z;  // NC2
      // ixyz << NaN,NaN,NaN;  // NC7
      // ixyz << NaN, NaN, NaN;  // NC9
      ixyz << 585, 785, z;  // L1
      // ixyz << NaN, NaN, NaN;  // L2
      // ixyz << NaN, NaN, NaN;  // L3
      // ixyz << NaN, NaN, NaN;  // L4
      // ixyz << 360, 528, z;  // L5
      // ixyz << NaN, NaN, NaN;  // L6
      // ixyz << 590, 355, z;  // L7
      // ixyz << NaN, NaN, NaN;  // L8
      // ixyz << 743, 546, z;  // L9
      // ixyz << 810, 630, z;  // L10
      // ixyz << 830, 720, z;  // L11
      // ixyz << NaN, NaN, NaN;  // L12
      plotPoint(W->getXYZ(ixyz), "test");

      Eigen::Vector3f uvw = Eigen::Vector3f::Zero();
      Eigen::Vector3i ixyz_test;
      double count = 0;
      int delta = 2;
      for (int i = -delta; i <= delta; i++) {
        for (int j = -delta; j <= delta; j++) {
          // fprintf(stderr, "i: %i, j: %i\n", i, j);
          ixyz_test << ixyz(0) + i, ixyz(1) + j, ixyz(2);
          // eigen_dump(ixyz_test);
          uvw += W->getUVW(ixyz_test);
          plotPoint(W->getXYZ(ixyz_test), "test_grid");
          count++;
        }
      }
      uvw /= count;

      // uvw = W->getUVW(ixyz);

      double uv_mag = uvw.segment(0, 2).norm();
      double dir = bot_to_degrees(atan2(uvw(1), uvw(0)));
      eigen_dump(uvw);
      eigen_dump(uv_mag);
      eigen_dump(dir);
    }

    if (dump_flag) {
      W->dumpData("map_data.txt");
    }

    if (!quiet && verbose) {
      fprintf(stderr, "Exiting...\n");
    }
  } else {
    fprintf(stderr, "Error: flow_plan main failed to load data.\n");
  }

  return 0;
}

