// Copyright 2015 Jake Ware

// c system includes
#include <getopt.h>
#include <eigen3/Eigen/Dense>

// cpp system includes
#include <vector>
#include <string>

// external library includes

// project includes
#include "flow_plan_app/flow_plan_app.h"
#include "flow_data/flow_data.h"
#include "astar/astar.h"
#include "utils/plot_utils.h"
#include "planner/planner.h"
// #include "rrt/rrt.h"
// #include "rrt_star/rrt_star.h"
// #include "dijkstra/dijkstra.h"

// TODO(jakeware) on Startup
// Set map bounds
// Set start and goal points
// Set wind flag
// Set air speed cap flag

void usage(const char *progname) {
  char *basename = g_path_get_basename(progname);
  printf(
      "Usage: %s [options]\n"
      "\n"
      "Options:\n"
      "\n"
      "    -h, --help                Shows this help text and exits\n"
      "    -v, --verbose             Enables additional text output\n"
      "    -q, --quiet               Suppresses text output\n"
      "representation\n"
      "    -f, --file                Specifies path of wind field files "
      "(./path/to/files/)\n"
      "    -n, --naive               Disables wind field planning\n"
      "    -a, --vela                Enables air speed constraints for naive "
      "planner\n"
      "    -r, --rand                Enables random starts with an argument "
      "specifying number of trials\n"
      "    -d, --dump                Enables writing trajectory data to a file"
      "\n"
      "    -p, --plot                Enables plotting in viewer\n"
      "    -m, --mindist             Specifies minimum distance between start "
      "and goal for random starts and goals\n"
      "    -t, --traj                Enables publishing trajectory to "
      "quad-planner\n."
      "    -s, --test                Plot start and goal locations without "
      "planning\n"
      "\n",
      basename);
  free(basename);
  exit(1);
}

void getRandSG(FlowData * W, App * app) {
  Eigen::Vector3i start_ixyz;
  Eigen::Vector3f start_pos;
  Eigen::Vector3i goal_ixyz;
  Eigen::Vector3f goal_pos;
  bool valid = false;
  while (!valid) {
    // get start location
    while (true) {
      // random sample
      start_ixyz(0) = rand_r(&app->seed) % (app->ixyz_max(0) - app->ixyz_min(0))
          + app->ixyz_min(0);
      start_ixyz(1) = rand_r(&app->seed) % (app->ixyz_max(1) - app->ixyz_min(1))
          + app->ixyz_min(1);
      start_ixyz(2) = app->z_index;

      // check and break if good
      if (!W->getObs3D(start_ixyz)) {
        break;
      }
    }

    // get goal location
    while (true) {
      // random sample
      goal_ixyz(0) = rand_r(&app->seed) % (app->ixyz_max(0) - app->ixyz_min(0))
          + app->ixyz_min(0);
      goal_ixyz(1) = rand_r(&app->seed) % (app->ixyz_max(1) - app->ixyz_min(1))
          + app->ixyz_min(1);
      goal_ixyz(2) = app->z_index;

      // check and break if good
      if (!W->getObs3D(goal_ixyz)) {
        break;
      }
    }

    // initialize start and goal states
    start_pos = W->getXYZ(start_ixyz);
    app->start_state.segment(0, 3) = start_pos;
    app->start_state(3) = app->vel_g_min;

    goal_pos = W->getXYZ(goal_ixyz);
    app->goal_state.segment(0, 3) = goal_pos;
    app->goal_state(3) = app->vel_g_min;

    if ((start_pos - goal_pos).norm() > app->min_dist) {
      valid = true;
    }
  }

  return;
}

void randPlan(FlowData* W, App* app) {
  // get paths
  int trial_count = 0;
  int fail_count = 0;
  for (int i = 0; i < app->rand_count; i++) {
    trial_count++;

    // set path number
    app->path_num = i;

    // get random start and goal locations
    getRandSG(W, app);

    // wind planner
    app->naive_flag = false;
    // PlannerD P2(W, app);
    Planner P2(W, app);

    // check path length
    if (P2.path_.size() < 2) {
      fail_count++;
      i--;
      continue;
    }

    // naive planner
    app->naive_flag = true;
    app->vel_a_cap_flag = true;
    // PlannerD P1(W, app);
    Planner P1(W, app);
  }

  fprintf(stderr, "total: %i, failures: %i\n", trial_count, fail_count);

  return;
}

FlowData* getFlowField(std::string dataset_str, std::string path_str,
                       App* app) {
  // get data
  FlowData * W = new FlowData(path_str.c_str(), dataset_str.c_str(), app->quiet,
                              app->verbose);

  // plotting
  if (W != NULL) {
    if (app->plot) {
      // flow field
      fprintf(stderr, "Plotting flow field\n");
      Eigen::Vector3i ixyz = W->getInd(app->start_state.segment(0, 3));
      int z = ixyz(2);
      W->plotUV(z, app->flow_scale, app->flow_increment);

      // obstacle map
      fprintf(stderr, "Plotting obstacle space\n");
      // W->plotOcc2D();
      W->plotObs2D();

      // W->plotOcc3D();
      W->plotObs3D();
    }
  }

  return W;
}

int main(int argc, char **argv) {
  // define app object
  App app;

  // set defaults
  std::string dataset_str = "DSM";
  std::string path_str = "./data/";

  // parse options
  const char *optstring = "hvqf:nar:dpm:ts";
  struct option long_opts[] = { { "help", no_argument, 0, 'h' }, { "verbose",
  no_argument, 0, 'v' }, { "quiet", no_argument, 0, 'q' }, { "file",
  required_argument, 0, 'f' }, { "naive",
  no_argument, 0, 'n' }, { "vela", no_argument, 0, 'a' }, { "rand",
  required_argument, 0, 'r' }, { "dump", no_argument, 0, 'd' }, { "plot",
  no_argument, 0, 'p' }, { "mindist", required_argument, 0, 'm' }, { "traj",
  no_argument, 0, 't' }, { "test", no_argument, 0, 's' }, { 0, 0, 0, 0 } };
  int c;
  while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
    switch (c) {
      case 'h':
        usage(argv[0]);
        break;
      case 'v':
        app.verbose = true;
        break;
      case 'q':
        app.quiet = true;
        break;
      case 'f':
        if (optarg != NULL) {
          app.filename_flag = true;
          path_str = std::string(optarg);
          app.filename = path_str;
        }
        break;
      case 'n':
        app.naive_flag = true;
        break;
      case 'a':
        app.vel_a_cap_flag = true;
        break;
      case 'r':
        if (optarg != NULL) {
          app.rand = true;
          app.rand_count = atoi(optarg);
        }
        break;
      case 'd':
        app.dump = true;
        break;
      case 'p':
        app.plot = true;
        break;
      case 'm':
        if (optarg != NULL) {
          app.min_dist = atof(optarg);
        }
        break;
      case 't':
        app.publish = true;
        break;
      case 's':
        app.test_sg = true;
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

  if (!app.quiet && !app.verbose) {
    fprintf(stderr, "Starting...\n");
  }

  // get flow field
  FlowData* W = getFlowField(dataset_str, path_str, &app);

  // check for failed load
  if (W != NULL) {
    // search
    if (app.rand) {
      randPlan(W, &app);
    } else {
      // PlannerD P(W, &app);
      // PlannerAS P(W, &app);
      Planner P(W, &app);
    }

    if (!app.quiet && !app.verbose) {
      fprintf(stderr, "Exiting...\n");
    }
  } else {
    fprintf(stderr, "Error: Failed to load flow data.\n");
  }

  return 0;
}
