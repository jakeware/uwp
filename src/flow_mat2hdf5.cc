#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <glib.h>
#include <stdint.h>
#include <termios.h>
#include <iostream>
#include <getopt.h>
#include <math.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

#include <string>

extern "C" {
  #include "mat2hdf5/mat2hdf5.h"
}

using namespace std;

void usage(const char *progname) {
  char *basename = g_path_get_basename(progname);
  printf("Usage: %s [options]\n"
         "\n"
         "Options:\n"
         "\n"
         "    -h, --help                Shows this help text and exits\n"
         "    -v, --verbose\n"
         "    -q, --quiet\n"
         "    -d, --dataset             Specifies the dataset tag, e.g. '1'\n"
         "    -p, --path                Specifies path to data files.\n"
         "\n",
         basename);
  free(basename);
  exit(1);
}

int main(int argc, char **argv) {
  bool verbose = false;
  bool quiet = false;
  string dataset = "DSM";
  string path = "./data/";

  // parse options
  const char *optstring = "hvqd:p:";
  struct option long_opts[] = { { "help", no_argument, 0, 'h' }, { "verbose",
  no_argument, 0, 'v' }, { "quiet", no_argument, 0, 'q' }, { "dataset",
  required_argument, 0, 'd' }, { "path", required_argument, 0, 'p' }, { 0, 0, 0,
      0 } };
  int c;
  while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
    switch (c) {
      case 'h':
        usage(argv[0]);
        break;
      case 'v':
        verbose = 1;
        break;
      case 'q':
        quiet = 1;
        break;
      case 'd':
        if (optarg != NULL) {
          dataset = string(optarg);
        }
        break;
      case 'p':
        if (optarg != NULL) {
          path = std::string(optarg);
        }
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

  const char *path_c = path.c_str();
  const char *dataset_c = dataset.c_str();
  char type;

  // velocity
  type = 'v';
  if (mat2hdf5(path_c, dataset_c, type, static_cast<int>(verbose),
               static_cast<int>(quiet))) {
    fprintf(stderr, "Error: mat2hdf5 failed for velocity\n");
  }

  // celltype
  type = 'c';
  if (mat2hdf5(path_c, dataset_c, type, static_cast<int>(verbose),
               static_cast<int>(quiet))) {
    fprintf(stderr, "Error: mat2hdf5 failed for celltype\n");
  }

  // windgrid
  type = 'w';
  if (mat2hdf5(path_c, dataset_c, type, static_cast<int>(verbose),
               static_cast<int>(quiet))) {
    fprintf(stderr, "Error: mat2hdf5 failed for windgrid\n");
  }

  return 0;
}
