// Copyright 2015 Jake Ware

// c system includes

// cpp system includes
#include <string>
#include <vector>

// external library includes

// project includes
#include "./flow_data.h"

void bot_lcmgl_line_3d(bot_lcmgl_t * lcmgl, double x_start, double y_start,
                       double z_start, double x_end, double y_end,
                       double z_end) {
  lcmglBegin(LCMGL_LINES);
  lcmglVertex3d(x_start, y_start, z_start);
  lcmglVertex3d(x_end, y_end, z_end);
  lcmglEnd();
}

FlowData::FlowData(const char* path_str, const char* dataset_str, bool _quiet,
                   bool _verbose) {
  // setup
  lcm = bot_lcm_get_global(NULL);
  lcmgl_flow_vels = bot_lcmgl_init(lcm, "flow_vels");
  param = bot_param_get_global(lcm, 0);
  quiet = _quiet;
  verbose = _verbose;

  // grid size
  dim_x = 0;
  dim_y = 0;
  dim_z = 0;

  // get data
  if (!quiet && verbose) {
    fprintf(stderr, "Loading data...\n");
  }

  data_flags.resize(8);
  data_flags.setZero();  // initialize to no files present
  if (getData(path_str, dataset_str)) {
    fprintf(stderr, " Error in getData\n");
  } else if (!quiet && verbose) {
    fprintf(stderr, "Map Dimensions: (%i,%i,%i)\n", dim_x, dim_y, dim_z);
  }

  // store max and min indicies
  ixyz_min << 0, 0, 0;
  ixyz_max << dim_x - 1, dim_y - 1, dim_z - 1;

  // resolution (assuming constant xy resolution)
  Eigen::Vector3i ixyz1, ixyz2;
  ixyz1 << 0, 0, 0;
  ixyz2 << 1, 0, 0;
  res = fabs(getX(ixyz1) - getX(ixyz2));

  getParams();

  // convert start_state to pos and ixyz
  start_pos = start_state.segment(0, 3);
  start_ixyz = getInd(start_pos);

  initCSpace(path_str);
}

FlowData::~FlowData() {
  // Nothing
}

void FlowData::getParams() {
  int r_obs_param;
  char r_obs_key[1024];
  const char * r_obs_string = "flow-data.r_obs";
  snprintf(r_obs_key, sizeof(r_obs_key), "%s", r_obs_string);
  int r_obs_size = bot_param_get_int(param, r_obs_key, &r_obs_param);
  if (r_obs_size < 0) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            r_obs_key);
  } else {
    r_obs = r_obs_param;
  }

  // start state
  double start_state_param[4];
  char start_state_key[1024];
  const char * start_state_string = "flow-plan.start_state";
  snprintf(start_state_key, sizeof(start_state_key), "%s", start_state_string);
  int start_state_size = bot_param_get_double_array(param, start_state_key,
                                                    start_state_param, 4);
  if (start_state_size != 4) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            start_state_key);
  } else {
    start_state(0) = start_state_param[0];
    start_state(1) = start_state_param[1];
    start_state(2) = start_state_param[2];
    start_state(3) = start_state_param[3];
  }

  return;
}

void FlowData::initCSpace(const char* path_str) {
  // check that we have occupancy files
  if (data_flags(7)) {
    if (!quiet && verbose) {
      fprintf(stderr, "Populating collision space... \n");
    }

    // 2d collision space
    fillOcc2D();  // occupancy
    fillObs2D();  // obstacles

    // 3d collision space
    // check if occ octomap exists
    std::string occpath = std::string(path_str) + "occ.bt";
    if (std::ifstream(occpath.c_str())) {
      if (!quiet && verbose) {
        fprintf(stderr, "Loading existing occ octomap...\n");

        double minNegLogLikelihood = 0;  // not used
        occ3d = octomap_utils::loadOctomap(occpath.c_str(),
                                           &minNegLogLikelihood);
      }
    } else {
      // populate octomap
      fillOcc3D();

      if (!quiet && verbose) {
        fprintf(stderr, "Saving occ octomap...\n");
      }

      double minNegLogLikelihood = 0;  // arbitrary
      octomap_utils::saveOctomap(occ3d, occpath.c_str(), minNegLogLikelihood);
    }

    // check if obs octomap exists
    std::string obspath = std::string(path_str) + "obs.bt";
    if (std::ifstream(obspath.c_str())) {
      if (!quiet && verbose) {
        fprintf(stderr, "Loading existing obs octomap...\n");
      }

      double minNegLogLikelihood = 0;  // not used
      obs3d = octomap_utils::loadOctomap(obspath.c_str(), &minNegLogLikelihood);
    } else {
      // populate octomap
      fillObs3D();

      if (!quiet && verbose) {
        fprintf(stderr, "Saving obs octomap...\n");
      }

      double minNegLogLikelihood = 0;  // arbitrary
      octomap_utils::saveOctomap(obs3d, obspath.c_str(), minNegLogLikelihood);
    }
  }

  return;
}

// TODO(jakeware): Need to to this in a more robust way
Eigen::Vector3i FlowData::getInd(Eigen::Vector3f pos) {
  Eigen::Vector3i ind;
  ind(0) = static_cast<int>(floor(static_cast<double>(pos(0))));
  ind(1) = static_cast<int>(floor(static_cast<double>(pos(1))));
  ind(2) = static_cast<int>(ceil(static_cast<double>(pos(2))));

  return ind;
}

int FlowData::checkMapBounds(Eigen::Vector3i ixyz) {
  if (ixyz(0) < ixyz_min(0) || ixyz(0) > ixyz_max(0) || ixyz(1) < ixyz_min(1)
      || ixyz(1) > ixyz_max(1) || ixyz(2) < ixyz_min(2)
      || ixyz(2) > ixyz_max(2)) {
    return 1;
  } else {
    return 0;
  }
}

float FlowData::getU(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  return this->u[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
}

float FlowData::getV(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  return this->v[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
}

float FlowData::getW(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  return this->w[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
}

Eigen::Vector3f FlowData::getUVW(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  Eigen::Vector3f vec;
  vec(0) = this->u[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
  vec(1) = this->v[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
  vec(2) = this->w[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];

  return vec;
}

Eigen::Vector3f FlowData::getUVWVar(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  Eigen::Vector3f vec;
  vec(0) = s2u[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
  vec(1) = s2v[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
  vec(2) = s2w[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];

  return vec;
}

void FlowData::setUVW(Eigen::Vector3i ixyz, Eigen::Vector3f vel) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  Eigen::Vector3d vec;
  u[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)] = vel(0);
  v[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)] = vel(1);
  w[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)] = vel(2);

  return;
}

void FlowData::setUVWVar(Eigen::Vector3i ixyz, Eigen::Vector3f var) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  Eigen::Vector3d vec;
  s2u[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)] = var(0);
  s2v[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)] = var(1);
  s2w[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)] = var(2);

  return;
}

float FlowData::getMag(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  return this->mag[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
}

float FlowData::getT(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  return this->t[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
}

int FlowData::getOcc2D(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    return -1;
  } else {
    int ixy[2];
    ixy[0] = ixyz(0);
    ixy[1] = ixyz(1);
    return (this->occ2d->readValue(ixy) > 0);
  }
}

int FlowData::getOcc3D(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    return -1;
  } else {
    octomap::point3d pxyz;
    pxyz(0) = getX(ixyz);
    pxyz(1) = getY(ixyz);
    pxyz(2) = getZ(ixyz);
    return (this->occ3d->search(pxyz) > 0);
  }
}

int FlowData::getObs2D(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    return -1;
  } else {
    int ixy[2];
    ixy[0] = ixyz(0);
    ixy[1] = ixyz(1);
    return (this->obs2d->readValue(ixy) > 0);
  }
}

int FlowData::getObs3D(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    return -1;
  } else {
    octomap::point3d pxyz;
    pxyz(0) = getX(ixyz);
    pxyz(1) = getY(ixyz);
    pxyz(2) = getZ(ixyz);
    return (this->obs3d->search(pxyz) > 0);
  }
}

float FlowData::getX(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  return this->x[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
}

float FlowData::getY(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  return this->y[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
}

float FlowData::getZ(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  return this->z[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
}

Eigen::Vector3f FlowData::getXYZ(Eigen::Vector3i ixyz) {
  if (checkMapBounds(ixyz)) {
    fprintf(stderr, "Bad Index\n");
    eigen_dump(ixyz);
  }

  Eigen::Vector3f vec;
  vec(0) = this->x[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
  vec(1) = this->y[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];
  vec(2) = this->z[ixyz(0) + dim_x * ixyz(1) + dim_x * dim_y * ixyz(2)];

  return vec;
}

int FlowData::getData(const char *path_str, const char *dataset_str) {
  if (!quiet && verbose) {
    fprintf(stderr, "Opening data folder... ");
  }

  DIR *dirp = opendir(path_str);
  struct dirent *dp;

  if (!quiet && verbose) {
    fprintf(stderr, "Done.\n");
  }

  while ((dp = readdir(dirp)) != NULL) {
    if (!quiet && verbose) {
      fprintf(stderr, "%s\n", dp->d_name);
    }
    std::string fname = std::string(dp->d_name);

    // u velocity
    if (fname.find("Vu", 0) != std::string::npos
        && fname.find(dataset_str, 0) != std::string::npos) {
      // fprintf(stderr, "check: u\n");

      if (data_flags(0)) {
        fprintf(stderr, "Error: getData found duplicate datasets for u\n");
        return 1;
      } else {
        if (!quiet && verbose) {
          fprintf(stderr, "Reading u data... \n");
        }
        data_flags(0) = true;
      }

      std::string fullpath = std::string(path_str) + fname;
      if (readHDF5(fullpath, dataset_str, u)) {
        fprintf(stderr, "Error: readHDF5 failed in getData for u\n");
        return 1;
      }
    } else if (fname.find("Vv", 0) != std::string::npos
        && fname.find(dataset_str, 0) != std::string::npos) {  // v velocity
        // fprintf(stderr, "check: v\n");

      if (data_flags(1)) {
        fprintf(stderr, "Error: getData found duplicate datasets for v\n");
        return 1;
      } else {
        if (!quiet && verbose) {
          fprintf(stderr, "Reading v data... \n");
        }
        data_flags(1) = true;
      }

      std::string fullpath = std::string(path_str) + fname;
      if (readHDF5(fullpath, dataset_str, v)) {
        fprintf(stderr, "Error: readHDF5 failed in getData for v\n");
        return 1;
      }
    } else if (fname.find("Vw", 0) != std::string::npos
        && fname.find(dataset_str, 0) != std::string::npos) {  // w velocity
        // fprintf(stderr, "check: w\n");

      if (data_flags(2)) {
        fprintf(stderr, "Error: getData found duplicate datasets for w\n");
        return 1;
      } else {
        if (!quiet && verbose) {
          fprintf(stderr, "Reading w data... \n");
        }
        data_flags(2) = true;
      }

      std::string fullpath = std::string(path_str) + fname;
      if (readHDF5(fullpath, dataset_str, w)) {
        fprintf(stderr, "Error: readHDF5 failed in getData for w\n");
        return 1;
      }
    } else if (fname.find("Vmag", 0) != std::string::npos
        && fname.find(dataset_str, 0) != std::string::npos) {  // mag velocity
        // fprintf(stderr, "check: mag\n");

      if (data_flags(3)) {
        fprintf(stderr, "Error: getData found duplicate datasets for mag\n");
        return 1;
      } else {
        if (!quiet && verbose) {
          fprintf(stderr, "Reading mag data... \n");
        }
        data_flags(3) = true;
      }

      std::string fullpath = std::string(path_str) + fname;
      if (readHDF5(fullpath, dataset_str, mag)) {
        fprintf(stderr, "Error: readHDF5 failed in getData for mag\n");
        return 1;
      }
    } else if (fname.find("Vx", 0) != std::string::npos
        && fname.find(dataset_str, 0) != std::string::npos) {  // x grid
        // fprintf(stderr, "check: x\n");

      if (data_flags(4)) {
        fprintf(stderr, "Error: getData found duplicate datasets for x\n");
        return 1;
      } else {
        if (!quiet && verbose) {
          fprintf(stderr, "Reading x data... \n");
        }
        data_flags(4) = true;
      }

      std::string fullpath = std::string(path_str) + fname;
      if (readHDF5(fullpath, dataset_str, x)) {
        fprintf(stderr, "Error: readHDF5 failed in getData for x\n");
        return 1;
      }
    } else if (fname.find("Vy", 0) != std::string::npos
        && fname.find(dataset_str, 0) != std::string::npos) {  // y grid
        // fprintf(stderr, "check: y\n");

      if (data_flags(5)) {
        fprintf(stderr, "Error: getData found duplicate datasets for y\n");
      } else {
        if (!quiet && verbose) {
          fprintf(stderr, "Reading y data... \n");
        }
        data_flags(5) = true;
      }

      std::string fullpath = std::string(path_str) + fname;
      if (readHDF5(fullpath, dataset_str, y)) {
        fprintf(stderr, "Error: readHDF5 failed in getData for y\n");
        return 1;
      }
    } else if (fname.find("Vz", 0) != std::string::npos
        && fname.find(dataset_str, 0) != std::string::npos) {  // z grid
      if (!quiet && verbose) {
        fprintf(stderr, "check: z data\n");
      }

      if (data_flags(6)) {
        fprintf(stderr, "Error: getData found duplicate datasets for z\n");
        return 1;
      } else {
        if (!quiet && verbose) {
          fprintf(stderr, "Reading z data... \n");
        }
        data_flags(6) = true;
      }

      std::string fullpath = std::string(path_str) + fname;
      if (readHDF5(fullpath, dataset_str, z)) {
        fprintf(stderr, "Error: readHDF5 failed in getData for z\n");
        return 1;
      }
    } else if (fname.find("Vt", 0) != std::string::npos
        && fname.find(dataset_str, 0) != std::string::npos) {  // celltype
        // fprintf(stderr, "check: t\n");

      if (data_flags(7)) {
        fprintf(stderr, "Error: getData found duplicate datasets for t\n");
        return 1;
      } else {
        if (!quiet && verbose) {
          fprintf(stderr, "Reading t data... \n");
        }
        data_flags(7) = true;
      }

      std::string fullpath = std::string(path_str) + fname;
      if (readHDF5(fullpath, dataset_str, t)) {
        fprintf(stderr, "Error: readHDF5 failed in getData for t\n");
        return 1;
      }
    } else {
      // nothing
    }
  }

  (void) closedir(dirp);

  return 0;
}

int FlowData::readHDF5(std::string fullpath, const char *dataset_str,
                       std::vector<float> &vec) {
  if (!quiet && verbose) {
    fprintf(stderr, "Reading data from path: %s\n", fullpath.c_str());
  }

  const std::string FILE_NAME(fullpath);
  const std::string DATASET_NAME(dataset_str);

  // Try block to detect exceptions raised by any of the calls inside it
  try {
    // Turn off the auto-printing when failure occurs so that we can
    // handle the errors appropriately
    H5::Exception::dontPrint();

    // Open an existing file and dataset.
    H5::H5File file(FILE_NAME, H5F_ACC_RDWR);
    H5::DataSet dataset = file.openDataSet(DATASET_NAME);
    H5T_class_t type_class = dataset.getTypeClass();

    if (type_class == H5T_FLOAT) {
      if (!quiet && verbose) {
        std::cout << "Data set has FLOAT type" << std::endl;
      }
      H5::FloatType floatype = dataset.getFloatType();

      // order
      std::string order_string;
      H5T_order_t order = floatype.getOrder(order_string);
      if (!quiet && verbose) {
        std::cout << order_string << std::endl;
      }

      // size
      size_t size = floatype.getSize();
      if (!quiet && verbose) {
        std::cout << "Data size: " << size << std::endl;
      }
    }

    // get dimensions
    H5::DataSpace dataspace = dataset.getSpace();
    static const int rank = dataspace.getSimpleExtentNdims();
    hsize_t dims_out[1];
    int ndims = dataspace.getSimpleExtentDims(dims_out, NULL);
    if (!quiet && verbose) {
      std::cout << "rank: " << rank << std::endl;
      std::cout << "dimensions; " << (unsigned long) (dims_out[0]) << std::endl;
    }

    // select portion of file to read
    hsize_t offset[1];  // hyperslab offset in the file
    hsize_t count[1];  // size of the hyperslab in the file
    offset[0] = 0;
    count[0] = dims_out[0];
    dataspace.selectHyperslab(H5S_SELECT_SET, count, offset);

    // define space in memory
    hsize_t dimsm[1]; /* memory space dimensions */
    dimsm[0] = dims_out[0];
    H5::DataSpace memspace(rank, dimsm);

    // define memory hyperslab.
    hsize_t offset_out[3];  // hyperslab offset in memory
    hsize_t count_out[3];  // size of the hyperslab in memory
    offset_out[0] = 0;
    count_out[0] = dims_out[0];
    memspace.selectHyperslab(H5S_SELECT_SET, count_out, offset_out);

    // read
    vec.resize(dims_out[0]);
    dataset.read(vec.data(), H5::PredType::NATIVE_FLOAT, memspace, dataspace);

    // find dimensions in file path
    int ix1 = fullpath.find("X", 7);
    ix1++;
    int ix2 = fullpath.find("_", ix1 + 1);
    // fprintf(stderr, "dimx: %i %i\n", ix1, ix2);
    int iy1 = fullpath.find("Y", 7);
    iy1++;
    int iy2 = fullpath.find("_", iy1 + 1);
    // fprintf(stderr, "dimy: %i %i\n", iy1, iy2);
    int iz1 = fullpath.find("Z", 7);
    iz1++;
    int iz2 = fullpath.find(".", iz1 + 1);
    // fprintf(stderr, "dimz: %i %i\n", iz1, iz2);

    // convert to string
    std::string dimx_str = fullpath.substr(ix1, ix2 - ix1);
    // fprintf(stderr, "%s\n", dimx_str);
    int dimx = atoi(dimx_str.c_str());
    // fprintf(stderr, "dimx: %i\n", dimx);
    std::string dimy_str = fullpath.substr(iy1, iy2 - iy1);
    // fprintf(stderr, "%s\n", dimy_str);
    int dimy = atoi(dimy_str.c_str());
    // fprintf(stderr, "dimy: %i\n", dimy);
    std::string dimz_str = fullpath.substr(iz1, iz2 - iz1);
    // fprintf(stderr, "%s\n", dimz_str);
    int dimz = atoi(dimz_str.c_str());
    // fprintf(stderr, "dimz: %i\n", dimz);

    // check and get x dimension
    if (dim_x == 0) {
      dim_x = dimx;
    } else if (dim_x != dimx) {
      fprintf(stderr, "Error: x dimensions not consistent given (%i,%i)\n",
              dim_x, dimx);
      return 1;
    } else if (dim_x == dimx) {
      // nothing
    } else {
      fprintf(stderr,
              "Error: Unhandled x dimension value in readData given (%i,%i)\n",
              dim_x, dimx);
    }

    // check and get y dimension
    if (dim_y == 0) {
      dim_y = dimy;
    } else if (dim_y != dimy) {
      fprintf(stderr, "Error: y dimensions not consistent given (%i,%i)\n",
              dim_y, dimy);
      return 1;
    } else if (dim_y == dimy) {
      // nothing
    } else {
      fprintf(stderr,
              "Error: Unhandled y dimension value in readData given (%i,%i)\n",
              dim_y, dimy);
    }

    // check and get z dimension
    if (dim_z == 0) {
      dim_z = dimz;
    } else if (dim_z != dimz) {
      fprintf(stderr, "Error: z dimensions not consistent given (%i,%i)\n",
              dim_z, dimz);
      return 1;
    } else if (dim_z == dimz) {
      // nothing
    } else {
      fprintf(stderr,
              "Error: Unhandled z dimension value in readData given (%i,%i)\n",
              dim_z, dimz);
    }
  }  // end of try block

  // catch failure caused by the H5File operations
  catch (H5::FileIException &error) {
    error.printError();
    return -1;
  }
  // catch failure caused by the DataSet operations
  catch (H5::DataSetIException &error) {
    error.printError();
    return -1;
  }
  // catch failure caused by the DataSpace operations
  catch (H5::DataSpaceIException &error) {
    error.printError();
    return -1;
  }
  // catch failure caused by the DataSpace operations
  catch (H5::DataTypeIException &error) {
    error.printError();
    return -1;
  }

  return 0;
}

void FlowData::fillOcc2D() {
  if (!quiet && verbose) {
    fprintf(stderr, "Filling 2D Occupancy...");
  }

  // init occ map
  double xy0[2];  // lower left
  double xy1[2];  // upper right
  xy0[0] = 0;
  xy0[1] = 0;
  xy1[0] = dim_x;
  xy1[1] = dim_y;

  occ2d = new occ_map::PixelMap<float>(xy0, xy1, res, 0, true, true);

  int k = 1;  // z index
  for (int i = 0; i < dim_x; i++) {
    for (int j = 0; j < dim_y; j++) {
      Eigen::Vector3i ixyz;
      ixyz << i, j, k;

      Eigen::Vector3f pos = getXYZ(ixyz);

      double point[2];
      point[0] = pos(0) - 1;
      point[1] = pos(1) - 1;

      if (getT(ixyz) == 0) {
        occ2d->writeValue(point, 1.0);
      } else {
        occ2d->writeValue(point, 0.0);
      }
    }
  }

  if (!quiet && verbose) {
    fprintf(stderr, " Done\n");
  }

  return;
}

void FlowData::fillOcc3D() {
  if (!quiet && verbose) {
    fprintf(stderr, "Filling 3D Occupancy...");
  }

  // init octomap
  occ3d = new octomap::OcTree(res);

  for (int i = 0; i < dim_x; i++) {
    for (int j = 0; j < dim_y; j++) {
      for (int k = 0; k < dim_z; k++) {
        Eigen::Vector3i ixyz;
        ixyz << i, j, k;

        if (getT(ixyz) == 0) {
          octomap::point3d pxyz;
          pxyz(0) = getX(ixyz);
          pxyz(1) = getY(ixyz);
          pxyz(2) = getZ(ixyz);
          occ3d->updateNode(pxyz, true, false);
        }
      }
    }
  }

  if (!quiet && verbose) {
    fprintf(stderr, " Done\n");
  }

  return;
}

// TODO(jakeware): there is some small fixed offset on this
void FlowData::fillObs2D() {
  if (!quiet && verbose) {
    fprintf(stderr, "Filling 2D Obstacles...");
  }

  // init occ map
  double xy0[2];  // lower left
  double xy1[2];  // upper right
  xy0[0] = 0;
  xy0[1] = 0;
  xy1[0] = dim_x;
  xy1[1] = dim_y;
  obs2d = new occ_map::PixelMap<float>(xy0, xy1, res, 0, true, true);

  // create test box matrix
  Eigen::MatrixXi r_test;
  r_test.resize(pow((2 * r_obs) + 1, 2), 2);
  for (int i = 0; i < 2 * r_obs + 1; i++) {
    for (int j = 0; j < 2 * r_obs + 1; j++) {
      // cout << "i: " << i << " j: " << j << "\n";
      // cout << pow(i-r_obs,2) << " " << pow(j-r_obs,2) << "\n";
      // check radius
      if (pow(i - r_obs, 2) + pow(j - r_obs, 2) <= pow(r_obs, 2)) {
        // cout << i-r_obs << " " << j-r_obs << "\n";
        r_test.row(i * (2 * r_obs + 1) + j) << i - r_obs, j - r_obs;
      } else {
        // cout << 0 << " " << 0 << "\n";
        r_test.row(i * (2 * r_obs + 1) + j) << 0, 0;
      }
    }
  }

  // eigen_dump(r_test);

  int k = 1;  // z index
  for (int i = 0; i < dim_x; i++) {
    for (int j = 0; j < dim_y; j++) {
      Eigen::Vector3i ixyz;
      ixyz << i, j, k;

      Eigen::Vector3f pos = getXYZ(ixyz);

      double point[2];
      point[0] = pos(0) - 1;
      point[1] = pos(1) - 1;

      // is center point occupied?
      if (getOcc2D(ixyz) == 0) {
        // test radius around point
        for (int r = 0; r < r_test.rows(); r++) {
          // get test point and skip if at center
          if (r_test(r, 0) == 0 && r_test(r, 1) == 0) {
            continue;
          }

          Eigen::Vector3i ixyz2;
          ixyz2 << i + r_test(r, 0), j + r_test(r, 1), k;
          // occupied?
          if (getOcc2D(ixyz2) == 1) {
            obs2d->writeValue(point, 0.5);
          }
        }
      } else {
        obs2d->writeValue(point, 1.0);
      }
    }
  }

  if (!quiet && verbose) {
    fprintf(stderr, " Done\n");
  }

  return;
}

void FlowData::fillObs3D() {
  if (!quiet && verbose) {
    fprintf(stderr, "Filling 3D Obstacles...");
  }

  // init octomap
  obs3d = new octomap::OcTree(res);

  // create test box matrix
  Eigen::MatrixXi r_test;
  r_test.resize(pow((2 * r_obs) + 1, 3), 3);
  for (int i = 0; i < 2 * r_obs + 1; i++) {
    for (int j = 0; j < 2 * r_obs + 1; j++) {
      for (int k = 0; k < 2 * r_obs + 1; k++) {
        // cout << "i: " << i << " j: " << j << "\n";
        // cout << pow(i-r_obs,2) << " " << pow(j-r_obs,2) << "\n";
        // check radius
        if (pow(i - r_obs, 2) + pow(j - r_obs, 2) + pow(k - r_obs, 2)
            <= pow(r_obs, 2)) {
          // cout << i-r_obs << " " << j-r_obs << "\n";
          r_test.row(i * pow((2 * r_obs + 1), 2) + j * (2 * r_obs + 1) + k)
              << i - r_obs, j - r_obs, k - r_obs;
        } else {
          // cout << 0 << " " << 0 << "\n";
          r_test.row(i * (2 * r_obs + 1) + j) << 0, 0, 0;
        }
      }
    }
  }

  // eigen_dump(r_test);

  for (int i = 0; i < dim_x; i++) {
    for (int j = 0; j < dim_y; j++) {
      for (int k = 0; k < dim_z; k++) {
        Eigen::Vector3i ixyz;
        ixyz << i, j, k;

        // is center point occupied?
        if (getOcc3D(ixyz) == 0) {
          // test radius around point
          for (int r = 0; r < r_test.rows(); r++) {
            // get test point and skip if at center
            if (r_test(r, 0) == 0 && r_test(r, 1) == 0 && r_test(r, 2) == 0) {
              continue;
            }

            Eigen::Vector3i ixyz2;
            ixyz2 << i + r_test(r, 0), j + r_test(r, 1), k + r_test(r, 2);
            // occupied and not the ground?
            if (getOcc3D(ixyz2) == 1 && getZ(ixyz2) > 0.0) {
              octomap::point3d pxyz;
              pxyz(0) = getX(ixyz);
              pxyz(1) = getY(ixyz);
              pxyz(2) = getZ(ixyz);
              obs3d->updateNode(pxyz, true, false);
            }
          }
        } else {
          octomap::point3d pxyz;
          pxyz(0) = getX(ixyz);
          pxyz(1) = getY(ixyz);
          pxyz(2) = getZ(ixyz);
          obs3d->updateNode(pxyz, true, false);
        }
      }
    }
  }

  if (!quiet && verbose) {
    fprintf(stderr, " Done\n");
  }

  return;
}

void FlowData::plotOcc2D() {
  if (!quiet && verbose) {
    fprintf(stderr, "Plotting 2D Occupancy...");
  }

  occ_map_pixel_map_t * msg = (occ_map_pixel_map_t *) occ2d->get_pixel_map_t(
      bot_timestamp_now());
  occ_map_pixel_map_t_publish(lcm, "STRUCTURE_PIXELMAP", msg);
  occ_map_pixel_map_t_destroy(msg);

  if (!quiet && verbose) {
    fprintf(stderr, " Done\n");
  }

  return;
}

void FlowData::plotOcc3D() {
  if (!quiet && verbose) {
    fprintf(stderr, "Plotting 3D Occupancy...");
  }

  octomap_raw_t msg;
  msg.utime = bot_timestamp_now();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      msg.transform[i][j] = 0;
    }
    msg.transform[i][i] = 1;
  }

  // set translation
  msg.transform[0][3] = -start_pos(0);
  msg.transform[1][3] = -start_pos(1);

  std::stringstream datastream;
  occ3d->writeBinaryConst(datastream);
  std::string datastring = datastream.str();
  msg.data = (uint8_t *) datastring.c_str();
  msg.length = datastring.size();

  octomap_raw_t_publish(lcm, "OCTOMAP", &msg);

  if (!quiet && verbose) {
    fprintf(stderr, " Done\n");
  }

  return;
}

void FlowData::plotObs2D() {
  if (!quiet && verbose) {
    fprintf(stderr, "Plotting 2D Obstacles...");
  }

  occ_map_pixel_map_t * msg = (occ_map_pixel_map_t *) obs2d->get_pixel_map_t(
      bot_timestamp_now());
  occ_map_pixel_map_t_publish(lcm, "CSPACE_PIXEL_MAP", msg);
  occ_map_pixel_map_t_destroy(msg);

  if (!quiet && verbose) {
    fprintf(stderr, " Done\n");
  }

  return;
}

void FlowData::plotObs3D() {
  if (!quiet && verbose) {
    fprintf(stderr, "Plotting 3D Obstacles...");
  }

  octomap_raw_t msg;
  msg.utime = bot_timestamp_now();

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      msg.transform[i][j] = 0;
    }
    msg.transform[i][i] = 1;
  }

  // set translation
  msg.transform[0][3] = -start_pos(0);
  msg.transform[1][3] = -start_pos(1);

  std::stringstream datastream;
  obs3d->writeBinaryConst(datastream);
  std::string datastring = datastream.str();
  msg.data = (uint8_t *) datastring.c_str();
  msg.length = datastring.size();

  octomap_raw_t_publish(lcm, "OCTOMAP", &msg);

  if (!quiet && verbose) {
    fprintf(stderr, " Done\n");
  }

  return;
}

void FlowData::plotUV(int k, double flow_scale, int increment) {
  if (!quiet && verbose) {
    fprintf(stderr, "Plotting flow...");
  }

  bot_lcmgl_line_width(lcmgl_flow_vels, 2);
  bot_lcmgl_color3f(lcmgl_flow_vels, 0.5, 0.5, 0.5);
  bot_lcmgl_begin(lcmgl_flow_vels, GL_LINES);
  for (int i = 0; i < dim_x; i += increment) {
    for (int j = 0; j < dim_y; j += increment) {
      Eigen::Vector3i ixyz;
      ixyz << i, j, k;

      // check for existence of occ data and then skip if occuppied
      if (data_flags(7)) {
        if (getObs3D(ixyz)) {
          continue;
        }
      }

      // plot stem of arrow
      Eigen::Vector3f p1 = getXYZ(ixyz);
      p1(0) -= start_pos(0);
      p1(1) -= start_pos(1);

      Eigen::Vector3f p2 = p1 + getUVW(ixyz) * flow_scale;

      bot_lcmgl_line_3d(lcmgl_flow_vels, static_cast<double>(p1(0)),
                        static_cast<double>(p1(1)), static_cast<double>(p1(2)),
                        static_cast<double>(p2(0)), static_cast<double>(p2(1)),
                        static_cast<double>(p2(2)));
    }
  }
  bot_lcmgl_end(lcmgl_flow_vels);

  // plot points at base of wind vector
  bot_lcmgl_point_size(lcmgl_flow_vels, 3);
  bot_lcmgl_color3f(lcmgl_flow_vels, 0.5, 0.5, 0.5);
  bot_lcmgl_begin(lcmgl_flow_vels, GL_POINTS);

  for (int i = 0; i < dim_x; i += increment) {
    for (int j = 0; j < dim_y; j += increment) {
      Eigen::Vector3i ixyz;
      ixyz << i, j, k;

      if (data_flags(7)) {
        if (getObs3D(ixyz)) {
          continue;
        }
      }

      Eigen::Vector3f p1 = getXYZ(ixyz);
      p1(0) -= start_pos(0);
      p1(1) -= start_pos(1);

      bot_lcmgl_vertex3d(lcmgl_flow_vels, static_cast<double>(p1(0)),
                         static_cast<double>(p1(1)),
                         static_cast<double>(p1(2)));
    }
  }
  bot_lcmgl_end(lcmgl_flow_vels);

  bot_lcmgl_switch_buffer(lcmgl_flow_vels);

  if (!quiet && verbose) {
    fprintf(stderr, " Done\n");
  }

  return;
}

void FlowData::plotUVGlobal(int k, double flow_scale, int increment) {
  if (!quiet && verbose) {
    fprintf(stderr, "Plotting flow...");
  }

  bot_lcmgl_line_width(lcmgl_flow_vels, 2);
  bot_lcmgl_color3f(lcmgl_flow_vels, 0.5, 0.5, 0.5);
  bot_lcmgl_begin(lcmgl_flow_vels, GL_LINES);
  for (int i = 0; i < dim_x; i += increment) {
    for (int j = 0; j < dim_y; j += increment) {
      Eigen::Vector3i ixyz;
      ixyz << i, j, k;

      // check for existence of occ data and then skip if occuppied
      if (data_flags(7)) {
        if (getObs3D(ixyz)) {
          continue;
        }
      }

      // plot stem of arrow
      Eigen::Vector3f p1 = getXYZ(ixyz);
      Eigen::Vector3f p2 = p1 + getUVW(ixyz) * flow_scale;

      bot_lcmgl_line_3d(lcmgl_flow_vels, static_cast<double>(p1(0)),
                        static_cast<double>(p1(1)), static_cast<double>(p1(2)),
                        static_cast<double>(p2(0)), static_cast<double>(p2(1)),
                        static_cast<double>(p2(2)));
    }
  }
  bot_lcmgl_end(lcmgl_flow_vels);

  // plot points at base of wind vector
  bot_lcmgl_point_size(lcmgl_flow_vels, 3);
  bot_lcmgl_color3f(lcmgl_flow_vels, 0.5, 0.5, 0.5);
  bot_lcmgl_begin(lcmgl_flow_vels, GL_POINTS);

  for (int i = 0; i < dim_x; i += increment) {
    for (int j = 0; j < dim_y; j += increment) {
      Eigen::Vector3i ixyz;
      ixyz << i, j, k;

      if (data_flags(7)) {
        if (getObs3D(ixyz)) {
          continue;
        }
      }

      Eigen::Vector3f p1 = getXYZ(ixyz);

      bot_lcmgl_vertex3d(lcmgl_flow_vels, static_cast<double>(p1(0)),
                         static_cast<double>(p1(1)),
                         static_cast<double>(p1(2)));
    }
  }
  bot_lcmgl_end(lcmgl_flow_vels);

  bot_lcmgl_switch_buffer(lcmgl_flow_vels);

  if (!quiet && verbose) {
    fprintf(stderr, " Done\n");
  }

  return;
}

void FlowData::dumpData(std::string fname) {
  std::ofstream file;
  file.open(fname.c_str());
  for (int i = 0; i < dim_x; i++) {
    for (int j = 0; j < dim_y; j++) {
      for (int k = 0; k < dim_z; k++) {
        Eigen::Vector3i ixyz;
        ixyz << i, j, k;
        file << i << ',' << j << ',' << k << ',' << getX(ixyz) << ','
             << getY(ixyz) << ',' << getZ(ixyz) << ',' << getU(ixyz) << ','
             << getV(ixyz) << ',' << getW(ixyz) << std::endl;
      }
    }
  }
  file.close();

  return;
}
