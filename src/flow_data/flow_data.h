#ifndef FLOW_PLAN_SRC_FLOW_DATA_FLOW_DATA_H_
#define FLOW_PLAN_SRC_FLOW_DATA_FLOW_DATA_H_

// Copyright 2015 Jake Ware

// c system includes
#include <dirent.h>

// cpp system includes
#include <string>
#include <vector>

// external library includes
#include "eigen3/Eigen/Dense"
#include "bot_param/param_client.h"
#include "bot_param/param_util.h"
#include "bot_lcmgl_client/lcmgl.h"
#include "eigen_utils/eigen_utils.hpp"
#include "lcm/lcm.h"
#include "bot_core/bot_core.h"
#include "H5Cpp.h"
#include "occ_map/PixelMap.hpp"
#include "octomap_utils/octomap_util.hpp"
#include "lcmtypes/octomap_utils.h"

// project includes

void bot_lcmgl_line_3d(bot_lcmgl_t * lcmgl, double x_start, double y_start,
                       double z_start, double x_end, double y_end,
                       double z_end);

class FlowData {
 public:
  FlowData(const char* path_str, const char* dataset_str, bool _quiet,
           bool _verbose);
  ~FlowData();

  // setup
  bot_lcmgl_t * lcmgl_flow_vels;
  lcm_t * lcm;
  BotParam * param;
  bool quiet;
  bool verbose;

  // map origin offset (start_state)
  Eigen::Vector4f start_state;
  Eigen::Vector3f start_pos;
  Eigen::Vector3i start_ixyz;

  // maximum dimensions read in from file name (indexed from 1)
  int dim_x;
  int dim_y;
  int dim_z;

  // min and max indices from file indexed from 0
  Eigen::Vector3i ixyz_min;
  Eigen::Vector3i ixyz_max;

  // unscented transform flag
  bool ut_flag;

  // grid resolutions
  // TODO(jakeware): need to change z res to allow variation with altitude
  float res;

  // occupancy inflation radius
  int r_obs;

  // velocity
  std::vector<float> u;
  std::vector<float> s2u;
  std::vector<float> v;
  std::vector<float> s2v;
  std::vector<float> w;
  std::vector<float> s2w;
  std::vector<float> mag;

  // file register
  Eigen::VectorXi data_flags;  // u,v,w,mag,x,y,z,t

  float getU(Eigen::Vector3i ixyz);
  float getV(Eigen::Vector3i ixyz);
  float getW(Eigen::Vector3i ixyz);
  float getMag(Eigen::Vector3i ixyz);
  Eigen::Vector3f getUVW(Eigen::Vector3i ixyz);
  Eigen::Vector3f getUVWVar(Eigen::Vector3i ixyz);
  void setUVW(Eigen::Vector3i ixyz, Eigen::Vector3f vel);
  void setUVWVar(Eigen::Vector3i ixyz, Eigen::Vector3f var);
  void plotUV(int k, double flow_scale, int increment);
  void plotUVGlobal(int k, double flow_scale, int increment);

  // celltype
  std::vector<float> t;  // type
  occ_map::PixelMap<float> * occ2d;
  octomap::OcTree * occ3d;
  occ_map::PixelMap<float> * obs2d;
  octomap::OcTree * obs3d;

  void initCSpace(const char* path_str);
  float getT(Eigen::Vector3i ixyz);
  int getOcc2D(Eigen::Vector3i ixyz);
  int getOcc3D(Eigen::Vector3i ixyz);
  int getObs2D(Eigen::Vector3i ixyz);
  int getObs3D(Eigen::Vector3i ixyz);
  void fillOcc2D();
  void fillOcc3D();
  void plotOcc2D();
  void plotOcc3D();
  void fillObs2D();
  void fillObs3D();
  void plotObs2D();
  void plotObs3D();

  // grid
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;

  float getX(Eigen::Vector3i ixyz);
  float getY(Eigen::Vector3i ixyz);
  float getZ(Eigen::Vector3i ixyz);
  Eigen::Vector3f getXYZ(Eigen::Vector3i ixyz);
  Eigen::Vector3i getInd(Eigen::Vector3f pos);
  int checkMapBounds(Eigen::Vector3i ixyz);

  float getS2X(Eigen::Vector3i ixyz);
  float getS2Y(Eigen::Vector3i ixyz);
  float getS2Z(Eigen::Vector3i ixyz);
  Eigen::Vector3f getS2XYZ(Eigen::Vector3i ixyz);

  void getParams();
  int getData(const char *path_str, const char *dataset_str);
  int readHDF5(std::string fname, const char *dataset_str,
               std::vector<float> &vec);
  void dumpData(std::string fname);
};

#endif  // FLOW_PLAN_SRC_FLOW_DATA_FLOW_DATA_H_
