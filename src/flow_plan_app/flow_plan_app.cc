// Copyright 2015 Jake Ware

// INCLUDES
#include "./flow_plan_app.h"

App::App() {
  // setup lcm
  lcm = bot_lcm_get_global(NULL);
  param = bot_param_get_global(lcm, 0);
  lstore = lcmgl_utils::LcmglStore(lcm);

  // default settings
  verbose = false;
  quiet = false;
  debug = false;
  plot = false;
  publish = false;
  dump = false;
  rand = false;
  filename_flag = false;
  naive_flag = false;
  vel_a_cap_flag = false;
  test_sg = false;

  // visualization configuration
  flow_scale = 0.5;
  drag_scale = 2.0;
  flow_increment = 1;
  sg_scale = 1;

  // position constraints
  z_index = 4;

  // velocity constraints

  vel_g_step = 0.5;
  vel_a_max = 10;

  // state variables
  start_state.setZero();
  start_pos.setZero();
  start_ixyz.setZero();

  goal_state.setZero();
  goal_pos.setZero();
  goal_ixyz.setZero();

  // bounds
  ixyz_min.setZero();
  ixyz_max.setZero();
  pos_min.setZero();
  pos_max.setZero();
  state_min.setZero();
  state_max.setZero();
  vel_g_min = 0.5;
  vel_g_max = 8;

  // discretization
  x_step = 0;
  y_step = 0;
  z_step = 0;
  vel_g_step = 0;

  // number of random trials
  rand_count = 0;
  path_num = 0;
  min_dist = 10;

  // parameters
  g = 9.81;
  mass = 2;
  alpha = 1;

  // power profile
  pow_prof.setZero(5);

  // filename
  filename = "./";

  // rand_r seed
  srand(time(NULL));  // do we need to do this?
  seed = time(NULL);

  getParams();
}

App::~App() {
  // Nothing
}

void App::getParams() {
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

  // goal state
  double goal_state_param[4];
  char goal_state_key[1024];
  const char * goal_state_string = "flow-plan.goal_state";
  snprintf(goal_state_key, sizeof(goal_state_key), "%s", goal_state_string);
  int goal_state_size = bot_param_get_double_array(param, goal_state_key,
                                                   goal_state_param, 4);
  if (goal_state_size != 4) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            goal_state_key);
  } else {
    goal_state(0) = goal_state_param[0];
    goal_state(1) = goal_state_param[1];
    goal_state(2) = goal_state_param[2];
    goal_state(3) = goal_state_param[3];
  }

  // ixyz min
  double ixyz_min_param[3];
  char ixyz_min_key[1024];
  const char * ixyz_min_string = "flow-plan.ixyz_min";
  snprintf(ixyz_min_key, sizeof(ixyz_min_key), "%s", ixyz_min_string);
  int ixyz_min_size = bot_param_get_double_array(param, ixyz_min_key,
                                                 ixyz_min_param, 3);
  if (ixyz_min_size != 3) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            ixyz_min_key);
  } else {
    ixyz_min(0) = ixyz_min_param[0];
    ixyz_min(1) = ixyz_min_param[1];
    ixyz_min(2) = ixyz_min_param[2];
  }

  // ixyz max
  double ixyz_max_param[3];
  char ixyz_max_key[1024];
  const char * ixyz_max_string = "flow-plan.ixyz_max";
  snprintf(ixyz_max_key, sizeof(ixyz_max_key), "%s", ixyz_max_string);
  int ixyz_max_size = bot_param_get_double_array(param, ixyz_max_key,
                                                 ixyz_max_param, 3);
  if (ixyz_max_size != 3) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            ixyz_max_key);
  } else {
    ixyz_max(0) = ixyz_max_param[0];
    ixyz_max(1) = ixyz_max_param[1];
    ixyz_max(2) = ixyz_max_param[2];
  }

  // alpha
  double alpha_param;
  char alpha_key[1024];
  const char * alpha_string = "flow-plan.alpha";
  snprintf(alpha_key, sizeof(alpha_key), "%s", alpha_string);
  int alpha_size = bot_param_get_double(param, alpha_key, &alpha_param);
  if (alpha_size < 0) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            alpha_key);
  } else {
    alpha = alpha_param;
  }

  // z_index
  double z_index_param;
  char z_index_key[1024];
  const char * z_index_string = "flow-plan.z_index";
  snprintf(z_index_key, sizeof(z_index_key), "%s", z_index_string);
  int z_index_size = bot_param_get_double(param, z_index_key, &z_index_param);
  if (z_index_size < 0) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            z_index_key);
  } else {
    z_index = z_index_param;
  }

  // flow_scale
  double flow_scale_param;
  char flow_scale_key[1024];
  const char * flow_scale_string = "flow-plan.flow_scale";
  snprintf(flow_scale_key, sizeof(flow_scale_key), "%s", flow_scale_string);
  int flow_scale_size = bot_param_get_double(param, flow_scale_key,
                                             &flow_scale_param);
  if (flow_scale_size < 0) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            flow_scale_key);
  } else {
    flow_scale = flow_scale_param;
  }

  // vel_g_min
  double vel_g_min_param;
  char vel_g_min_key[1024];
  const char * vel_g_min_string = "flow-plan.vel_g_min";
  snprintf(vel_g_min_key, sizeof(vel_g_min_key), "%s", vel_g_min_string);
  int vel_g_min_size = bot_param_get_double(param, vel_g_min_key,
                                            &vel_g_min_param);
  if (vel_g_min_size < 0) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            vel_g_min_key);
  } else {
    vel_g_min = vel_g_min_param;
  }

  // vel_g_max
  double vel_g_max_param;
  char vel_g_max_key[1024];
  const char * vel_g_max_string = "flow-plan.vel_g_max";
  snprintf(vel_g_max_key, sizeof(vel_g_max_key), "%s", vel_g_max_string);
  int vel_g_max_size = bot_param_get_double(param, vel_g_max_key,
                                            &vel_g_max_param);
  if (vel_g_max_size < 0) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            vel_g_max_key);
  } else {
    vel_g_max = vel_g_max_param;
  }

  // vel_g_step
  double vel_g_step_param;
  char vel_g_step_key[1024];
  const char * vel_g_step_string = "flow-plan.vel_g_step";
  snprintf(vel_g_step_key, sizeof(vel_g_step_key), "%s", vel_g_step_string);
  int vel_g_step_size = bot_param_get_double(param, vel_g_step_key,
                                             &vel_g_step_param);
  if (vel_g_step_size < 0) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            vel_g_step_key);
  } else {
    vel_g_step = vel_g_step_param;
  }

  // vel_a_max
  double vel_a_max_param;
  char vel_a_max_key[1024];
  const char * vel_a_max_string = "flow-plan.vel_a_max";
  snprintf(vel_a_max_key, sizeof(vel_a_max_key), "%s", vel_a_max_string);
  int vel_a_max_size = bot_param_get_double(param, vel_a_max_key,
                                            &vel_a_max_param);
  if (vel_a_max_size < 0) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            vel_a_max_key);
  } else {
    vel_a_max = vel_a_max_param;
  }

  // min_dist
  double min_dist_param;
  char min_dist_key[1024];
  const char * min_dist_string = "flow-plan.min_dist";
  snprintf(min_dist_key, sizeof(min_dist_key), "%s", min_dist_string);
  int min_dist_size = bot_param_get_double(param, min_dist_key,
                                           &min_dist_param);
  if (min_dist_size < 0) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            min_dist_key);
  } else {
    min_dist = min_dist_param;
  }

  // power profile
  double pow_prof_param[5];
  char pow_prof_key[1024];
  const char * pow_prof_string = "flow-plan.pow_prof";
  snprintf(pow_prof_key, sizeof(pow_prof_key), "%s", pow_prof_string);
  int pow_prof_size = bot_param_get_double_array(param, pow_prof_key,
                                                 pow_prof_param, 5);
  if (pow_prof_size != 5) {
    fprintf(stderr, "Error: Missing or funny param value: '%s'.  Using default "
            "value.\n",
            pow_prof_key);
  } else {
    pow_prof(0) = pow_prof_param[0];
    pow_prof(1) = pow_prof_param[1];
    pow_prof(2) = pow_prof_param[2];
    pow_prof(3) = pow_prof_param[3];
    pow_prof(4) = pow_prof_param[4];
  }

  return;
}

