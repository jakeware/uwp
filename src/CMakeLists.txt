find_package(HDF5 REQUIRED COMPONENTS CXX HL)

# FLOW_PLAN ####################################################################
# define required packages
set(FLOW_PLAN_EXTERNAL_PACKAGES
	lcm
	bot2-param-client
	lcmgl-utils
	eigen-utils
	eigen3
	marshall-utils
  octomap
)

set(FLOW_PLAN_PROJECT_PACKAGES
	flow_data
	flow_plan_app
	flow_planner
	flow_node
	flow_astar
	flow_utils
)

pkg_check_modules(FLOW_PLAN_EXTERNALS REQUIRED ${FLOW_PLAN_EXTERNAL_PACKAGES})

# Build
set(MY_EXECUTABLE_NAME flow_plan)
set(MY_EXECUTABLE_SRC flow_plan.cc)
include_directories(SYSTEM ${FLOW_PLAN_EXTERNALS_INCLUDE_DIRS})
link_directories(${FLOW_PLAN_EXTERNALS_LIBRARY_DIRS})
add_executable(${MY_EXECUTABLE_NAME} ${MY_EXECUTABLE_SRC})
target_link_libraries(${MY_EXECUTABLE_NAME} ${FLOW_PLAN_EXTERNALS_LIBRARIES} ${FLOW_PLAN_PROJECT_PACKAGES})
set_target_properties(${MY_EXECUTABLE_NAME} PROPERTIES 
    COMPILE_FLAGS "-std=c++11 -Wall")

# Install
install(TARGETS ${MY_EXECUTABLE_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# PLOTTER ######################################################################
# define required packages
set(PLOTTER_EXTERNAL_PACKAGES
	lcm
	bot2-core 
	bot2-param-client
	lcmgl-utils
	eigen-utils
	eigen3
	marshall-utils
	octomap-util
)

set(PLOTTER_PROJECT_PACKAGES
	flow_data
	flow_plan_app
	flow_utils
	flow_planner
	flow_node
)

pkg_check_modules(PLOTTER_EXTERNALS REQUIRED ${PLOTTER_EXTERNAL_PACKAGES})

# Build
set(MY_EXECUTABLE_NAME flow_plot)
set(MY_EXECUTABLE_SRC flow_plot.cc)
include_directories(SYSTEM ${PLOTTER_EXTERNALS_INCLUDE_DIRS})
link_directories(${PLOTTER_EXTERNALS_LIBRARY_DIRS})
add_executable(${MY_EXECUTABLE_NAME} ${MY_EXECUTABLE_SRC})
target_link_libraries(${MY_EXECUTABLE_NAME} ${PLOTTER_EXTERNALS_LIBRARIES} ${PLOTTER_PROJECT_PACKAGES})
set_target_properties(${MY_EXECUTABLE_NAME} PROPERTIES 
    COMPILE_FLAGS "-std=c++11 -Wall")

# Install
install(TARGETS ${MY_EXECUTABLE_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# GEN_MAP_ARR ##################################################################
# Build
set(MY_EXECUTABLE_NAME generate_map_arr)
set(MY_EXECUTABLE_SRC generate_map_arr.cc)
include_directories(SYSTEM ${HDF5_INCLUDE_DIRS})
link_directories(${EXTERNALS_LIBRARY_DIRS} ${HDF5_LIBRARY_DIRS})
add_executable(${MY_EXECUTABLE_NAME} ${MY_EXECUTABLE_SRC})
target_link_libraries(${MY_EXECUTABLE_NAME} ${EXTERNALS_LIBRARIES} ${HDF5_LIBRARIES})
set_target_properties(${MY_EXECUTABLE_NAME} PROPERTIES 
    COMPILE_FLAGS "-std=c++11 -Wall")

# Install
install(TARGETS ${MY_EXECUTABLE_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# GEN_MAP_VEC ##################################################################
# Build
set(MY_EXECUTABLE_NAME generate_map_vec)
set(MY_EXECUTABLE_SRC generate_map_vec.cc)
include_directories(SYSTEM ${HDF5_INCLUDE_DIRS})
link_directories(${EXTERNALS_LIBRARY_DIRS} ${HDF5_LIBRARY_DIRS})
add_executable(${MY_EXECUTABLE_NAME} ${MY_EXECUTABLE_SRC})
target_link_libraries(${MY_EXECUTABLE_NAME} ${EXTERNALS_LIBRARIES} ${HDF5_LIBRARIES})
set_target_properties(${MY_EXECUTABLE_NAME} PROPERTIES 
    COMPILE_FLAGS "-std=c++11 -Wall")

# Install
install(TARGETS ${MY_EXECUTABLE_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# MAT2HDF5 #####################################################################
find_package(Matlab)

if(Matlab_FOUND)
  # include directories
  include_directories(/usr/local/MATLAB/R2014a/extern/include)

  # define required header files
  set(HEADERS
	  src/flow_mat2hdf5.hpp
	  src/mat2hdf5.h
	  src/matlab_helpers.h
    )

  # define required source files
  set(SOURCES
	  flow_mat2hdf5.cc
	  mat2hdf5/mat2hdf5.c
	  mat2hdf5/matlab_helpers.c
	  /usr/local/MATLAB/R2014a/extern/examples/mex/explore.c
    )

  # Find pkgconfig packages
  set(MAT2HDF5_EXTERNAL_PACKAGES
	  lcm
	  bot2-core 
	  bot2-param-client
	  lcmgl-utils
	  eigen3
	  marshall-utils)

  pkg_check_modules(MAT2HDF5_EXTERNALS REQUIRED ${MAT2HDF5_EXTERNAL_PACKAGES})

  # Set RPATH
  # use, i.e. don't skip the full RPATH for the build tree
  SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

  # when building, don't use the install RPATH already
  # (but later on when installing)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

  SET(CMAKE_INSTALL_RPATH "${MATLAB_LIBRARY_DIRS}")

  # add the automatically determined parts of the RPATH
  # which point to directories outside the build tree to the install RPATH
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

  # the RPATH to be used when installing, but only if it's not a system directory
  LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
  IF("${isSystemDir}" STREQUAL "-1")
    SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  ENDIF("${isSystemDir}" STREQUAL "-1")

  # Build
  set(MY_EXECUTABLE_NAME flow_mat2hdf5)
  set(MY_EXECUTABLE_SRC ${SOURCES})
  include_directories(SYSTEM ${MAT2HDF5_EXTERNALS_INCLUDE_DIRS} ${MATLAB_INCLUDE_DIR} ${HDF5_INCLUDE_DIRS})
  link_directories(${MAT2HDF5_EXTERNALS_LIBRARY_DIRS} ${MATLAB_LIBRARY_DIRS} ${MATLAB_LIBRARY_DIRS} ${HDF5_LIBRARY_DIRS})
  add_executable(${MY_EXECUTABLE_NAME} ${MY_EXECUTABLE_SRC})
  target_link_libraries(${MY_EXECUTABLE_NAME} ${MAT2HDF5_EXTERNALS_LIBRARIES} ${MATLAB_MEX_LIBRARY} ${MATLAB_MX_LIBRARY} ${MATLAB_MAT_LIBRARY} ${HDF5_LIBRARIES})
  set_target_properties(${MY_EXECUTABLE_NAME} PROPERTIES 
    COMPILE_FLAGS "-std=c++11 -Wall -fPIC")

  # Install
  install(TARGETS ${MY_EXECUTABLE_NAME}
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
endif(Matlab_FOUND)
