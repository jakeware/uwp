# define required packages
set(PLANNER_EXTERNAL_PACKAGES
	lcmgl-utils
    eigen-utils
    eigen3
)

pkg_check_modules(PLANNER_EXTERNALS REQUIRED ${PLANNER_EXTERNAL_PACKAGES})

# Build
set(MY_LIBRARY_NAME flow_planner)
include_directories(planner SYSTEM ${PLANNER_EXTERNALS_INCLUDE_DIRS})
link_directories(${PLANNER_EXTERNALS_LIBRARY_DIRS})
add_library(${MY_LIBRARY_NAME} SHARED planner.cc)
target_link_libraries(${MY_LIBRARY_NAME} ${PLANNER_EXTERNALS_LIBRARIES} flow_dijkstra flow_astar)
set_target_properties(${MY_LIBRARY_NAME} PROPERTIES 
    COMPILE_FLAGS "-std=c++11 -Wall")

# Install
install(TARGETS ${MY_LIBRARY_NAME} 
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Configure
install_pkg_config_file(${MY_LIBRARY_NAME}
  DESCRIPTION "Planner for the flow project."
  LIBS -l${MY_LIBRARY_NAME}
  VERSION ${${MY_LIBRARY_NAME}_VERSION}
  REQUIRES ${PLANNER_EXTERNAL_PACKAGES}
  CFLAGS "-std=c++11")
