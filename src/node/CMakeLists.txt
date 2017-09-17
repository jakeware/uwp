# define required packages
set(NODE_EXTERNAL_PACKAGES
	lcmgl-utils
    eigen-utils
    eigen3
)

pkg_check_modules(NODE_EXTERNALS REQUIRED ${NODE_EXTERNAL_PACKAGES})

# Build
set(MY_LIBRARY_NAME flow_node)
include_directories(node SYSTEM ${NODE_EXTERNALS_INCLUDE_DIRS})
link_directories(${NODE_EXTERNALS_LIBRARY_DIRS})
add_library(${MY_LIBRARY_NAME} SHARED node.cc)
target_link_libraries(${MY_LIBRARY_NAME} ${NODE_EXTERNALS_LIBRARIES})
set_target_properties(${MY_LIBRARY_NAME} PROPERTIES 
    COMPILE_FLAGS "-std=c++11 -Wall")

# Install
install(TARGETS ${MY_LIBRARY_NAME} 
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Configure
install_pkg_config_file(${MY_LIBRARY_NAME}
  DESCRIPTION "Node object for the flow planner."
  LIBS -l${MY_LIBRARY_NAME}
  VERSION ${${MY_LIBRARY_NAME}_VERSION}
  REQUIRES ${NODE_EXTERNAL_PACKAGES}
  CFLAGS "-std=c++11")
