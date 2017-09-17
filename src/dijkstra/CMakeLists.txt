# define required packages
set(EXTERNAL_PACKAGES
    eigen-utils
    eigen3
)

pkg_check_modules(EXTERNALS REQUIRED ${EXTERNAL_PACKAGES})

# Build
set(MY_LIBRARY_NAME flow_dijkstra)
include_directories(dijkstra SYSTEM ${EXTERNALS_INCLUDE_DIRS})
link_directories(${EXTERNALS_LIBRARY_DIRS})
add_library(${MY_LIBRARY_NAME} SHARED dijkstra.cc)
target_link_libraries(${MY_LIBRARY_NAME} ${EXTERNALS_LIBRARIES})
set_target_properties(${MY_LIBRARY_NAME} PROPERTIES 
    COMPILE_FLAGS "-std=c++11 -Wall")

# Install
install(TARGETS ${MY_LIBRARY_NAME} 
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Configure
install_pkg_config_file(${MY_LIBRARY_NAME}
  DESCRIPTION "Dijkstra's algorithm for flow project."
  LIBS -l${MY_LIBRARY_NAME}
  VERSION ${${MY_LIBRARY_NAME}_VERSION}
  REQUIRES ${EXTERNAL_PACKAGES}
  CFLAGS "-std=c++11")