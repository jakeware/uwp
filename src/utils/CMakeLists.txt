# define required packages
set(UTILS_EXTERNAL_PACKAGES
	lcmgl-utils
    eigen-utils
    eigen3
)

pkg_check_modules(UTILS_EXTERNALS REQUIRED ${UTILS_EXTERNAL_PACKAGES})

# Build
set(MY_LIBRARY_NAME flow_utils)
include_directories(utils SYSTEM ${UTILS_EXTERNALS_INCLUDE_DIRS})
link_directories(${UTILS_EXTERNALS_LIBRARY_DIRS})
add_library(${MY_LIBRARY_NAME} SHARED plot_utils.cc)
target_link_libraries(${MY_LIBRARY_NAME} ${UTILS_EXTERNALS_LIBRARIES})
set_target_properties(${MY_LIBRARY_NAME} PROPERTIES 
    COMPILE_FLAGS "-std=c++11 -Wall")

# Install
install(TARGETS ${MY_LIBRARY_NAME} 
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Configure
install_pkg_config_file(${MY_LIBRARY_NAME}
  DESCRIPTION "Utilities for the flow project."
  LIBS -l${MY_LIBRARY_NAME}
  VERSION ${${MY_LIBRARY_NAME}_VERSION}
  REQUIRES ${UTILS_EXTERNAL_PACKAGES}
  CFLAGS "-std=c++11")
