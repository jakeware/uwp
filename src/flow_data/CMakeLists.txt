find_package(HDF5 REQUIRED COMPONENTS CXX HL)

# define required packages
set(FLOW_DATA_EXTERNAL_PACKAGES
        bot2-core 
        bot2-param-client
        lcmgl-utils
        eigen-utils
        eigen3
        octomap-util
)

pkg_check_modules(FLOW_DATA_EXTERNALS REQUIRED ${FLOW_DATA_EXTERNAL_PACKAGES})

# Build
set(MY_LIBRARY_NAME flow_data)
include_directories(flow_data SYSTEM ${FLOW_DATA_EXTERNALS_INCLUDE_DIRS} ${HDF5_INCLUDE_DIRS})
link_directories(${FLOW_DATA_EXTERNALS_LIBRARY_DIRS} ${HDF5_LIBRARY_DIRS})
add_library(${MY_LIBRARY_NAME} SHARED flow_data.cc)
target_link_libraries(${MY_LIBRARY_NAME} ${FLOW_DATA_EXTERNALS_LIBRARIES} ${HDF5_LIBRARIES})
set_target_properties(${MY_LIBRARY_NAME} PROPERTIES 
    COMPILE_FLAGS "-std=c++11 -Wall")

# Install
install(TARGETS ${MY_LIBRARY_NAME} 
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Configure
install_pkg_config_file(${MY_LIBRARY_NAME}
  DESCRIPTION "Data structure for the flow project."
  LIBS -l${MY_LIBRARY_NAME}
  VERSION ${${MY_LIBRARY_NAME}_VERSION}
  REQUIRES ${FLOW_DATA_EXTERNAL_PACKAGES}
  CFLAGS "-std=c++11")
