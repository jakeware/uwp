# define required packages
set(APP_EXTERNAL_PACKAGES
	lcm
	bot2-core 
    bot2-param-client
	lcmgl-utils
	eigen-utils
	eigen3
)

pkg_check_modules(APP_EXTERNALS REQUIRED ${APP_EXTERNAL_PACKAGES})

# Build
set(MY_LIBRARY_NAME flow_plan_app)
include_directories(flow_plan_app SYSTEM ${APP_EXTERNALS_INCLUDE_DIRS})
link_directories(${APP_EXTERNALS_LIBRARY_DIRS})
add_library(${MY_LIBRARY_NAME} SHARED flow_plan_app.cc)
target_link_libraries(${MY_LIBRARY_NAME} ${APP_EXTERNALS_LIBRARIES})
set_target_properties(${MY_LIBRARY_NAME} PROPERTIES 
    COMPILE_FLAGS "-std=c++11 -Wall")

# Install
install(TARGETS ${MY_LIBRARY_NAME} 
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Configure
install_pkg_config_file(${MY_LIBRARY_NAME}
  DESCRIPTION "Helper data structure for the flow project."
  LIBS -l${MY_LIBRARY_NAME}
  VERSION ${${MY_LIBRARY_NAME}_VERSION}
  REQUIRES ${APP_EXTERNAL_PACKAGES}
  CFLAGS "-std=c++11")
