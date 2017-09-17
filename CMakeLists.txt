cmake_minimum_required(VERSION 2.8)

# Configure CCache if available
find_program(CCACHE_FOUND ccache)
if (CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif (CCACHE_FOUND)

project(flow)
set(${PROJECT_NAME}_VERSION 1.0.0)

# Options
option(BUILD_DOCUMENTATION "Use Doxygen to create the HTML based API documentation" ON)
option(BUILD_TESTS "Use GTest to build and test libraries" ON)

option(WITH_COVERAGE "Compile with code coverage" OFF)
if (WITH_COVERAGE)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -fprofile-arcs -ftest-coverage")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -g -O0 -fprofile-arcs -ftest-coverage")
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fprofile-arcs -ftest-coverage")
endif ()

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif (NOT CMAKE_BUILD_TYPE)

# CMake includes
include(cmake/setup.cmake)
include(cmake/install.cmake)
include(cmake/matlab.cmake)

# Find PkgConfig
find_package(PkgConfig REQUIRED)

# Install headers
install(DIRECTORY src/ DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h")
  
# Include headers
include_directories(src)

add_subdirectory(src/flow_data)
add_subdirectory(src/node)
add_subdirectory(src/utils)
add_subdirectory(src/astar)
add_subdirectory(src/dijkstra)
#add_subdirectory(src/rrt)
#add_subdirectory(src/rrt_star)
add_subdirectory(src/planner)
add_subdirectory(src/flow_plan_app)
add_subdirectory(src)

# Build test target
enable_testing()
