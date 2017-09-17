# Macros to simplify using gtest.
# To enable macros, just 'include(tools/tests.cmake)'
# 
# Author: wrvb
# Date: Fri Oct 17 18:55:00 EDT 2014
# Copyright 2014 RRG CSAIL MIT, all rights reserved

enable_testing()
#add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})

# add_gtest(name SOURCES)
# Define a makefile target, linking all files in ${SOURCES} as well as the gtest source. 
# Target will be named ${name}, and linker and compiler flags can be set using that name.
# Function assumes that test/gtest includes the fused gtest sources.
function(add_gtest test_name)
    if (${ARGC} LESS 1) 
        message(WARNING "add_gtest called with no arguments.\n"
        "USAGE: add_gtest(name source1 source2 ...)")
        return()
    elseif (${ARGC} LESS 2)
        message(WARNING "add_gtest trying to build ${test_name} without any sources, skipping invocation.\n"
        "USAGE: add_gtest(name source1 source2 ...)")
        return()
    endif()

    list(REMOVE_AT ARGV 0)

    add_executable(${test_name} ${ARGV} ${PROJECT_SOURCE_DIR}/test/gtest/gtest-all.cc)
    target_link_libraries(${test_name} pthread)
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction(add_gtest)

# add_all_tests()
# Define an makefile target for all test source files in test/
# Target will be named test_PROJECT_NAME, and linker and compiler flags can be set using that name.
# Function assumes that test/gtest includes the fused gtest sources.
function(add_all_tests)
    if (${ARGC} GREATER 0)
        message(WARNING "add_all_tests passed extra arguments, ignoring...\n"
            "USAGE: add_all_tests()")
    endif()

    SET(test_name "test_${CMAKE_PROJECT_NAME}")
    file(GLOB TEST_SOURCES "${PROJECT_SOURCE_DIR}/test/*.cc")

    add_executable(${test_name} ${TEST_SOURCES} ${PROJECT_SOURCE_DIR}/test/gtest/gtest-all.cc)
    target_link_libraries(${test_name} pthread)
    add_test(NAME ${test_name} COMMAND ${test_name})
endfunction(add_all_tests)

# add_each_test()
# Define a separate makefile target for each test source file in test/
# Targets will have the same name as the source file (with the extension stripped).  Linker and compiler flags can be set using that name.
# Function assumes that test/gtest includes the fused gtest sources.
function(add_each_test)
    if (${ARGC} GREATER 0)
        message(WARNING "add_each_test passed extra arguments, ignoring...\n"
            "USAGE: add_each_test()")
    endif()
    file(GLOB TEST_SOURCES "${PROJECT_SOURCE_DIR}/test/*.cc")
    foreach(source ${TEST_SOURCES})
        get_filename_component(test_name ${source} NAME_WE)
        add_gtest(${test_name} ${source})
    endforeach()
endfunction(add_each_test)

