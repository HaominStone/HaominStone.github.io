cmake_minimum_required(VERSION 3.10)

PROJECT(LOOP_CLOSURE)
SET(CMAKE_CXX_STANDARD 14)

#version number
set( LOOP_CLOSURE_VERSION_MAJOR 1 )
set( LOOP_CLOSURE_VERSION_MINOR 0 )

#set build type
set(CMAKE_BUILD_TYPE Debug)

#add path for find scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

#include CTest so we can run unit tests with CMake
include(CTest)

#configure a header file to pass CMake settings
configure_file(
  "${PROJECT_SOURCE_DIR}/LC_config.h.in"
  "${PROJECT_BINARY_DIR}/LC_config.h")

#add binary tree to search path so we can find BA_config.h
include_directories("${PROJECT_BINARY_DIR}")

## Major project
add_subdirectory(test)
include_directories(/usr/local/include/eigen3)
include_directories(/usr/include/eigen3)

set(LP_SRC bag_of_words/bow_tools.cpp
           loop_closure/LC_tools.cpp 
           vo/VO_tools.cpp
           ba/BA_tools.cpp)
add_library(lp_lib ${LP_SRC})
