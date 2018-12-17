cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_ie)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(LIBRARY_OUTPUT_PATH  "${CMAKE_SOURCE_DIR}/bin")

find_package(yaml-cpp REQUIRED)
find_package(libLAS REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${YAML_CPP_INCLUDE_DIR} ${libLAS_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(include)

add_library(pc_exhibition SHARED pointcloud_expplore.cpp)
target_link_libraries(pc_exhibition yaml-cpp las)
