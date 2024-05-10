cmake_minimum_required(VERSION 2.8)
project(pose_graph)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")  # Enable C++17

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# g2o
find_package(g2o REQUIRED)
find_package(fmt REQUIRED)

# Executable for pose_graph using SE3
add_executable(pose_graph_g2o_SE3 pose_graph_g2o_SE3.cpp)
target_link_libraries(pose_graph_g2o_SE3
    g2o_core g2o_stuff g2o_types_slam3d ${CHOLMOD_LIBRARIES} fmt::fmt
)
