cmake_minimum_required(VERSION 3.0.2)
project(aruco_lib_integration)

include(ExternalProject)
ExternalProject_Add(aruco_src
  SOURCE_DIR /aruco-3.1.12
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  -DCMAKE_BUILD_TYPE:STRING=Release
)

find_package(catkin REQUIRED)
catkin_package(LIBRARIES aruco)
