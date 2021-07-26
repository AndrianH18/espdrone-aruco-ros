cmake_minimum_required(VERSION 3.0.2)
project(aruco_ros)

add_compile_options(-std=c++11)

find_package(aruco REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  roscpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  aruco_msgs
  sensor_msgs
  visualization_msgs
  aruco_lib_integration
)

generate_dynamic_reconfigure_options(
  cfg/ArucoThreshold.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aruco_ros_utils
  CATKIN_DEPENDS
    cv_bridge
    dynamic_reconfigure
    geometry_msgs
    image_transport
    roscpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    aruco_msgs
    sensor_msgs
    visualization_msgs
    aruco_lib_integration
)

include_directories(
  include
)
include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(aruco_ros_utils src/aruco_ros_utils.cpp)
target_link_libraries(aruco_ros_utils ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(simple_single src/simple_single.cpp src/aruco_ros_utils.cpp)
add_dependencies(simple_single ${PROJECT_NAME}_gencfg)
target_link_libraries(simple_single ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(map_single src/map_single.cpp src/aruco_ros_utils.cpp)
add_dependencies(map_single ${PROJECT_NAME}_gencfg)
target_link_libraries(map_single ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

#############
## Install ##
#############

install(TARGETS simple_single map_single
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS aruco_ros_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)