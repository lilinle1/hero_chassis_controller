cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

add_compile_options(-std=c++14)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  roslint
  controller_interface
  hardware_interface
  forward_command_controller
  pluginlib
  control_toolbox
  geometry_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGE_DEPENDENCIES})
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/hero_chassis_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

roslint_cpp()
