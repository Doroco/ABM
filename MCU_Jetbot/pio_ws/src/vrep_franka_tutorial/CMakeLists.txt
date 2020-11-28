cmake_minimum_required(VERSION 2.8.3)
project(vrep_franka_tutorial)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  actionlib_msgs
  actionlib
  control_msgs
  ros_tutorial_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  #DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
  INCLUDE_DIRS include
  #LIBRARIES #Todo
  CATKIN_DEPENDS #Todo
)

set(SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/franka_controller.cpp
)
set(HEADS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vrep_franka_tutorial/franka_controller.hpp
)
add_library(vrep_franka_control ${SRCS} ${HEADS})

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(vrep_franka_tutorial src/vrep_franka_tutorial.cpp)
target_link_libraries(vrep_franka_tutorial
   vrep_franka_control
   ${catkin_LIBRARIES}
)

#SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE Debug)
