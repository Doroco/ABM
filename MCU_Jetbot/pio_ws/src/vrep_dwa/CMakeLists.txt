cmake_minimum_required(VERSION 2.8.3)
project(vrep_dwa)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  actionlib_msgs
  actionlib
  control_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  #DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
  INCLUDE_DIRS include
  #LIBRARIES #Todo
  CATKIN_DEPENDS #Todo
)

set(SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/vrep_dwa_control.cpp
)
set(HEADS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/vrep_dwa/vrep_dwa_control.hpp
)
add_library(vrep_dwa_control ${SRCS} ${HEADS})

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(vrep_dwa src/vrep_dwa.cpp)
target_link_libraries(vrep_dwa
   vrep_dwa_control
   ${catkin_LIBRARIES}
)

#SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE Debug)
