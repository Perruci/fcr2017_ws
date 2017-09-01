cmake_minimum_required(VERSION 2.8.3)
project(forward_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  p2os_msgs
  tf
)

catkin_package(
 # INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Forward Kinematics
add_executable(forward_kinematics forward_kinematics.cpp lib/ForwardKin.hpp src/ForwardKin.cpp )
target_link_libraries(forward_kinematics ${catkin_LIBRARIES})
