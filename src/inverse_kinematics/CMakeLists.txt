cmake_minimum_required(VERSION 2.8.3)
project(inverse_kinematics)

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
# Inversed Kinematics
add_executable(inverse_kinematics inverse_kinematics.cpp lib/InversedKin.h src/InversedKin.cpp )
target_link_libraries(inverse_kinematics ${catkin_LIBRARIES})
