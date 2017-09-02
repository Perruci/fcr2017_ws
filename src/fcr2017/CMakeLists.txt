cmake_minimum_required(VERSION 2.8.3)
project(fcr2017)

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

add_executable(gazebo_sonar gazebo_sonar.cpp)
target_link_libraries(gazebo_sonar ${catkin_LIBRARIES})

add_executable(publisher3at publisher3at.cc)
target_link_libraries(publisher3at ${catkin_LIBRARIES})

add_executable(pose pose.cpp)
target_link_libraries(pose ${catkin_LIBRARIES})

add_executable(forward_kinematics_pioneer forward_kinematics_pioneer.cpp)
target_link_libraries(forward_kinematics_pioneer ${catkin_LIBRARIES})

#add_executable(pwm_quad pwm_quad.cpp)
#target_link_libraries(pwm_quad ${catkin_LIBRARIES})
