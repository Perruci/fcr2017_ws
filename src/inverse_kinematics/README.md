# Inversed Kinematics ROS Package

This node handles the movement of the robot Pioneer through the velocities coordenate of the movements.

## How to Use
Add to the CMake directive:
```
	#Inverse Kinematics
	add_executable(inverse_kinematics src/inverse_kinematics/inverse_kinematics.cpp src/inverse_kinematics/lib/InversedKin.hpp src/inverse_kinematics/src/InversedKin.cpp )
	target_link_libraries(inverse_kinematics ${catkin_LIBRARIES})

```
Then, run
```
	catkin_make
```
and, finally run the executable by:
```
	rosrun fcr2017 inversed_kinematics
```
Key commands:
* 'r': Move as a rectangle
* 'c': Move as a circle
* 's': Command to stop