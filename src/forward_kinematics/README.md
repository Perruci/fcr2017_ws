# Forward Kinematics ROS Package

This node handles the movement of the robot Pioneer through the velocities of each of it's wheels.

## How to Use
Add to the CMake directive:
```
	# Forward Kinematics
	add_executable(forward_kinematics src/forward_kinematics/forward_kinematics.cpp src/forward_kinematics/lib/ForwardKin.hpp src/forward_kinematics/src/ForwardKin.cpp )
	target_link_libraries(forward_kinematics ${catkin_LIBRARIES})
```
Then, go to your catkin_ws Root directory
```
	cd pathTo/catkin_ws
```

Compile our changes
```
	catkin_make
```

Run the simulation
```
	roslaunch fcr2017 pioneer3at.gazebo.launch
```

and, finally run the executable by:
```
	rosrun fcr2017 forward_kinematics
```

### Key commands:
* 'r': Move as a rectangle
* 'c': Move as a circle
* 's': Command to stop