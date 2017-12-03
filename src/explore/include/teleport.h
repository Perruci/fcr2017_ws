#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"

class Teleport
{
private:
    ros::NodeHandle n;
    geometry_msgs::Pose start_pose;
    geometry_msgs::Twist start_twist;
    gazebo_msgs::ModelState modelstate;
public:
    Teleport(int argc, char *argv[]);
    void set_position(float, float);
    void set_orientation(float);
    void set_velocity_0();
    void serviceTeleport();
};
