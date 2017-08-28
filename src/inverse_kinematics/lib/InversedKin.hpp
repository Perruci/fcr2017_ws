// Forward Kinematics Class structure
#ifndef INVERSED_KIN
#define INVERSED_KIN

#include "ros/ros.h"
#include <string>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

class InversedKin
{
private:
    // Declare node handle
    ros::NodeHandle* n_;
    // Topics names
    std::string topicName;
    // Declare Publishers
    ros::Publisher cmd_vel_pub;
    // Loop Rate
    ros::Rate* loop_rate_;

public:
    InversedKin(int argc, char *argv[]);
    void moveLinear(float speed);
    void moveAngular(float speed);
    void moveStop();
    void sleep();
};

#endif
