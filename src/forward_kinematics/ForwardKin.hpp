// Forward Kinematics Class structure
#ifndef FORWARD_KIN
#define FORWARD_KIN

#include "ros/ros.h"
#include <string>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

class ForwardKin
{
private:
    // Declare node handle
    ros::NodeHandle* n_;
    // Topics names
    std::string rightStr;
    std::string leftStr;
    // Declare Publishers
    ros::Publisher right_pub;
    ros::Publisher left_pub;
    // Loop Rate
    ros::Rate* loop_rate_;

public:
    ForwardKin(int argc, char *argv[]);
    void moveLinear(float speed);
    void moveAngular(float speed);
    void moveStop();
    void sleep();
};

#endif
