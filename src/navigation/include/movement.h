#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "ros/ros.h"
#include <string>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

class Movement
{
private:
    // Declare node handle
    ros::NodeHandle* n_;
    // Topics names
    std::string topicName;
    // Declare Publishers
    ros::Publisher cmd_vel_pub;
    // Declare Message
    geometry_msgs::Twist msg_vel;
    // Loop Rate
    ros::Rate* loop_rate_;

    inline void zeroMsg();
    inline void sendMsg();
    inline void sleep();
public:
    Movement(int argc, char *argv[]);
    ~Movement();
    void moveLinear(float speed);
    void moveAngular(float speed);
    void moveAndSpin(float linear, float angular);
    void moveStop();
};

#endif
