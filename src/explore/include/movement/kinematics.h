#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "../namespaces/parameters.h"

class Kinematics
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

    inline void zeroMsg()
    {
       this->msg_vel.linear.x  = 0;
       this->msg_vel.linear.y  = 0;
       this->msg_vel.linear.z  = 0;
       this->msg_vel.angular.x = 0;
       this->msg_vel.angular.y = 0;
       this->msg_vel.angular.z = 0;
    };
    inline void sendMsg()
    {
        /* Publish it */
        cmd_vel_pub.publish(msg_vel);
        /* Complete cycle */
        this -> sleep();
    };
public:
    Kinematics(int argc, char *argv[]);
    virtual ~Kinematics();
    void moveLinear(float speed);
    void moveAngular(float speed);
    void moveAndSpin(float linear, float angular);
    void moveStop();

    inline void sleep()
    {
       loop_rate_ -> sleep();
       ros::spinOnce();
    };
};

#endif
