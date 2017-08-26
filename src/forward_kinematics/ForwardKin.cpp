#include "ForwardKin.hpp"

ForwardKin::ForwardKin(int argc, char *argv[])
{
    /* ROS setup */
    ros::init(argc, argv, "forward_kinematics");
    // Declare node handle
    this -> n_ = new ros::NodeHandle;
    // Topics names
    this -> rightStr    = "v_right";
    this -> leftStr     = "v_left";
    // Set up global Publishers
    this -> right_pub   = n_ -> advertise<std_msgs::Float32>(rightStr, 1000);
    this -> left_pub    = n_ -> advertise<std_msgs::Float32>(leftStr, 1000);
    loop_rate_ = new ros::Rate(100);
    loop_rate_ -> sleep();
}

void ForwardKin::moveLinear(float speed)
{
    std_msgs::Float32 vel_linear;
    vel_linear.data = speed;
    right_pub.publish(vel_linear);
    left_pub.publish(vel_linear);
    return;
}
void ForwardKin::moveAngular(float speed)
{
    std_msgs::Float32 vel_spinPlus;
    std_msgs::Float32 vel_spinMinus;
    vel_spinPlus.data = speed;
    vel_spinMinus.data = -speed;
    right_pub.publish(vel_spinPlus);
    left_pub.publish(vel_spinMinus);
    return;
}
void ForwardKin::moveStop()
{
    std_msgs::Float32 vel_stop;
    vel_stop.data = 0;
    right_pub.publish(vel_stop);
    left_pub.publish(vel_stop);
    return;
}

void ForwardKin::sleep()
{
    loop_rate_ -> sleep();
}
