#ifndef ODOMETRY_SUB_H
#define ODOMETRY_SUB_H

#include "../angleOps.h"
#include "ros/ros.h"
#include <string>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"

class OdometySubscriber{
public:
    OdometySubscriber(int argc, char *argv[]);
    ros::NodeHandle nh;
    ros::Subscriber msg_sub;
    std::array<double, 3> odometryState;
    /* Setup odometry state alliases */
    double& X = odometryState[0];
    double& Y = odometryState[1];
    double& Yaw = odometryState[2];
    std::array<double, 3> getOdometry();
    void printOdometry();
    void odomCallBack(const nav_msgs::Odometry::ConstPtr&);
};
#endif
