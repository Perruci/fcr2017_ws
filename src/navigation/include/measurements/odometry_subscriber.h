#ifndef ODOMETRY_SUB_H
#define ODOMETRY_SUB_H

#include "ros/ros.h"
#include <string>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"


class OdometySubscriber{
public:
    OdometySubscriber(int argc, char *argv[]);
    std::array<double, 3> getOdometry();
    void printOdometry();
    ros::NodeHandle nh;
    ros::Subscriber msg_sub;
    std::array<double, 3> odometryState;
    void odomCallBack(const nav_msgs::Odometry::ConstPtr&);
};
#endif
