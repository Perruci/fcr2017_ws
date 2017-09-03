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
    OdometySubscriber();
    std::vector<double> getOdometry();

    ros::NodeHandle nh;
    ros::Subscriber msg_sub;
    double xPos, yPos, yaw;
    void odomCallBack(const nav_msgs::Odometry::ConstPtr&);
};
#endif
