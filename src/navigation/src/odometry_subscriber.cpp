#include "../include/measurements/odometry_subscriber.h"

void OdometySubscriber::odomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    xPos = msg->pose.pose.position.x;
    yPos = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

OdometySubscriber::OdometySubscriber(int argc, char *argv[])
{
    ros::init(argc, argv, "odometry_subscriber");
    /* Special callback call when using classes */
    /*                                        pointer to the callback function  object */
    this->msg_sub = nh.subscribe("/pose", 10, &OdometySubscriber::odomCallBack, this);
}

std::vector<double> OdometySubscriber::getOdometry()
{
    std::vector<double> odometryVec;
    odometryVec.push_back(yaw);
    odometryVec.push_back(yPos);
    odometryVec.push_back(xPos);
    return odometryVec;
}

void printOdometry(std::vector<double> odometryVec)
{
    ROS_INFO("pose: x = %lf, y = %lf, yaw = %lf", odometryVec.at(0), odometryVec.at(1), odometryVec.at(2));
}

// int main(int argc, char *argv[])
// {
//     OdometySubscriber odometry_sub();
//
//     ros::Rate loop_rate(100);
//     loop_rate.sleep();
//
//     while(ros::ok())
//     {
//         printOdometry(odometry_sub.getOdometry());
//         loop_rate.sleep();
//         ros::spinOnce();
//     }
// }
