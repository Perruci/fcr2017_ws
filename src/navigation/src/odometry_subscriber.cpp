#include "../include/measurements/odometry_subscriber.h"

void OdometySubscriber::odomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    this->X = msg->pose.pose.position.x;
    this->Y = msg->pose.pose.position.y;
    this->Yaw = tf::getYaw(msg->pose.pose.orientation);
}

OdometySubscriber::OdometySubscriber(int argc, char *argv[])
{
    ros::init(argc, argv, "odometry_subscriber");
    /* Special callback call when using classes */
    /*                                        pointer to the callback function  object */
    this->msg_sub = nh.subscribe("/pose", 10, &OdometySubscriber::odomCallBack, this);
}

std::array<double, 3> OdometySubscriber::getOdometry()
{
    return odometryState;
}

void OdometySubscriber::printOdometry()
{
    ROS_INFO("pose: x = %lf, y = %lf, yaw = %lf", odometryState[0], odometryState[1], angleOps::radiansToDegrees(odometryState[2]));
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
