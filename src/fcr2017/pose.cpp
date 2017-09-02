#include "ros/ros.h"
#include <string>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"

void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    double x,y, yaw;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);

    ROS_INFO("pose: x = %lf, y = %lf, yaw = %lf", x, y, yaw);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pose");
    ros::NodeHandle n;
    ros::Subscriber odom_sub = n.subscribe("/pose", 10, odomCallBack);

    ros::Rate loop_rate(100);
    loop_rate.sleep();

    while(ros::ok())
    {

        loop_rate.sleep();
        ros::spinOnce();
    }
}
