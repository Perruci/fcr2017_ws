#include "ros/ros.h"
#include <string>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "forward_kinematics");
    // Declare node handle
    ros::NodeHandle n;
    // Topics names
    std::string right = "v_right";
    std::string left = "v_left";
    // Declare Publishers
    ros::Publisher right_pub = n.advertise<std_msgs::Float32>(right, 1000);
    ros::Publisher left_pub = n.advertise<std_msgs::Float32>(left, 1000);

    ros::Rate loop_rate(100);
    loop_rate.sleep();

    std_msgs::Float32 vel_linear;
    std_msgs::Float32 vel_spinPlus;
    std_msgs::Float32 vel_spinMinus;
    std_msgs::Float32 vel_stop;
    vel_linear.data = 1.0;
    vel_spinPlus.data = 1.0;
    vel_spinMinus.data = -1.0;
    vel_stop.data = 0;

    char c;
    while(ros::ok())
    {
        std::cin >> c;
        if(c == 'l')
        {
            right_pub.publish(vel_linear);
            left_pub.publish(vel_linear);
        }
        if(c == 'r')
        {
            right_pub.publish(vel_spinPlus);
            left_pub.publish(vel_spinMinus);
        }
        if(c == 's')
        {
            right_pub.publish(vel_stop);
            left_pub.publish(vel_stop);
            right_pub.publish(vel_stop);
            left_pub.publish(vel_stop);

        }
        if(c == 'q')
        {
            break;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}
