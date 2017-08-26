#include "ros/ros.h"
#include <string>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

/* shame on me */
ros::Publisher right_pub;
ros::Publisher left_pub;

void moveLinear(float speed)
{
    std_msgs::Float32 vel_linear;
    vel_linear.data = speed;
    right_pub.publish(vel_linear);
    left_pub.publish(vel_linear);
    return;
}
void moveAngular(float speed)
{
    std_msgs::Float32 vel_spinPlus;
    std_msgs::Float32 vel_spinMinus;
    vel_spinPlus.data = speed;
    vel_spinMinus.data = -speed;
    right_pub.publish(vel_spinPlus);
    left_pub.publish(vel_spinMinus);
    return;
}
void moveStop()
{
    std_msgs::Float32 vel_stop;
    vel_stop.data = 0;
    right_pub.publish(vel_stop);
    left_pub.publish(vel_stop);
    return;
}

int main(int argc, char *argv[])
{
    /* ROS setup */
    ros::init(argc, argv, "forward_kinematics");
    // Declare node handle
    ros::NodeHandle n;
    // Topics names
    std::string right = "v_right";
    std::string left = "v_left";
    // Set up global Publishers
    right_pub = n.advertise<std_msgs::Float32>(right, 1000);
    left_pub = n.advertise<std_msgs::Float32>(left, 1000);
    ros::Rate loop_rate(100);
    loop_rate.sleep();
    /* Program begins */
    float vel = 0.1;
    char c;
    while(ros::ok())
    {
        std::cin >> c;
        if(c == 'l')
        {
            moveLinear(vel);
        }
        if(c == 'r')
        {
            moveAngular(vel);
        }
        if(c == 's')
        {
            moveStop();
        }
        if(c == 'q')
        {
            break;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}
