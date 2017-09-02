#include "hector_uav_msgs/MotorPWM.h"
#include "ros/ros.h"
#include <stdint.h>
#include <vector>

std::vector<unsigned char> pwmsHigh (4, 255); // MAX POWER
std::vector<unsigned char> pwmsLow (4, 0); // MIN POWER

int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "teste");
    ros::NodeHandle n;
    ros::Publisher motor_commander_1 = n.advertise<hector_uav_msgs::MotorPWM>("/motor_pwm", 100);   
    ros::Rate loop_rate(50);
    while (ros::ok()) // Check for ctrl+c and ROS
    {
      hector_uav_msgs::MotorPWM msg1;
      msg1.pwm = pwmsHigh;
      motor_commander_1.publish(msg1); // Publish msg
      hector_uav_msgs::MotorPWM msg1Stop;   
      //motor_commander_1.publish(msg1Stop);

      ros::spinOnce();
      loop_rate.sleep();
    }
    return 0;
}
