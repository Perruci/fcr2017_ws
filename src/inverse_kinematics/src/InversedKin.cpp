#include "../lib/InversedKin.hpp"

InversedKin::InversedKin(int argc, char *argv[])
{
   /* ROS setup */
   ros::init(argc, argv, "inverse_kinematics");
   // Declare node handle
   this -> n_ = new ros::NodeHandle;
   // Topics names
   this -> topicName    = "cmd_vel";
   // Set up global Publishers
   this -> cmd_vel_pub    = n_ -> advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
   loop_rate_ = new ros::Rate(100);
   loop_rate_ -> sleep();
}


void InversedKin::moveLinear(float speed)
{
   /* Setup Message */
   geometry_msgs::Twist vel;
   vel.linear.x = speed;
   /* Publish it */
   cmd_vel_pub.publish(vel);
   /* Complete cycle */
   this -> sleep();
   return;
}
void InversedKin::moveAngular(float angSpeed)
{
   /* Setup Message */
   geometry_msgs::Twist vel;
   vel.angular.z = angSpeed;
   /* Publish it */
   cmd_vel_pub.publish(vel);
   /* Complete cycle */
   this -> sleep();
   return;
}
void InversedKin::moveStop()
{
   /* Complete cycle */
   geometry_msgs::Twist vel_stop;
   vel_stop.linear.x = 0;
   vel_stop.angular.z = 0;
   /* Publish it */
   this -> cmd_vel_pub.publish(vel_stop);
   /* Complete cycle */
   this -> sleep();
   return;
}

void InversedKin::sleep()
{
   loop_rate_ -> sleep();
   ros::spinOnce();
}
