#include "../lib/InversedKin.h"

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
   zeroMsg();
   loop_rate_ = new ros::Rate(100);
   loop_rate_ -> sleep();
}

void InversedKin::zeroMsg()
{
   this->msg_vel.linear.x  = 0;
   this->msg_vel.linear.y  = 0;
   this->msg_vel.linear.z  = 0;
   this->msg_vel.angular.x = 0;
   this->msg_vel.angular.y = 0;
   this->msg_vel.angular.z = 0;
}

void InversedKin::moveAndSpin(float linear, float angular)
{
   /* Setup Message */
   zeroMsg();
   msg_vel.linear.x = linear;
   msg_vel.angular.z = angular;
   /* Publish it */
   cmd_vel_pub.publish(msg_vel);
   /* Complete cycle */
   this -> sleep();
   return;
}

void InversedKin::moveLinear(float speed)
{
   /* Setup Message */
   zeroMsg();
   msg_vel.linear.x = speed;
   /* Publish it */
   cmd_vel_pub.publish(msg_vel);
   /* Complete cycle */
   this -> sleep();
   return;
}
void InversedKin::moveAngular(float angSpeed)
{
   /* Setup Message */
   zeroMsg();
   msg_vel.angular.z = angSpeed;
   /* Publish it */
   cmd_vel_pub.publish(msg_vel);
   /* Complete cycle */
   this -> sleep();
   return;
}
void InversedKin::moveStop()
{
   zeroMsg();
   /* Publish it */
   this -> cmd_vel_pub.publish(msg_vel);
   /* Complete cycle */
   this -> sleep();
   return;
}

void InversedKin::sleep()
{
   loop_rate_ -> sleep();
   ros::spinOnce();
}
