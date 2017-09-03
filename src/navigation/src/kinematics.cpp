#include "../include/movement/kinematics.h"

Kinematics::Kinematics(int argc, char *argv[])
{
   /* ROS setup */
   ros::init(argc, argv, "inverse_kinematics");
   // Declare node handle
   this -> n_ = new ros::NodeHandle;
   // Topics names
   this -> topicName = "cmd_vel";
   // Set up global Publishers
   this -> cmd_vel_pub  = n_ -> advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
   zeroMsg();
   loop_rate_ = new ros::Rate(100);
   loop_rate_ -> sleep();
}

Kinematics::~Kinematics()
{
    delete n_;
    delete loop_rate_;
}

void Kinematics::moveAndSpin(float linear, float angular)
{
   /* Setup Message */
   this->zeroMsg();
   msg_vel.linear.x = linear;
   msg_vel.angular.z = angular;
   this->sendMsg();
   return;
}

void Kinematics::moveLinear(float speed)
{
   /* Setup Message */
   this->zeroMsg();
   msg_vel.linear.x = speed;
   this->sendMsg();
   return;
}
void Kinematics::moveAngular(float angSpeed)
{
   /* Setup Message */
   this->zeroMsg();
   msg_vel.angular.z = angSpeed;
   this->sendMsg();
   return;
}
void Kinematics::moveStop()
{
   this->zeroMsg();
   this->sendMsg();
   return;
}

// int main(int argc, char *argv[])
// {
    // Kinematics move(argc, argv);
    // ros::Rate loop_rate(100);
    // loop_rate.sleep();
    // float vel = 0.2;
    // float angVel = 0.1;
    // while(ros::ok())
    // {
        // char c = 0;
        // std::cout << "Say the command\n-> ";
        // std::cin >> c;
        // switch (c)
        // {
        // case 'w':
            // move.moveLinear(vel);
            // break;
        // case 'a':
            // move.moveAngular(angVel);
            // break;
        // case 'd':
            // move.moveAngular(-angVel);
            // break;
        // case 's':
            // move.moveStop();
            // break;
        // default:
            // break;
        // }
        // if(c == 'q')
            // break;
    // }
// }
