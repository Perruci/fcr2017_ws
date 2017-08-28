#include "ros/ros.h"
#include "lib/InversedKin.hpp"

#define PI 3.1415926535897
/* Square Functions */
void runAndStop(InversedKin&, float, float);
double timeTo90deg(float);
void spin90degrees(InversedKin&, float);
void moveSquare(InversedKin &iKin, float moveSpeed, float timeForward)
{
    for(size_t i = 0; i < 4; i++)
    {
        std::cout << "Move Forward" << '\n';
        runAndStop(iKin, moveSpeed, timeForward);
        iKin.sleep();
        std::cout << "Rotate" << '\n';
        spin90degrees(iKin, moveSpeed);
        iKin.sleep();
    }
}

/* Circle functions */
double timeTo360deg(float angular)
{
    /* Angular velocity as defined in forward_kinematic_pioneer.cpp */
    /* (v_right - v_left)/0.5 */
    double timeToAngle = 2*PI/angular;
    return timeToAngle;
}
 void moveCircle(InversedKin &iKin, float linear, float angular)
{
    double delayTime = timeTo360deg(angular);
    /* Set up movement */
    ros::Duration delay(delayTime);
    /* Move */
    iKin.moveAndSpin(linear, angular);
    /* Wait */
    delay.sleep();
    /* Stop */
    iKin.moveStop();
    
}

int main(int argc, char *argv[])
{
    /* Forward Kinematics class */
    InversedKin iKin(argc, argv);
    /* Program begins */
    float vel = 0.2;
    while(ros::ok())
    {
        char c = 0;
        std::cout << "Say the command\n-> ";
        std::cin >> c;
        switch (c)
        {
        case 'r':
            moveSquare(iKin, vel, 2);
            break;
        case 'c':
            // moveCircle(iKin, vel, 0);
            break;
        case 's':
            iKin.moveStop();
            break;
        default:
            break;
        }
        iKin.moveStop();
        if(c == 'q')
            break;
    }
}

void runAndStop(InversedKin &iKin, float vel, float moveTime)
{
    ros::Duration delay(moveTime);
    iKin.moveLinear(vel);
    delay.sleep();
    iKin.moveStop();
}

double timeTo90deg(float rotationSpeed)
{
    /* Angular velocity as defined in forward_kinematic_pioneer.cpp */
    /* (v_right - v_left)/0.5 */
    float angularVel = 2*rotationSpeed/0.5;
    double timeToAngle = (PI/2)/angularVel;
    return timeToAngle;
}

void spin90degrees(InversedKin &iKin, float rotationSpeed)
{
    double rotationTime = timeTo90deg(rotationSpeed);
    /* Set up movement */
    ros::Duration delay(rotationTime);
    iKin.moveAngular(rotationSpeed);
    delay.sleep();
    iKin.moveStop();
}
