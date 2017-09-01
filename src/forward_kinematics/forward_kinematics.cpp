#include "ros/ros.h"
#include "lib/ForwardKin.h"

#define PI 3.1415926535897
/* Square Functions */
void runAndStop(ForwardKin&, float, float);
double timeTo90deg(float);
void spin90degrees(ForwardKin&, float);
void moveSquare(ForwardKin &fKin, float moveSpeed, float timeForward)
{
    for(size_t i = 0; i < 4; i++)
    {
        std::cout << "Move Forward" << '\n';
        runAndStop(fKin, moveSpeed, timeForward);
        fKin.sleep();
        std::cout << "Rotate" << '\n';
        spin90degrees(fKin, moveSpeed);
        fKin.sleep();
    }
}

/* Circle functions */
double timeTo360deg(float velR, float velL)
{
    /* Angular velocity as defined in forward_kinematic_pioneer.cpp */
    /* (v_right - v_left)/0.5 */
    float angularVel = (velR - velL)/0.5;
    double timeToAngle = 2*PI/angularVel;
    return timeToAngle;
}
void moveCircle(ForwardKin &fKin, float velR, float velL)
{
    double delayTime = timeTo360deg(velR, velL);
    /* Set up movement */
    ros::Duration delay(delayTime);
    /* Move */
    fKin.setSpeedLeft(velL);
    fKin.setSpeedRight(velR);
    /* Wait */
    delay.sleep();
    /* Stop */
    fKin.moveStop();

}

int main(int argc, char *argv[])
{
    /* Forward Kinematics class */
    ForwardKin fKin(argc, argv);
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
            moveSquare(fKin, vel, 2);
            break;
        case 'c':
            moveCircle(fKin, vel, 0);
            break;
        case 's':
            fKin.moveStop();
            break;
        default:
            break;
        }
        fKin.moveStop();
        if(c == 'q')
            break;
    }
}

void runAndStop(ForwardKin &fKin, float vel, float moveTime)
{
    ros::Duration delay(moveTime);
    fKin.moveLinear(vel);
    delay.sleep();
    fKin.moveStop();
}

double timeTo90deg(float rotationSpeed)
{
    /* Angular velocity as defined in forward_kinematic_pioneer.cpp */
    /* (v_right - v_left)/0.5 */
    float angularVel = 2*rotationSpeed/0.5;
    double timeToAngle = (PI/2)/angularVel;
    return timeToAngle;
}

void spin90degrees(ForwardKin &fKin, float rotationSpeed)
{
    double rotationTime = timeTo90deg(rotationSpeed);
    /* Set up movement */
    ros::Duration delay(rotationTime);
    fKin.moveAngular(rotationSpeed);
    delay.sleep();
    fKin.moveStop();
}
