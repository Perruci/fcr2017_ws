#include "../include/movement/movement.h"

Movement::Movement(int argc, char *argv[])
{
    this->moveCommands = new Kinematics(argc, argv);
    this->anglePID = new PID( pid_orientation::timeStep,
                              pid_orientation::maxVel,
                              pid_orientation::minVel,
                              pid_orientation::gainP,
                              pid_orientation::gainD,
                              pid_orientation::gainI );
}

Movement::~Movement()
{
    delete moveCommands;
    delete anglePID;
}

/* Open Loop Movements */

void Movement::movementTimeLoop(ros::Duration timeMoving)
{
    auto begin = ros::Time::now();
    for(ros::Time timeLoop = ros::Time::now(); (timeLoop-begin) < timeMoving; timeLoop = ros::Time::now())
    {

    }
}

void Movement::moveMeters(float distance, float linearVel)
{
    auto timeMoving = timeToDistance(distance, linearVel);
    this->moveCommands->moveLinear(linearVel);

    this->movementTimeLoop(timeMoving);

    this->moveCommands->moveStop();
}

ros::Duration Movement::timeToDistance(float distance, float linearVel)
{
    auto timeMoving = std::abs(distance / linearVel);
    if(timeMoving < 0)
    {
        throw std::logic_error("Negative time deducted when moving");
        return ros::Duration(0);
    }
    else
        return ros::Duration(timeMoving);
}

void Movement::spinDegrees(float angle, float angularVel)
{
    angle = angleOps::degreesToRadians(angle);
    auto timeSpinning = timeToAngle(angle, angularVel);
    this->moveCommands->moveAngular(angularVel);

    this->movementTimeLoop(timeSpinning);

    this->moveCommands->moveStop();
}

ros::Duration Movement::timeToAngle(float angle, float angularVel)
{
    auto timeMoving = std::abs(angle / angularVel);
    if(timeMoving < 0)
    {
        throw std::logic_error("Negative time deducted when spinning");
        return ros::Duration(0);
    }
    else
        return ros::Duration(timeMoving);
}

void Movement::stopMoving()
{
    this->moveCommands->moveStop();
}

/* Closed Loop Movements */
/*      Control Loops       */
/* Adjust Linear Velocity and Varies the Orientation (always move forward) */
void Movement::adjust_and_run(double angularError)
{
    /* Get PID computed values */
    float omega = this->anglePID->calculate(angularError);
    /* adjust linear Velocity */
    float adjustVel = move_speeds::linear / std::pow(std::abs( omega ) + 1, 2);
    /* send command messages */
    this->moveCommands->moveAndSpin(adjustVel, omega);
}
