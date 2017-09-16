#include "../include/movement/movement.h"

Movement::Movement(int argc, char *argv[])
{
    this->moveCommands = new Kinematics(argc, argv);
    this->odometryMonitor = new OdometySubscriber(argc, argv);
}

Movement::~Movement()
{
    delete moveCommands;
    delete odometryMonitor;
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
    this->odometryMonitor->printOdometry();
}

/* Closed Loop Movements */
/*      Control Loops       */
double Movement::orientationError(geometry_msgs::Point point)
{
    double diffX, diffY;
    diffX = point.x - this->odometryMonitor->X;
    diffY = point.y - this->odometryMonitor->Y;
    /* atan2 gives a standard radian value between (-pi, pi] */
    double goalOrientation = std::atan2(diffY, diffX);
    /* Get instantaneus orientation ajustment */
    double diffOrientation;
    double currentOrientation = this->odometryMonitor->Yaw;
    diffOrientation = goalOrientation - currentOrientation;
    diffOrientation = angleOps::constrainAngle(diffOrientation);
    return(diffOrientation);
}

double Movement::locationError(geometry_msgs::Point point)
{
    double diffX, diffY;
    diffX = point.x - this->odometryMonitor->X;
    diffY = point.y - this->odometryMonitor->Y;
    double locationError;
    locationError = sqrt(pow(diffX,2) + pow(diffY,2));
    return locationError;
}

/* Adjust Linear Velocity and Varies the Orientation (always move forward) */
void Movement::go_to_goal(geometry_msgs::Point point, float vel)
{
    /* Tolerance for both orientation and localization */
    float toleranceAngle = 0.1;
    float toleranceModule = 0.1;

    /* PID setup */
    float Kp = 1;
    float Ki = 0.0;
    float Kd = 0.0;
    double oldError = 0;
    double accumError = 0;
    float maxVel = MAX_ANG_VEL;
    /* Loop until orientation is adjusted and location is reached */
    double error = orientationError(point);
    double positionError = locationError(point);
    while(std::abs(positionError) > toleranceModule)
    {
        float PID = error * Kp + Ki * accumError + Kd * (error - oldError);
        float omega = PID;
        float adjustVel = vel / std::pow(std::abs( omega ) + 1, 2);
        this->moveCommands->moveAndSpin(adjustVel, omega);
        oldError = error;
        accumError = error + accumError;
        error = orientationError(point);
        positionError = locationError(point);
    }
    /* End Movement */
    this->stopMoving();
}
