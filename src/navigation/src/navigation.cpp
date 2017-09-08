#include "../include/navigation.h"

Navigation::Navigation(int argc, char *argv[])
{
    this->moveCommands = new Kinematics(argc, argv);
    this->laserMonitor = new LaserSubscriber(argc, argv);
    this->odometryMonitor = new OdometySubscriber(argc, argv);
    this->sonarMonitor = new UltrasoundSubscriber(argc, argv);
}

Navigation::~Navigation()
{
    delete moveCommands;
    delete laserMonitor;
    delete odometryMonitor;
    delete sonarMonitor;
}

void Navigation::moveMeters(float distance, float linearVel)
{
    auto timeMoving = timeToDistance(distance, linearVel);
    auto begin = ros::Time::now();
    for(ros::Time timeLoop = ros::Time::now(); (timeLoop-begin) < timeMoving; timeLoop = ros::Time::now())
    {
        /* Move */
        this->moveCommands->moveLinear(linearVel);
    }
    this->moveCommands->moveStop();
}

ros::Duration Navigation::timeToDistance(float distance, float linearVel)
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

void Navigation::spinDegrees(float angle, float angularVel)
{
    angle = angleOps::degreesToRadians(angle);
    auto timeSpinning = timeToAngle(angle, angularVel);
    ros::Time begin = ros::Time::now();
    for(ros::Time timeLoop = ros::Time::now(); (timeLoop-begin) < timeSpinning; timeLoop = ros::Time::now())
    {
        /* Move */
        this->moveCommands->moveAngular(angularVel);
    }
    this->moveCommands->moveStop();
}

ros::Duration Navigation::timeToAngle(float angle, float angularVel)
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

void Navigation::stopMoving()
{
    this->moveCommands->moveStop();
    this->odometryMonitor->printOdometry();
    this->sonarMonitor->printSonar();
    this->laserMonitor->printLaser();
}

int main(int argc, char *argv[])
{
    Navigation navigate(argc, argv);
    ros::Rate loop_rate(100);
    loop_rate.sleep();
    float vel = 0.2;
    float angVel = 0.2;
    int Time = 5;
    while(ros::ok())
    {
        char c = 0;
        std::cout << "Say the command\n-> ";
        std::cin >> c;
        switch (c)
        {
        case 'w':
            navigate.moveMeters(1, vel);
            break;
        case 'a':
            navigate.spinDegrees(90, angVel);
            break;
        case 'd':
            navigate.spinDegrees(90, -angVel);
            break;
        case 's':
            navigate.stopMoving();
            break;
        default:
            break;
        }
        if(c == 'q')
            break;
    }
}
