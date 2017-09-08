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

/* Movement Functions */

void Navigation::movementTimeLoop(ros::Duration timeMoving)
{
    auto begin = ros::Time::now();
    for(ros::Time timeLoop = ros::Time::now(); (timeLoop-begin) < timeMoving; timeLoop = ros::Time::now())
    {

    }
}

void Navigation::moveMeters(float distance, float linearVel)
{
    auto timeMoving = timeToDistance(distance, linearVel);
    this->moveCommands->moveLinear(linearVel);

    this->movementTimeLoop(timeMoving);

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
    this->moveCommands->moveAngular(angularVel);

    this->movementTimeLoop(timeSpinning);

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
    // this->laserMonitor->printLaser();
}

/* Navigation Functions */
double Navigation::orientationError(geometry_msgs::Point point)
{
    double diffX, diffY;
    diffX = point.x - this->odometryMonitor->X;
    diffY = point.y - this->odometryMonitor->Y;
    /* Get instantaneus orientation ajustment */
    double diffOrientation;
    diffOrientation = std::atan2(diffY, diffX) - this->odometryMonitor->Yaw;
    return(diffOrientation);
}

double Navigation::locationError(geometry_msgs::Point point)
{
    double diffX, diffY;
    diffX = point.x - this->odometryMonitor->X;
    diffY = point.y - this->odometryMonitor->Y;
    double locationError;
    locationError = sqrt(pow(diffX,2) + pow(diffY,2));
    return locationError;
}

void Navigation::moveToPosition(geometry_msgs::Point point, float vel)
{
    /* Nonstop move to position */
    double diffOrientation = orientationError(point);
    /* Tolerance for both orientation and localization */
    float toleranceAngle = 0.1;
    /* Loop until orientation is adjusted */
    for(; diffOrientation > toleranceAngle; diffOrientation = orientationError(point))
    {
        float adjustVel = diffOrientation > 0? vel: -vel;
        this->moveCommands->moveAndSpin(vel, adjustVel);
    }
    std::cout << "Angle Adjustment Completed" << '\n';
    /* Loop until position is adjusted */
    double diffPosition = locationError(point);
    double toleranceModule = 0.1;
    while(diffPosition > toleranceModule)
    {
        /* Stop angular movement, keeps only linear */
        this->moveCommands->moveLinear(vel);
        diffPosition = locationError(point);
        std::cout << "Diff position " << diffPosition << '\n';
    }
    this->stopMoving();
}


int main(int argc, char *argv[])
{
    Navigation navigate(argc, argv);
    ros::Rate loop_rate(100);
    loop_rate.sleep();
    float vel = 0.2;
    float angVel = 0.2;
    geometry_msgs::Point point;
    point.x = 0;
    point.y = 1;
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
        case 'p':
            navigate.moveToPosition(point, vel);
            break;
        default:
            break;
        }
        if(c == 'q')
            break;
    }
}
