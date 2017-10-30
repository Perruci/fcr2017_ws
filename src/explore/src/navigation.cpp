#include "../include/navigation.h"

Navigation::Navigation(int argc, char *argv[])
{
    this->moveCommands = new Movement(argc, argv);
    this->odometryMonitor = new OdometrySubscriber(argc, argv);
    this->laserMonitor = new LaserSubscriber(argc, argv);
    this->sonarMonitor = new UltrasoundSubscriber(argc, argv);
    this->stopMoving();
}

Navigation::~Navigation()
{
    delete moveCommands;
    delete odometryMonitor;
    delete laserMonitor;
    delete sonarMonitor;
}

/* topological map itegration ---------------------------------- */
void Navigation::explore()
{
  return;
}

/* Sensors Processing ------------------------------------------ */
bool Navigation::setMinObstaclePoints()
{
    float maxFrontAngle = angleOps::degreesToRadians(obstacle_detection::max_front_deg);
    float minFrontAngle = angleOps::degreesToRadians(obstacle_detection::min_front_deg);

    std::vector<laser_point> frontPoints = laserMonitor->getInRange(minFrontAngle, maxFrontAngle);
    if(frontPoints.empty())
        return false;

    this->nearestOrientation = 0;
    this->nearestDistance = obstacle_detection::distance;
    size_t pointsCount = 0;
    if(frontPoints.empty())
        return false;
    size_t vecSize = frontPoints.size();
    for(size_t i = 0; i < vecSize; i++)
    {
        if(frontPoints[i][laser::distance] < nearestDistance)
        {
            nearestOrientation = frontPoints[i][laser::orientation];
            nearestDistance = frontPoints[i][laser::distance];
        }
        pointsCount += 1;
    }

    return pointsCount > 0? true : false;
}

bool Navigation::obstacleDetection(float distance)
{
    return setMinObstaclePoints();
}

/* Navigation Movements ---------------------------------------- */
double Navigation::orientationError(geometry_msgs::Point point)
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

double Navigation::orientationError(double goalOrientation)
{
    /* Get instantaneus orientation ajustment */
    double diffOrientation;
    double currentOrientation = this->odometryMonitor->Yaw;
    diffOrientation = goalOrientation - currentOrientation;
    diffOrientation = angleOps::constrainAngle(diffOrientation);
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

void Navigation::go_to_goal(geometry_msgs::Point point)
{
    this->stopMoving();
    /* Loop until orientation is adjusted and location is reached */
    double angularError = orientationError(point);
    double positionError = locationError(point);
    while(std::abs(positionError) > tolerance::location)
    {
        /* Check for obstacles */
        if(obstacleDetection())
            obstacleAvoidance();
        else
            this->moveCommands->adjust_and_run(angularError);
        /* Update errors */
        angularError = orientationError(point);
        positionError = locationError(point);
    }
    /* Stop Movement */
    this->stopMoving();
};

void Navigation::obstacleAvoidance()
{
    /* Proportional gain: orientation orthogonal to obstacle */
     double reboundAngle = nearestOrientation < 0?
        nearestOrientation + M_PI / 2 : nearestOrientation - M_PI /2 ;
     this->moveCommands->moveCommands->moveAndSpin(move_speeds::linear, reboundAngle);
}
