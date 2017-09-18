#include "../include/navigation.h"

Navigation::Navigation(int argc, char *argv[])
{
    this->moveCommands = new Movement(argc, argv);
    this->odometryMonitor = new OdometySubscriber(argc, argv);
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

/* Sensors Processing ------------------------------------------ */
bool Navigation::setMinObstaclePoints()
{
    float maxFrontAngle = angleOps::degreesToRadians(obstacle_detection::max_front_deg);
    float minFrontAngle = angleOps::degreesToRadians(obstacle_detection::min_front_deg);

    std::vector<laser_point> frontPoints = laserMonitor->getInRange(minFrontAngle, maxFrontAngle);
    if(frontPoints.empty())
        return false;

    /* TODO adapt to recursive mean */
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

/* Bubble Rebound Obstacle Avoidance */
/* Inpired by https://pdfs.semanticscholar.org/519e/790c8477cfb1d1a176e220f010d5ec5b1481.pdf */
bool Navigation::bubleRebound(float distance)
{
    float maxObstacleAngle = angleOps::degreesToRadians(obstacle_detection::max_obstacle_deg);
    float minObstacleAngle = angleOps::degreesToRadians(obstacle_detection::min_obstacle_deg);
    float maxFrontAngle = angleOps::degreesToRadians(obstacle_detection::max_front_deg);
    float minFrontAngle = angleOps::degreesToRadians(obstacle_detection::min_front_deg);

    std::vector<laser_point> frontPoints = laserMonitor->getRanges(minFrontAngle, maxFrontAngle);
    if(frontPoints.empty())
        return false;

    double sumOrientationDistance = 0;
    double sumDistance = 0;
    double minDistance = obstacle_detection::max_range;
    size_t vecSize = frontPoints.size();
    for(size_t i = 0; i < vecSize; i++)
    {
        /* Sum orientation and distance product */
        /* normalize big distances */
        sumOrientationDistance += frontPoints[i][laser::orientation] * frontPoints[i][laser::distance];
        sumDistance += frontPoints[i][laser::distance];
        /* For obstacle detection consider only max_obstacle_deg and min_obstacle_deg*/
        if(frontPoints[i][laser::orientation] > minObstacleAngle)
            if(frontPoints[i][laser::orientation] < maxObstacleAngle)
                if(minDistance > frontPoints[i][laser::distance])
                    minDistance = frontPoints[i][laser::distance];
    }
    this->reboundAngle = sumOrientationDistance / sumDistance;

    return minDistance < distance? true : false;
}

bool Navigation::obstacleDetection(float distance)
{
    return setMinObstaclePoints();

    // return bubleRebound();
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
     std::cout << "Obstacle detected!" << '\n';
     double reboundAngle = nearestOrientation < 0?
        nearestOrientation + M_PI / 2 : nearestOrientation - M_PI /2 ;
     std::cout << "Rebound angle: " << reboundAngle << '\n';
     this->moveCommands->moveCommands->moveAndSpin(move_speeds::linear, reboundAngle);
}
