#include "../include/navigation.h"

Navigation::Navigation(int argc, char *argv[])
{
    this->moveCommands = new Movement(argc, argv);
    this->odometryMonitor = new OdometySubscriber(argc, argv);
    this->laserMonitor = new LaserSubscriber(argc, argv);
    this->sonarMonitor = new UltrasoundSubscriber(argc, argv);
}

Navigation::~Navigation()
{
    delete moveCommands;
    delete odometryMonitor;
    delete laserMonitor;
    delete sonarMonitor;
}

/* Sensors Processing ------------------------------------------ */
bool Navigation::setMeanObstaclePoints(std::vector<laser_point> &frontPoints)
{
    /* TODO adapt to recursive mean */
    this->meanOrientation = 0;
    this->meanDistance = 0;
    size_t pointsCount = 0;
    if(frontPoints.empty())
        return false;
    size_t vecSize = frontPoints.size();
    for(size_t i = 0; i < vecSize; i++)
    {
        meanOrientation += frontPoints[i][laser::orientation];
        meanDistance += frontPoints[i][laser::distance];
        pointsCount += 1;
    }
    /* Compute mean values */
    meanOrientation = pointsCount > 0? meanOrientation / pointsCount : 0;
    meanDistance = pointsCount > 0? meanDistance / pointsCount : 0;

    return pointsCount>0? true : false;
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
    return bubleRebound();
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
    double angularError = orientationError(this->reboundAngle);
    this->moveCommands->adjust_and_run(angularError);
}

std::vector<geometry_msgs::Point> squarePoints()
{
    std::vector<geometry_msgs::Point> vecPoints;
    int arrayX[] { -1, -1, 0, 0 };
    int arrayY[] { 0, 1, 1, 0 };
    for(size_t i = 0; i < 4; i++)
    {
        geometry_msgs::Point point;
        point.x = arrayX[i];
        point.y = arrayY[i];
        vecPoints.push_back(point);
    }
    return vecPoints;
}

int main(int argc, char *argv[])
{
    Navigation navigate(argc, argv);
    ros::Rate loop_rate(ros_loopRates::navigationLoop);
    loop_rate.sleep();
    std::vector<geometry_msgs::Point> vecPoints;
    vecPoints = squarePoints();
    while(ros::ok())
    {
        char c = 0;
        std::cout << "Say the command\n-> ";
        std::cin >> c;
        switch (c)
        {
        case 's':
            navigate.stopMoving();
            break;
        case 'n':
            for(size_t i = 0; i < vecPoints.size(); i++)
            {
                std::cout << "Heading to point ["
                          << vecPoints[i].x << ", "
                          << vecPoints[i].y << "] "
                          << '\n';
                navigate.go_to_goal(vecPoints[i]);
            }
            break;
        default:
            break;
        }
        if(c == 'q')
            break;
    }
}
