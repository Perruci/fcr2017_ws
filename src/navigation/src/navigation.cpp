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
bool Navigation::obstacleDetection(float distance)
{
    float maxFrontAngle = angleOps::degreesToRadians(tolerance::max_front_deg);
    float minFrontAngle = angleOps::degreesToRadians(tolerance::min_front_deg);
    std::vector<laser_point> frontPoints = laserMonitor->getInRange(minFrontAngle, maxFrontAngle);
    if(frontPoints.empty())
        return false;
    size_t vecSize = frontPoints.size();
    for(size_t i = 0; i < vecSize; i++)
        std::cout << "Front Point " << frontPoints[i][laser::orientation]
                  << ", "          << frontPoints[i][laser::distance] << '\n';
    return true;
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
    /* Loop until orientation is adjusted and location is reached */
    double angularError = orientationError(point);
    double positionError = locationError(point);
    while(std::abs(positionError) > tolerance::location)
    {
        this->moveCommands->adjust_and_run(angularError, positionError);
        /* Update errors */
        angularError = orientationError(point);
        positionError = locationError(point);
    }
    /* Stop Movement */
    this->stopMoving();
};

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
