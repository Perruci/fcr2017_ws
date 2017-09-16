#include "../include/navigation.h"

Navigation::Navigation(int argc, char *argv[])
{
    this->moveCommands = new Movement(argc, argv);
    this->laserMonitor = new LaserSubscriber(argc, argv);
    this->sonarMonitor = new UltrasoundSubscriber(argc, argv);
}

Navigation::~Navigation()
{
    delete moveCommands;
    delete laserMonitor;
    delete sonarMonitor;
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
    ros::Rate loop_rate(100);
    loop_rate.sleep();
    float vel = 0.2;
    float angVel = 0.2;
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
                navigate.go_to_goal(vecPoints[i], vel);
            }
            break;
        default:
            break;
        }
        if(c == 'q')
            break;
    }
}
