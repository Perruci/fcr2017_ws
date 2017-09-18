#include "../include/navigation.h"

int main(int argc, char *argv[])
{
    Navigation navigate(argc, argv);
    ros::spinOnce();

    geometry_msgs::Point point;

    while(ros::ok())
    {
        std::cout << "Where would you like to go? \n"
                  << "Insert X value:\n-> ";
        std::cin >> point.x;
        std::cout << "Insert Y value:\n-> ";
        std::cin >> point.y;

        std::cout << "Heading to point [" << point.x << ", " << point.y << "]" << '\n';

        navigate.go_to_goal(point);

        navigate.stopMoving();
    }
    return 0;
}
