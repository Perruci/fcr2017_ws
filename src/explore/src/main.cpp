#include "../include/explore.h"
#include "../include/teleport.h"
#include "../include/movement/navigation.h"

char startMenu();

int main(int argc, char *argv[])
{
    Navigation navigate(argc, argv);
    Explore explore(argc, argv);
    Teleport teleport(argc, argv);
    ros::spinOnce();

    bool explore_complete = false;
    geometry_msgs::Point point;
    float orientation;
    while(ros::ok())
    {
        char command = startMenu();
        if(command != 'q')
        {
            switch (command)
            {
                case 'm':
                    std::cout << "Where would you like to go? \n"
                              << "Insert X value:\n-> ";
                    std::cin >> point.x;
                    std::cout << "Insert Y value:\n-> ";
                    std::cin >> point.y;

                    std::cout << "Heading to point [" << point.x << ", " << point.y << "]" << '\n';

                    navigate.go_to_goal(point);

                    navigate.stopMoving();
                    break;

                case 'p':
                    while(explore.run())
                    {
                        ros::spinOnce();
                        while(!explore.best_path_recieved)
                            ros::spinOnce();
                        explore.go_to_goal(navigate);
                        ros::spinOnce();
                    }
                    break;

                case 't':
                    std::cout << "Where would you like to go? \n"
                              << "Insert X value:\n-> ";
                    std::cin >> point.x;
                    std::cout << "Insert Y value:\n-> ";
                    std::cin >> point.y;
                    std::cout << "Which orientation? (rad)\n-> ";
                    std::cin >> orientation;
                    teleport.set_position(point.x, point.y);
                    teleport.set_orientation(orientation);
                    teleport.serviceTeleport();
            }
        }
        else
            break;
    }
    return 0;
}

char startMenu()
{
    std::cout << "Hello Human o/" << '\n';
    std::cout << "What would you like me to do?" << '\n';
    std::cout << "\tp --- run through all nodes" << '\n';
    std::cout << "\tm --- move to a point" << '\n';
    std::cout << "\tt --- teleport to a point" << '\n';
    std::cout << "\tq --- quit and sleep" << '\n';
    std::cout << "-> ";
    while (true)
    {
        char c;
        std::cin >> c;
        switch (c)
        {
            case 'p':
                return 'p';
                break;
            case 'm':
                return 'm';
                break;
            case 't':
                return 't';
            case 'q':
                return 'q';
                break;
            default:
                std::cout << "Oops, I didn't get it\ntry again: -> " << '\n';
                break;
        }
    }
}
