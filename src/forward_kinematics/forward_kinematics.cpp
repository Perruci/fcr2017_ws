#include "ros/ros.h"
#include "lib/ForwardKin.hpp"

int main(int argc, char *argv[])
{
    /* Forward Kinematics class */
    ForwardKin fKin(argc, argv);
    /* Program begins */
    float vel = 0.1;
    char c;
    while(ros::ok())
    {
        std::cin >> c;
        if(c == 'l')
        {
            fKin.moveLinear(vel);
        }
        if(c == 'r')
        {
            fKin.moveAngular(vel);
        }
        if(c == 's')
        {
            fKin.moveStop();
        }
        if(c == 'q')
        {
            break;
        }
        fKin.sleep();
        ros::spinOnce();
    }
}
