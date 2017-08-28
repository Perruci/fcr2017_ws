#include "ros/ros.h"
#include "lib/ForwardKin.hpp"

void runAndStop(ForwardKin &fKin, float vel)
{
    ros::Duration delay(5.0);
    fKin.moveLinear(vel);
    delay.sleep();
    fKin.moveStop();
}

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
        switch (c)
        {
        case 'r':
            runAndStop(fKin, vel);
            break;
        case 's':
            fKin.moveStop();
            break;
        default:
            break;
        }
        if(c == 'q')
            break;
    }
}
