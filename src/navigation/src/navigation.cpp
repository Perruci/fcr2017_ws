#include "../include/navigation.h"

Navigation::Navigation(int argc, char *argv[])
{
    this->moveCommands = new Movement(argc, argv);
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

void Navigation::moveFront(float vel, float Time)
{
    this->moveCommands->runAndStop(vel, Time);
    this->odometryMonitor->printOdometry();
}

void Navigation::turn90degrees(float vel)
{
    this->moveCommands->spin90degrees(vel);
    this->odometryMonitor->printOdometry();
}

int main(int argc, char *argv[])
{
    Navigation navigate(argc, argv);
    ros::Rate loop_rate(100);
    loop_rate.sleep();
    float vel = 0.2;
    float angVel = 0.2;
    int Time = 5;
    while(ros::ok())
    {
        char c = 0;
        std::cout << "Say the command\n-> ";
        std::cin >> c;
        switch (c)
        {
        case 'w':
            navigate.moveFront(vel, Time);
            break;
        case 'a':
            navigate.turn90degrees(angVel);
            break;
        case 'd':
            navigate.turn90degrees(-angVel);
            break;
        // case 's':
        //     move.kin_->moveStop();
            break;
        default:
            break;
        }
        if(c == 'q')
            break;
    }
}
