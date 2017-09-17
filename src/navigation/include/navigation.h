#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "measurements/laser_subscriber.h"
#include "measurements/ultrasound_subscriber.h"
#include "movement/movement.h"
#include "angleOps.h"
#include <stdexcept>

class Navigation
{
public:
    Navigation(int argc, char *argv[]);
    ~Navigation();

    inline void go_to_goal(geometry_msgs::Point point, float vel)
    {
        this->moveCommands->go_to_goal(point, vel);
    };

    inline void stopMoving()
    {
        this->moveCommands->stopMoving();
        this->moveCommands->odometryMonitor->printOdometry();
        this->sonarMonitor->printSonar();
        // this->laserMonitor->printLaser();
    }

    LaserSubscriber*      laserMonitor;
    UltrasoundSubscriber* sonarMonitor;
    Movement* moveCommands;
};

#endif
