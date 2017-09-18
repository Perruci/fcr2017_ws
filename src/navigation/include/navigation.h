#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "ros/ros.h"
#include "namespaces/parameters.h"
#include "namespaces/angleOps.h"
#include "geometry_msgs/Point.h"
#include "measurements/laser_subscriber.h"
#include "measurements/ultrasound_subscriber.h"
#include "movement/movement.h"
#include <stdexcept>

class Navigation
{
public:
    Navigation(int argc, char *argv[]);
    ~Navigation();
    double orientationError(geometry_msgs::Point);
    double locationError(geometry_msgs::Point);

    /* Extern Interfaces */
    inline std::array<double, 3> getOdometry(){return this->odometryMonitor->getOdometry();};

    /* Sensors Processing */
    bool obstacleDetection(float distance = tolerance::objects);

    /* Go-To-Goal movement pattern */
    void go_to_goal(geometry_msgs::Point);

    inline void stopMoving()
    {
        this->moveCommands->stopMoving();
        this->odometryMonitor->printOdometry();
        this->obstacleDetection();
    }

    LaserSubscriber      *laserMonitor;
    OdometySubscriber    *odometryMonitor;
    UltrasoundSubscriber *sonarMonitor;
    Movement *moveCommands;
};

#endif
