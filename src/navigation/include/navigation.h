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

    /* Extern Interfaces ----------------------------------------*/
    inline std::array<double, 3> getOdometry(){return this->odometryMonitor->getOdometry();};

    /* Sensors Processing ----------------------------------------*/
    laser_point objectsMean;
    double reboundAngle;
    /* Setup object mean alliases */
    double& meanOrientation = objectsMean[laser::orientation];
    double& meanDistance = objectsMean[laser::distance];
    bool setMeanObstaclePoints(std::vector<laser_point> &frontPoints);
    bool bubleRebound(float distance = obstacle_detection::distance);
    bool obstacleDetection(float distance = obstacle_detection::distance);
    void obstacleAvoidance();

    /* Go-To-Goal movement pattern ----------------------------------------*/
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
