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

    /* topological map integration ------------------------------ */
    void explore();

    /* Extern Interfaces ----------------------------------------*/
    inline std::array<double, 3> getOdometry(){return this->odometryMonitor->getOdometry();};

    /* Sensors Processing ----------------------------------------*/
    laser_point objectNear;
    double reboundAngle;
    /* Setup object mean alliases */
    double& nearestOrientation = objectNear[laser::orientation];
    double& nearestDistance = objectNear[laser::distance];
    bool setMinObstaclePoints();
    bool bubleRebound(float distance = obstacle_detection::distance);
    bool obstacleDetection(float distance = obstacle_detection::distance);
    void obstacleAvoidance();

    /* Go-To-Goal movement pattern ----------------------------------------*/
    double orientationError(geometry_msgs::Point);
    double orientationError(double);
    double locationError(geometry_msgs::Point);
    void go_to_goal(geometry_msgs::Point);

    /* Inline functions */
    inline void stopMoving()
    {
        this->moveCommands->stopMoving();
    }

    LaserSubscriber      *laserMonitor;
    OdometrySubscriber    *odometryMonitor;
    UltrasoundSubscriber *sonarMonitor;
    Movement *moveCommands;
};

#endif
