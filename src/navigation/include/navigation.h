#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "measurements/laser_subscriber.h"
#include "measurements/odometry_subscriber.h"
#include "measurements/ultrasound_subscriber.h"
#include "movement/kinematics.h"
#include <stdexcept>

class Navigation
{
public:
    Navigation(int argc, char *argv[]);
    ~Navigation();

    /* Time management loop */
    void movementTimeLoop(ros::Duration);

    /* General Movements */
    void moveMeters(float, float);
    ros::Duration timeToDistance(float, float);
    void spinDegrees(float, float);
    ros::Duration timeToAngle(float, float);
    void stopMoving();

    /* Cartesian Navigation */
    double orientationError(geometry_msgs::Point);
    double locationError(geometry_msgs::Point);
    void moveToPosition(geometry_msgs::Point, float);

    LaserSubscriber*      laserMonitor;
    OdometySubscriber*    odometryMonitor;
    UltrasoundSubscriber* sonarMonitor;
    Kinematics* moveCommands;
};

#endif
