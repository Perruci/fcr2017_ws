#ifndef _MOVEMENT_H_
#define _MOVEMENT_H_

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "measurements/odometry_subscriber.h"
#include "movement/kinematics.h"
#include "angleOps.h"
#include <stdexcept>

// Define the maximum velocity to PID
#define MAX_LIN_VEL 1
#define MAX_ANG_VEL 1

class Movement
{
public:
    Movement(int argc, char *argv[]);
    ~Movement();

    /* Time management loop */
    void movementTimeLoop(ros::Duration);

    /* Open Loop Movements */
    void moveMeters(float, float);
    ros::Duration timeToDistance(float, float);
    void spinDegrees(float, float);
    ros::Duration timeToAngle(float, float);
    void stopMoving();

    /* Closed Loop Movements */
    double orientationError(geometry_msgs::Point);
    double locationError(geometry_msgs::Point);
    void go_to_goal(geometry_msgs::Point, float);

    OdometySubscriber*    odometryMonitor;
    Kinematics* moveCommands;
};

#endif
