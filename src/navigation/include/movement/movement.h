#ifndef _MOVEMENT_H_
#define _MOVEMENT_H_

#include "ros/ros.h"
#include "../namespaces/parameters.h"
#include "../namespaces/angleOps.h"
#include "geometry_msgs/Point.h"
#include "measurements/odometry_subscriber.h"
#include "movement/kinematics.h"
#include "pid.h"
#include <stdexcept>

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
    void adjust_and_run(double, double);

    Kinematics* moveCommands;
    PID* anglePID;
};

#endif
