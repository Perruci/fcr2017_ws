#ifndef _MOVEMENT_H_
#define _MOVEMENT_H_

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "measurements/odometry_subscriber.h"
#include "movement/kinematics.h"
#include "angleOps.h"
#include "pid.h"
#include <stdexcept>

// Define the maximum velocity to PID
#define MIN_ANG_VEL -1
#define MAX_ANG_VEL 1
// Define orientation PIDs paramenters
#define PID_ORIENTATION_P 1.0
#define PID_ORIENTATION_I 0.0
#define PID_ORIENTATION_D 0.0
#define PID_ORIENTATION_FS 1

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
    PID* anglePID;
};

#endif
