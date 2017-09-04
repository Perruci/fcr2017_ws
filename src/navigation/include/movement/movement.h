#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "ros/ros.h"
#include "kinematics.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

#define PI 3.14159265358979323846

class Movement
{
public:
    Movement(int argc, char *argv[]);
    ~Movement();
    Kinematics* kin_;
    /* Square Functions */
    void runAndStop(float, float);
    double timeTo90deg(float);
    void spin90degrees(float);
    void moveSquare(float,float);
    /* Circle functions */
    double timeTo360deg(float);
    void moveCircle(float, float);

    void stopMoving();
};
#endif
