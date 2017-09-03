#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "measurements/laser_subscriber.h"
#include "measurements/odometry_subscriber.h"
#include "measurements/ultrasound_subscriber.h"
#include "movement/kinematics.h"
#include "movement/movement.h"

class Navigation
{
public:
    Navigation(int argc, char *argv[]);
    ~Navigation();

    LaserSubscriber*      laserMonitor;
    OdometySubscriber*    odometryMonitor;
    UltrasoundSubscriber* sonarMonitor;
    Movement* moveCommands;
};

#endif
