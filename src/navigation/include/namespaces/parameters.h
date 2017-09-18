#ifndef _PARAMS_H_
#define _PARAMS_H_

#include <iostream>
#include <boost/array.hpp>

/* ROS loop rates */
namespace ros_loopRates
{
    /* Used in kinematics.cpp */
    static unsigned int const movementLoop = 25;
    /* Used in navigation.cpp */
    static unsigned int const navigationLoop = 25;
}

/* Movement Speeds */
namespace move_speeds
{
    static float const linear = 0.2;
    static float const omega = 1;
}

/* Movement Tolerances */
namespace tolerance
{
    /* Tolerance for both orientation and localization */
    static float const orientation = 0.1;
    static float const location = 0.1;
}

/* Obstacle Detection (class Navigation)*/
namespace obstacle_detection
{
    /* Distance to Objects */
    static float const distance = 1.0;
    static float const max_range = 2.0;
    static float const min_front_deg = -45;
    static float const max_front_deg =  45;
}

/* Laser Points */
namespace laser
{
    enum {orientation, distance};
}

/* Orientation PID, used in class Movement */
namespace pid_orientation
{
    // Define the maximum velocity to PID
    static float const minVel = - move_speeds::omega;
    static float const maxVel =   move_speeds::omega;
    // Define orientation PIDs paramenters
    static float const gainP = 1.0;
    static float const gainI = 0.0;
    static float const gainD = 0.0;
    static float const timeStep = 1;
};
#endif
