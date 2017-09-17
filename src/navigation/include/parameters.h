#ifndef _PARAMS_H_
#define _PARAMS_H_

/* ROS loop rates */
namespace ros_loopRates
{
    /* Used in kinematics.cpp */
    static unsigned int const movementLoop = 25;
    /* Used in navigation.cpp */
    static unsigned int const navigationLoop = 25;
}

/* Orientation PID, used in class Movement */
namespace pid_orientation
{
    // Define the maximum velocity to PID
    static float const minVel = -1;
    static float const maxVel = 1;
    // Define orientation PIDs paramenters
    static float const gainP = 1.0;
    static float const gainI = 0.0;
    static float const gainD = 0.0;
    static float const timeStep = 1;
};
#endif
