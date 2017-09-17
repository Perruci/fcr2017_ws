#ifndef _PARAMS_H_
#define _PARAMS_H_

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
