#ifndef _ANGLE_OPS
#define _ANGLE_OPS

#include <iostream>
#include "math.h"

namespace angleOps
{
    static double const M_2PI = 2*M_PI;
    inline float degreesToRadians(float angleDegrees){return(angleDegrees * M_PI / 180.0);};
    inline float radiansToDegrees(float angleRadians){return(angleRadians * 180.0 / M_PI);};
    /* Radians Operation --------------------------------------*/
    // Reference: https://stackoverflow.com/questions/15634400/continous-angles-in-c-eq-unwrap-function-in-matlab
    // Normalize to [-180,180):
    inline double constrainAngle(double x){
        x = fmod(x + M_PI,M_2PI);
        if (x < 0)
            x += M_2PI;
        return x - M_PI;
    }
    // convert to [-360,360]
    inline double angleConv(double angle){
        return fmod(constrainAngle(angle),M_2PI);
    }
    inline double angleDiff(double a,double b){
        double dif = fmod(b - a + M_PI,M_2PI);
        if (dif < 0)
            dif += M_2PI;
        return dif - M_PI;
    }
    inline double unwrap(double previousAngle,double newAngle){
        return previousAngle - angleDiff(newAngle, previousAngle);
    }
    /* laser subscriber orientation --------------------------*/
    inline double getOrientation(unsigned int index, float minAngle, float maxAngle, float step)
    {
        float orientation = index*step + minAngle;
        if(orientation > maxAngle)
            std::logic_error("getOrientation() - calculated angle exeeded maxAngle");
        return orientation;
    }
}
#endif
