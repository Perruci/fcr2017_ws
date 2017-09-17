/* Inspired By: https://gist.github.com/bradley219/5373998 */
#ifndef _PID_H_
#define _PID_H_

#include <stdexcept>
#include <iostream>
#include <cmath>

class PIDImpl
{
public:
    PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
    ~PIDImpl();
    double calculate( double setpoint, double pv );
    double calculate( double error );

private:
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;
};

class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double dt, double max, double min, double Kp, double Kd, double Ki );

        // Returns the manipulated variable given a setpoint and current process value
        inline double calculate( double setpoint, double pv ){pimpl->calculate(setpoint, pv);};
        inline double calculate( double error ){pimpl->calculate(error);};
        ~PID();

    private:
        PIDImpl *pimpl;
};


#endif
