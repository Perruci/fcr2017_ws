#ifndef LASER_SUB_H
#define LASER_SUB_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <boost/array.hpp>

// Size of C array.
template <typename T,unsigned S>
inline unsigned arraysize(const T (&v)[S]) { return S; }

class LaserSubscriber{
public:
    LaserSubscriber(int argc, char *argv[]);
    ~LaserSubscriber();
    ros::NodeHandle nh;
    ros::Subscriber msg_sub;
    bool setupComplete;
    /* Get to know hokuyo_scan */
    size_t rangesSize;
    std::vector<float> laserRanges_;
    float scan_time;        //# time between scans [seconds]
    float range_min;        //# minimum range value [m]
    float range_max;        //# maximum range value [m]
    float angle_min;        //# start angle of the scan [rad]
    float angle_max;        //# end angle of the scan [rad]
    float angle_increment;  //# angular distance between measurements [rad]
    // std::vector<float> laserIntensities_; all intensities return zero...

    void printLaser();
    void plotResults();
    void setParameters(const sensor_msgs::LaserScan::ConstPtr&);
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr&);
};

#endif
