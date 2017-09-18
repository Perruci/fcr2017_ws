#ifndef LASER_SUB_H
#define LASER_SUB_H

#include "../namespaces/parameters.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <boost/array.hpp>

typedef std::array<double, 2> laser_point;
class LaserSubscriber{
public:
    LaserSubscriber(int argc, char *argv[]);
    ~LaserSubscriber();

    inline void printLaser()
    {
        if(setupComplete)
            for(size_t i = 0; i < rangesSize; i++)
                std::cout << i << " Range: " << laserRanges_[i] << '\n';
    };

    inline void printNearPoints()
    {
        if(!setupComplete)
            return;
        auto vecPoints = getNearPoints();
        size_t vecSize = vecPoints.size();
        for(size_t i = 0; i < vecSize; i++)
            std::cout << "Near Point " << vecPoints[i][orientation]
                      << ", "          << vecPoints[i][distance] << '\n';
    };

    void plotResults();
    void setParameters(const sensor_msgs::LaserScan::ConstPtr&);
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr&);

    /* Laser Points Structure:
    laser_points[0] = orientation;
    laser_points[1] = distance;
    */
    enum {orientation, distance};
    std::vector<laser_point> getNearPoints(float distance = tolerance::objects);

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
};

#endif
