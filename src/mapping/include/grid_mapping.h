#ifndef _GRID_MAP_H_
#define _GRID_MAP_H_

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

#include "namespaces/parameters.h"
#include "namespaces/angleOps.h"
#include "measurements/laser_subscriber.h"
#include "measurements/odometry_subscriber.h"

class Grid_Mapping
{
public:
    Grid_Mapping(int argc, char **argv);
    ~Grid_Mapping();

    void laserCallBack();
    void createGridMap();
    void generateGridMap();
    void publishGridMap(ros::Time& time);

    inline bool ok(){return nh_.ok();};

private:
    inline double getOrientation(unsigned int index)
    {
        this->laserMonitor_->getOrientation(index);
    }
    grid_map::Position getPosition(size_t,float);

    ros::NodeHandle nh_;
    ros::Publisher publisher;
    grid_map::GridMap map;
    grid_map::Position gridPose;
    LaserSubscriber* laserMonitor_;
    OdometrySubscriber* odometryMonitor_;
};

#endif
