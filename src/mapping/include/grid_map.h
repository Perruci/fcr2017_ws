#ifndef _GRID_MAP_H_
#define _GRID_MAP_H_

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>

namespace grid_map_params
{
    float LengthX = 5.0;
    float LengthY = 5.0;
    float cellSize = 0.05;
}

namespace laser_params
{
    float max_range = 2;
}

class Grid_Mapping
{
public:
    Grid_Mapping();
    ~Grid_Mapping();

    void laserCallBack();
    void createGridMap();
    void generateGridMap();
    void publishGridMap(ros::Time& time);

    inline bool ok(){return nh_.ok();};

private:
    float angle_step;
    float minAngle, maxAngle;
    std::vector<float> laserRanges_;
    void laserCallBack(const sensor_msgs::LaserScan::ConstPtr&);
    double getOrientation(unsigned int index, float minAngle, float maxAngle, float step);
    grid_map::Position getPosition(size_t,float);

    ros::NodeHandle nh_;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    grid_map::GridMap map;
    grid_map::Position gridPose;
};

#endif
