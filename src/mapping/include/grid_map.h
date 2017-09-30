#ifndef _GRID_MAP_H_
#define _GRID_MAP_H_

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>

inline void generateGridMap(grid_map::GridMap& map, ros::Time time)
{
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it)
    {
        grid_map::Position position;
        map.getPosition(*it, position);
        map.at("obstacles", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
    }
}

inline void publishGridMap(grid_map::GridMap& map, ros::Time time, ros::Publisher& pub)
{
    // Publish grid map.
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    pub.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}


#endif
