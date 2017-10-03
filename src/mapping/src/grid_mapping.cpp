#include "../include/grid_mapping.h"

Grid_Mapping::Grid_Mapping(int argc, char **argv)
{
    // Initialize node and publisher.
    this->publisher = nh_.advertise<grid_map_msgs::GridMap>("grid_map_node/grid_map", 1, true);
    this->laserMonitor_ = new LaserSubscriber(argc, argv);
    this->odometryMonitor_ = new OdometrySubscriber(argc, argv);
    this->createGridMap();
}

Grid_Mapping::~Grid_Mapping()
{
    delete laserMonitor_;
}

void Grid_Mapping::createGridMap()
{
    this->gridPose = grid_map::Position(0.0,0.0);
    // Create grid map.
    this->map = grid_map::GridMap({"obstacles"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(grid_map_params::LengthX,
                                     grid_map_params::LengthY),
                                     grid_map_params::cellSize);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
              map.getLength().x(), map.getLength().y(),
              map.getSize()(0), map.getSize()(1));

    map.add("obstacles", 0.5);
}

void Grid_Mapping::generateGridMap()
{
    ros::Time time = ros::Time::now();

    if(!this->laserMonitor_->setupComplete)
    {
        return;
    }
    grid_map::Index start;
    grid_map::Index end;

    for(size_t i = 0; i < laserMonitor_->rangesSize; i++)
    {
        /* If found obstacle */
        if(laserMonitor_->laserRanges_[i] < laser_params::max_range)
        {
            map.getIndex(this->gridPose, start);
            map.getIndex(this->getPosition(i, laserMonitor_->laserRanges_[i]), end);
            for (grid_map::LineIterator it(this->map, start, end);
                !it.isPastEnd(); ++it)
            {
                grid_map::Position position;
                map.getPosition(*it, position);
                map.at("obstacles", *it) *= grid_map_params::freeProp;
            }
            map.at("obstacles", end) = grid_map_params::obstacleProp;
        }
        else
        {
            map.getIndex(this->gridPose, start);
            map.getIndex(grid_map::Position(laser_params::max_range, 0), end);
            for (grid_map::LineIterator it(this->map, start, end);
                !it.isPastEnd(); ++it)
            {
                grid_map::Position position;
                map.getPosition(*it, position);
                map.at("obstacles", *it) *= grid_map_params::freeProp;
            }
        }
    }

    this->publishGridMap(time);
}

void Grid_Mapping::publishGridMap(ros::Time& time)
{
    // Publish grid map.
    this->map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(this->map, message);
    this->publisher.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}

grid_map::Position Grid_Mapping::getPosition(size_t index, float rangesValue)
{
    double orientation = getOrientation(index);
    // get module
    int module = floor(rangesValue);
    float X = module*cos(orientation);
    float Y = module*sin(orientation);
    return grid_map::Position(X,Y);
}
