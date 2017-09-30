#include "../include/grid_map.h"

Grid_Mapping::Grid_Mapping()
{
    // Initialize node and publisher.
    this->publisher = nh_.advertise<grid_map_msgs::GridMap>("grid_map_node/grid_map", 1, true);
    this->subscriber = nh_.subscribe("/hokuyo_scan", 1000, &Grid_Mapping::laserCallBack, this);
    this->createGridMap();
}

Grid_Mapping::~Grid_Mapping()
{

}

void Grid_Mapping::laserCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    this->laserRanges_.assign(std::begin(msg->ranges), std::end(msg->ranges));
    this->generateGridMap();
}

void Grid_Mapping::createGridMap()
{
    // Create grid map.
    this->map = grid_map::GridMap({"obstacles"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(grid_map_params::LengthX,
                                     grid_map_params::LengthY),
                                     grid_map_params::cellSize);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
              map.getLength().x(), map.getLength().y(),
              map.getSize()(0), map.getSize()(1));
}

void Grid_Mapping::generateGridMap()
{
    ros::Time time = ros::Time::now();
    for (grid_map::GridMapIterator it(this->map); !it.isPastEnd(); ++it)
    {
        grid_map::Position position;
        map.getPosition(*it, position);
        map.at("obstacles", *it) = -0.04 + 0.2 * std::sin(3.0 * time.toSec() + 5.0 * position.y()) * position.x();
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

int main(int argc, char** argv)
{
    // Initialize node and publisher.
    ros::init(argc, argv, "grid_map_node");
    Grid_Mapping gmap;
    gmap.generateGridMap();
    /* Wait for Laser data to generate grid map */
    while (gmap.ok())
    {
        ros::spinOnce();
    }
    return 0;
}
