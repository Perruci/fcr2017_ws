#include "../include/grid_map.h"

// using namespace grid_map;

int main(int argc, char** argv)
{
    // Initialize node and publisher.
    ros::init(argc, argv, "grid_map_simple_demo");
    ros::NodeHandle nh("~");
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    // Create grid map.
    grid_map::GridMap map({"obstacles"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(5.0, 5.0), 0.05);
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
        map.getLength().x(), map.getLength().y(),
        map.getSize()(0), map.getSize()(1));

    // Work with grid map in a loop.
    ros::Rate rate(30.0);
    while (nh.ok())
    {
        // Add data to grid map.
        ros::Time time = ros::Time::now();
        generateGridMap(map, time);
        publishGridMap(map, time, publisher);
        // Wait for next cycle.
        rate.sleep();
    }
    return 0;
}
