#include "../include/grid_mapping.h"

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
