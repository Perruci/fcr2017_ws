#include "../include/grid_mapping.h"

int main(int argc, char** argv)
{
    // Initialize node and publisher.
    ros::init(argc, argv, "grid_map_node");
    Grid_Mapping gmap(argc, argv);
    /* Wait for Laser data to generate grid map */
    while (gmap.ok())
    {
        ros::spinOnce();
        gmap.generateGridMap();
    }
    return 0;
}
