#ifndef _PARAMS_H_
#define _PARAMS_H_

#include <iostream>

/* Movement Tolerances */
namespace tolerance
{
    /* Tolerance for both orientation and localization */
    static float const orientation = 0.1;
    static float const location = 0.1;
}

/* Obstacle Detection (class Navigation)*/
namespace obstacle_detection
{
    /* Distance to Objects */
    static float const distance = 1;
    static float const max_range = 5.0;
    /* For orientation adjustmente */
    static float const min_front_deg = -90;
    static float const max_front_deg =  90;
    /* For obstacle detection */
    static float const min_obstacle_deg = -45;
    static float const max_obstacle_deg =  45;
}

/* Laser Points */
namespace laser
{
    enum {orientation, distance};
}

/* Grid Mapping Parameters */
namespace grid_map_params
{
    static float LengthX = 5.0;
    static float LengthY = 5.0;
    static float cellSize = 0.05;

    static float obstacleProp = 0.2;
    static float freeProp = 1;
}

namespace laser_params
{
    static float max_range = 2;
}

#endif
