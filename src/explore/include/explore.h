#ifndef EXPLORE_H
#define EXPLORE_H

#include "movement/navigation.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

class Explore
{
public:
    Explore(int argc, char *argv[])
    ~Explore();

    bool run();
    void poseArrayCallback();
    void position_publish();
    ros::NodeHandle nh;
    ros::Subscriber poseArray_sub;
    ros::Publisher  poseArray_pub;
};

#endif
