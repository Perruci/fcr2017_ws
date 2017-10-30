#ifndef EXPLORE_H
#define EXPLORE_H

#include "movement/navigation.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

class Explore
{
public:
    Explore(int argc, char *argv[]);
    ~Explore();

    bool run();
    void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr&);
    void targetPublish(geometry_msgs::Pose);
    /* variables */
    bool best_path_recieved;
    std::vector<geometry_msgs::Pose> posePath;
    bool target_pose_sent;
    geometry_msgs::Pose targetPose;
    /* ros interface */
    ros::NodeHandle nh;
    ros::Subscriber poseArray_sub;
    ros::Publisher  pose_pub;
};

#endif
