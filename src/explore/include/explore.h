#ifndef EXPLORE_H
#define EXPLORE_H

#include "movement/navigation.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

class Explore
{
public:
    Explore(int argc, char *argv[]);
    ~Explore();

    bool run();
    void loadToplogicalNodes();
    void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr&);
    void poseTargetPublish(geometry_msgs::Pose);
    void idTargetPublish(std::string);
    void go_to_goal(Navigation&);
    /* variables */
    std::vector<std::string> map_nodes;
    bool best_path_recieved;
    std::vector<geometry_msgs::Pose> posePath;
    bool target_pose_sent;
    geometry_msgs::Pose targetPose;
    /* ros interface */
    ros::NodeHandle nh;
    ros::Subscriber poseArray_sub;
    ros::Publisher  pose_pub;
    ros::Publisher  id_pub;
};

#endif
