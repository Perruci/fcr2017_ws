#include "../include/explore.h"

Explore::Explore(int argc, char *argv[])
{
    this->poseArray_sub = nh.subscribe("topological/best_path/poses",  geometry_msgs::PoseArray, 10, &Explore::poseArrayCallback, this);
    this->pose_pub = nh.advertise<geometry_msgs::Pose>("topological/where_to", 10);
}

void Explore::poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{

}

void Explore::targetPublish(geometry_msgs::Pose pose)
{
    this->pose_pub.publish(pose)
}

bool Explore::run()
{
    
}
