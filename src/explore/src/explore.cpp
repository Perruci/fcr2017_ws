#include "../include/explore.h"

Explore::Explore(int argc, char *argv[])
{
    ros::init(argc, argv, "explore");
    this->poseArray_sub = nh.subscribe("topological/best_path/poses", 10, &Explore::poseArrayCallback, this);
    this->pose_pub = nh.advertise<geometry_msgs::Pose>("topological/where_to", 10);
    this->posePath.clear();
    this->target_pose_sent = false;
    this->best_path_recieved = false;
}

Explore::~Explore()
{

}

void Explore::poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    // if(target_pose_sent)
    // {
    //     this->posePath.assign(std::begin(msg->poses), std::end(msg->poses));
    //     std::cout << "Best path poses recieved: " << '\n';
    //     for(auto pose : this->posePath)
    //     {
    //         std::cout << "pose: " << pose.position.x << " " << pose.position.y << '\n';
    //     }
    //     this->best_path_recieved = true;
    //     this->target_pose_sent = false;
    // }
}

void Explore::targetPublish(geometry_msgs::Pose pose)
{
    this->pose_pub.publish(pose);
}

bool Explore::run()
{

}
