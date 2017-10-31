#include "../include/explore.h"

Explore::Explore(int argc, char *argv[])
{
    ros::init(argc, argv, "explore");
    this->poseArray_sub = nh.subscribe("topological/best_path/poses", 10, &Explore::poseArrayCallback, this);
    this->pose_pub = nh.advertise<geometry_msgs::Pose>("topological/where_to/pose", 10);
    this->id_pub = nh.advertise<std_msgs::String>("topological/where_to/id", 10);
    this->target_pose_sent = false;
    this->best_path_recieved = false;
    this->loadToplogicalNodes();
}

Explore::~Explore()
{

}

void Explore::loadToplogicalNodes()
{
    this->map_nodes = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "16", "17", "18"};
}

void Explore::poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    std::cout << "Best path poses recieved: " << '\n';
    this->posePath.assign(std::begin(msg->poses), std::end(msg->poses));
    for(auto pose : this->posePath)
    {
        std::cout << "pose: " << pose.position.x << " " << pose.position.y << '\n';
    }
    this->best_path_recieved = true;
    this->target_pose_sent = false;
}

void Explore::poseTargetPublish(geometry_msgs::Pose pose)
{
    this->pose_pub.publish(pose);
    std::cout << "Target pose published" << '\n';
    this->target_pose_sent = true;
}

void Explore::idTargetPublish(std::string str)
{
    std::cout << "Target id published" << '\n';
    std_msgs::String msg;
    msg.data = str;
    this->id_pub.publish(msg);
}

bool Explore::run()
{
    if(this->map_nodes.empty())
    {
        this->loadToplogicalNodes();
        return false;
    }
    this->best_path_recieved = false;
    std::string target = map_nodes.back();
    map_nodes.pop_back();
    std::cout << "Explore node " << target << '\n';
    this->idTargetPublish(target);
    ros::spinOnce();
    return false;
}
