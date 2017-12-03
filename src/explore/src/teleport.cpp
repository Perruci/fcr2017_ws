#include "../include/teleport.h"

Teleport::Teleport(int argc, char *argv[])
{
    ros::init(argc, argv,"teleport_pioneer");
    this->set_position(0,0);
    this->set_orientation(0);
    this->set_velocity_0();
}

void Teleport::set_position(float x, float y)
{
    this->start_pose.position.x = x;
    this->start_pose.position.y = y;
    this->start_pose.position.z = 0;
}

void Teleport::set_orientation(float z)
{
    this->start_pose.orientation.x =0.0;
    this->start_pose.orientation.y =0.0;
    this->start_pose.orientation.z = z;
    this->start_pose.orientation.w =0.0;
}

void Teleport::set_velocity_0()
{
    this->start_twist.linear.x =0.0;
    this->start_twist.linear.y =0.0;
    this->start_twist.linear.z =0.0;
    this->start_twist.angular.x =0.0;
    this->start_twist.angular.y =0.0;
    this->start_twist.angular.z =0.0;
}

void Teleport::serviceTeleport()
{
    this->modelstate.model_name =(std::string)"Pioneer3at";
    this->modelstate.reference_frame =(std::string)"world";
    this->modelstate.pose = this->start_pose;
    this->modelstate.twist = this->start_twist;

    ros::ServiceClient client = this->n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState setmodelstate;
    setmodelstate.request.model_state = modelstate;
    client.call(setmodelstate);
}
