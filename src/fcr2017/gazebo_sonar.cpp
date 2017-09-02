#include "ros/ros.h"
#include "p2os_msgs/SonarArray.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

p2os_msgs::SonarArray cacic_sonar;
double ranges[16];

void frontSonarRaw(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    for(int i = 0; i < 8; i++)
        ranges[i] = msg->ranges[i];
}

void backSonarRaw(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    for(int i = 0; i < 8; i++)
        ranges[i+8] = msg->ranges[i];
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "fake_sonar");
    ros::NodeHandle n;
    string front_topic = "/front_sonar_raw";
    string back_topic = "/back_sonar_raw";
    ros::Publisher sonar_pub = n.advertise<p2os_msgs::SonarArray>("sonar", 1000);
    ros::Subscriber front_sub = n.subscribe(front_topic, 1000, frontSonarRaw);
    ros::Subscriber back_sub = n.subscribe(back_topic, 1000, backSonarRaw);

    ros::Rate loop_rate(25);

    loop_rate.sleep();

    cacic_sonar.ranges_count = 16;
    cacic_sonar.ranges.resize(16);
    
    for(int i = 0; i < 16; i++)
        ranges[i] = 0;

    while(ros::ok())
    {
        for(int i = 0; i < 16; i++)
            cacic_sonar.ranges[i] = ranges[i];
        
        cacic_sonar.header.stamp = ros::Time::now();
        sonar_pub.publish(cacic_sonar);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
