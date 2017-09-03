#ifndef ULTRASOUND_SUB_H
#define ULTRASOUND_SUB_H

#include "ros/ros.h"
#include "p2os_msgs/SonarArray.h"

class UltrasoundSubscriber{
public:
    UltrasoundSubscriber(int argc, char *argv[]);
    ros::NodeHandle nh;
    ros::Subscriber msg_sub;
    std::vector<double> sonarRanges;
    void printSonar();
    void ultrasoundCallBack(const p2os_msgs::SonarArray::ConstPtr&);
};

#endif
