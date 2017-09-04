#include "../include/measurements/ultrasound_subscriber.h"

void UltrasoundSubscriber::ultrasoundCallBack(const p2os_msgs::SonarArray::ConstPtr& msg)
{
    for(size_t i = 0; i < 16; i++)
        this -> sonarRanges[i] = msg -> ranges[i];
}

UltrasoundSubscriber::UltrasoundSubscriber(int argc, char *argv[])
{
    ros::init(argc, argv, "ultrasound_subscriber");
    // sonarRanges.resize(16);
    this->msg_sub = nh.subscribe("/sonar", 1000, &UltrasoundSubscriber::ultrasoundCallBack, this);
}

void UltrasoundSubscriber::printSonar()
{
    std::cout << "Front Messages:\n \t[";
    for(size_t i = 0; i < 8; i++)
        std::cout << " " << this->sonarRanges[i] << ",";
    std::cout << "]" << '\n';
    std::cout << "Back Messages:\n \t[";
    for(size_t i = 0; i < 8; i++)
        std::cout << " " << this->sonarRanges[i+8] << ",";
}

// int main(int argc, char *argv[])
// {
//     UltrasoundSubscriber ultrasound_sub;
//
//     ros::Rate loop_rate(25);
//     loop_rate.sleep();
//
//     while(ros::ok())
//     {
//         ultrasound_sub.printSonar();
//         loop_rate.sleep();
//         ros::spinOnce();
//     }
// }
