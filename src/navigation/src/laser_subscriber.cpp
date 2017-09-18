#include "../include/measurements/laser_subscriber.h"

LaserSubscriber::LaserSubscriber(int argc, char *argv[])
{
    ros::init(argc, argv, "laser_subscriber");
    this->setupComplete = false;
    this->msg_sub = nh.subscribe("/hokuyo_scan", 1000, &LaserSubscriber::laserCallBack, this);
}

LaserSubscriber::~LaserSubscriber()
{
}

void LaserSubscriber::setParameters(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    this -> scan_time = msg -> scan_time;        //# time between scans [seconds]
    this -> range_min = msg -> range_min;        //# minimum range value [m]
    this -> range_max = msg -> range_max;        //# maximum range value [m]
    this -> angle_min = msg -> angle_min;        //# start angle of the scan [rad]
    this -> angle_max = msg -> angle_max;        //# end angle of the scan [rad]
    this -> angle_increment = msg -> angle_increment;  //# angular distance between measurements [rad]
    this -> laserRanges_.assign(std::begin(msg->ranges), std::end(msg->ranges));
    this -> rangesSize = this->laserRanges_.size();
    // this -> laserIntensities_.resize(rangesSize);
    this -> setupComplete = true;
}

void LaserSubscriber::laserCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if(!setupComplete)
        this -> setParameters(msg);
    else
        this -> laserRanges_.assign(std::begin(msg->ranges), std::end(msg->ranges));
}

std::vector<laser_point> LaserSubscriber::getNearPoints(float distance)
{
    if(!setupComplete)
    {
        throw std::logic_error("Negative time deducted when moving");
        return {};
    }
    std::vector<laser_point> nearPoints;
    for(size_t i = 0; i < rangesSize; i++)
        if(laserRanges_[i] < distance)
        {
            laser_point auxPoint = {i, laserRanges_[i]};
            nearPoints.push_back(auxPoint);
        }
    return nearPoints;
}

// int main(int argc, char *argv[])
// {
//     LaserSubscriber laser_sub(argc, argv);
//
//     ros::Rate loop_rate(25);
//     loop_rate.sleep();
//
//     while(ros::ok())
//     {
//         laser_sub.printLaser();
//         loop_rate.sleep();
//         ros::spinOnce();
//     }
// }
