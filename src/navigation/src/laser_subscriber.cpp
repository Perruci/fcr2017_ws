#include "../include/measurements/laser_subscriber.h"

LaserSubscriber::LaserSubscriber(int argc, char *argv[])
{
    ros::init(argc, argv, "Laser_subscriber");
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
    this -> rangesSize = sizeof(msg -> ranges);
    this -> laserRanges_.resize(rangesSize);
    // this -> laserIntensities_.resize(rangesSize);
    this -> setupComplete = true;
}

void LaserSubscriber::laserCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if(!setupComplete)
        this -> setParameters(msg);
    else
        for(size_t i = 0; i < rangesSize; i++)
        {
            laserRanges_[i] = msg -> ranges[i];
            // laserIntensities_[i] = msg -> intensities[i];
        }
}

void LaserSubscriber::printLaser()
{
    if(setupComplete)
        for(size_t i = 0; i < rangesSize; i++)
        {
            std::cout << i << " Range: " << laserRanges_[i] << '\n';
            // std::cout << i << " Intensity: " << laserIntensities_[i] << '\n';
        }
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
