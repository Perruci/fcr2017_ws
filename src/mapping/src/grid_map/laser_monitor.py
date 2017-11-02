import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class LaserMonitor:

    def laser_callback(self, laser_data):
        if self.first_run:
            # define laser default parameters
            self.scan_time = laser_data.scan_time               # time between scans [seconds]
            self.range_min = laser_data.range_min               # minimum range value [m]
            self.range_max = laser_data.range_max               # maximum range value [m]
            self.angle_min = laser_data.angle_min               # start angle of the scan [rad]
            self.angle_max = laser_data.angle_max               # end angle of the scan [rad]
            self.angle_increment = laser_data.angle_increment   # angular distance between measurements [rad]
            self.first_run = False
            # output variables
            self.laser_ranges = np.array(laser_data.ranges)
            self.ranges_orientation = np.arange(self.angle_min, self.angle_max, self.angle_increment)

    def __init__(self):
        self.sub_laser = rospy.Subscriber('hokuyo_scan', LaserScan,  self.laser_callback)
        self.first_run = True

    def get_ranges(self):
        if self.first_run:
            print 'waiting for laser messages'
            return None
        else:
            return self.laser_ranges

    def get_obstacles(self):
        if self.first_run:
            print 'waiting for laser messages'
            return None
        obstacles = self.laser_ranges
        # returns the concatenation of both obstacles and orientation
        return obstacles
