import rospy
from nav_msgs.msg import Odometry

class OdometryMonitor:
    ''' Odometry Subscriber Class '''
    def odometry_callback(self, odometry_data):
        ''' get pose from odometry message '''
        self.pose = odometry_data.pose.pose

    def __init__(self):
        self.sub_odometry = rospy.Subscriber('pose', Odometry, self.odometry_callback)


    def get_position(self):
        return self.pose.position
