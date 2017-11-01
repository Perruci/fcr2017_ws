import rospy
from nav_msgs.msg import Odometry

class OdometryMonitor:
    ''' Odometry Subscriber Class '''
    def odometry_callback(self, odometry_data):
        ''' get pose from odometry message '''
        self.pose = odometry_data.pose.pose
        if self.first_run:
            self.first_run = False

    def __init__(self):
        self.sub_odometry = rospy.Subscriber('pose', Odometry, self.odometry_callback)
        self.first_run = True


    def get_position(self):
        if self.first_run:
            print 'Waiting for odometry message'
            return None
        else:
            return self.pose.position
