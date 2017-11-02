import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class OdometryMonitor:
    ''' Odometry Subscriber Class '''
    def odometry_callback(self, odometry_data):
        ''' get pose from odometry message '''
        if not self.first_run:
            self.pose = odometry_data.pose.pose
        else:
            self.first_run = False


    def __init__(self):
        self.sub_odometry = rospy.Subscriber('pose', Odometry, self.odometry_callback)
        self.first_run = True


    def get_position(self):
        if self.first_run:
            print 'Waiting for odometry message'
            return Point()
        else:
            return self.pose.position
