import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
import math

def quaternion_to_euler_angle(quaternion):
    ''' Conversion fro quaternion to euler. Inspired by https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles '''
    w = quaternion.w
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z

    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z

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
        self.pose = Pose()


    def get_position(self):
        if self.first_run:
            print 'Waiting for odometry message'
            return Point()
        else:
            return self.pose.position

    def get_orientation(self):
        if self.first_run:
            print 'Waiting for odometry message'
            return 0.0
        else:
            (roll, pitch, yaw) = quaternion_to_euler_angle(self.pose.orientation)
            return yaw
