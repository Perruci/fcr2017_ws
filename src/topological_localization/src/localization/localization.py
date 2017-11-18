import rospy
import math
import numpy as np
import cic_probabilities

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from topological_localization.msg import LocalizationFeatures

def point_distance(pt1, pt2):
    ''' Returns distance between pt1 and pt2 geometry_msgs.Point() values '''
    diff_x = pt1.x - pt2.x
    diff_y = pt1.y - pt2.y
    return math.sqrt(diff_x**2+diff_y**2)

class Localization:
    ''' Main Class for localization node '''
    def __init__(self):
        self.cic_prob = cic_probabilities.CIC_Probabilities()
        self.msg_reading = np.array([0, 0, 0])
        self.old_position = Point()
        self.new_position = Point()
        self.position_tolerance = 0.01 # meters
        self.first_run = True
        self.sub_line_features = rospy.Subscriber('localization/line_features', LocalizationFeatures, self.line_features_callback)
        self.sub_odometry = rospy.Subscriber('pose', Odometry, self.odometry_callback)

    def line_features_callback(self, msg):
        ''' Callback for Localization subscriber: sub_line_features of custom type LocalizationFeatures '''
        if msg.num_features != 3:
            print 'Recieved wrong type of message on topic localization/line_features'
            pass
        rospy.loginfo("Recived features message: %s", msg.features_list)
        self.msg_reading = np.array(msg.features_list)

    def odometry_callback(self, msg):
        '''
            Callback for pose/ subscriber.
            Sets self.moved to true when old_position differs from msg.pose.pose.position in more than position_tolerance.
        '''
        if self.first_run:
            self.old_position = msg.pose.pose.position
            self.first_run = False
        else:
            self.new_position = msg.pose.pose.position

    def has_moved(self):
        ''' Returns true if old position differs from new position, false otherwise '''
        diff_position = point_distance(self.old_position, self.new_position)
        self.update_position()
        if diff_position > self.position_tolerance:
            return True
        else:
            return False

    def update_position(self):
        ''' Updates old_position value '''
        self.old_position.x = self.new_position.x
        self.old_position.y = self.new_position.y


    def update_belief(self, z, has_moved):
        ''' Process sensor reading given in vector z and the movement information'''
        self.cic_prob.update_belief(z, has_moved)

    def run(self):
        z = self.msg_reading
        has_moved = self.has_moved()
        self.update_belief(z, has_moved)
        self.plot_belief(z, has_moved)

    def plot_belief(self, z, has_moved):
        self.cic_prob.plot_belief(z, has_moved)
