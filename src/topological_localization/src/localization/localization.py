import rospy
import numpy as np
import cic_probabilities

from topological_localization.msg import LocalizationFeatures

class Localization:
    ''' Main Class for localization node '''
    def __init__(self):
        self.cic_prob = cic_probabilities.CIC_Probabilities()
        self.msg_reading = np.array([0, 0, 0])
        self.sub_line_features = rospy.Subscriber('localization/line_features', LocalizationFeatures, self.line_features_callback)

    def line_features_callback(self, msg):
        ''' Callback for Localization subscriber: sub_line_features of custom type LocalizationFeatures '''
        if msg.num_features != 3:
            print 'Recieved wrong type of message on topic localization/line_features'
            pass
        print 'Recived features message: ', msg.features_list
        self.msg_reading = np.array(msg.features_list)

    def update_belief(self, z):
        ''' Process sensor reading given in vector z '''
        self.cic_prob.update_belief(z)

    def run(self):
        self.update_belief(self.msg_reading)
        self.plot_belief()

    def plot_belief(self):
        self.cic_prob.plot_belief()
