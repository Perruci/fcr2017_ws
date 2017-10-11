import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt

import laser_monitor
import odometry_monitor
import topological_monitor

import layer

class GridMap:
    ''' GridMap node main Class '''
    def init_basic_layer(self):
        self.num_layer = self.num_layer + 1
        new_layer = layer.BasicLayer(self.basic_layer)
        self.layer_dict[self.basic_layer] = new_layer

    def __init__(self):
        self.basic_layer = 'occupancy_map'
        self.layer_dict = {}
        self.num_layer = 0
        self.laser_monitor = laser_monitor.LaserMonitor()
        self.odometry_monitor = odometry_monitor.OdometryMonitor()
        self.topological_monitor = topological_monitor.TopologicalMonitor()
        self.init_basic_layer()

    def __iter__(self):
        return iter(self.layer_dict.values())

    def add_layer(self, node):
        self.num_layer = self.num_layer + 1
        new_layer = layer.Layer(node)
        self.layer_dict[node] = new_layer

    def get_grid(self, node):
        if node in self.layer_dict:
            return self.layer_dict[node].get_grid()
        else:
            return None

    def show_layer(self, node, windowname):
        if node in self.layer_dict:
            cv2.imshow(windowname, self.get_grid(node))
        else:
            print 'tried to show an unitialized layer'

    def save_layer(self, node, filename):
        if node in self.layer_dict:
            cv2.imwrite(filename, self.get_grid(node))
        else:
            print 'tried to save an unitialized layer'

    def run(self):
        self.show_layer(self.basic_layer, 'grid_map')
