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
        self.current_id = '0' # default node id
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

    def get_layer(self, node):
        if node in self.layer_dict:
            return self.layer_dict[node]
        else:
            return None

    def get_grid(self, node):
        if node in self.layer_dict:
            return self.layer_dict[node].get_grid()
        else:
            return None

    def set_occupancy_border(self):
        regionPt1, regionPt2 = self.topological_monitor.get_region()
        self.get_layer(self.basic_layer).set_borders(regionPt1, regionPt2)

    def node_id_monitor(self):
        gotten_id = self.topological_monitor.get_id()
        if self.current_id != gotten_id:
            print 'changed topological node', self.current_id, 'to', gotten_id
            self.current_id = gotten_id
            self.add_layer(self.current_id)
            self.set_occupancy_border()

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
        self.node_id_monitor()
