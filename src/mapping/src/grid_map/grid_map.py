import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt

import laser_monitor
import odometry_monitor

class Layer:
    def init_layer(self):
        # basic setup
        grid_widith = 25
        grid_height = 20
        # number of points per meter
        resolution = 5
        # get rows and cols
        rows = grid_height * resolution
        cols = grid_widith * resolution
        # create empty numpy instace
        return np.ones([rows, cols], np.float32)/2

    def __init__(self, name):
        self.name = name
        self.grid = self.init_layer()

    def get_grid(self):
        return self.grid

    def set_grid(self, matrix):
        if matrix.shape == self.grid.shape:
            self.grid = matrix
        else:
            print 'Trying to assign grid layer of a different shape'

class GridMap:
    def __init__(self):
        self.layer_dict = {}
        self.num_layer = 0
        self.laser_monitor = laser_monitor.LaserMonitor()
        self.odometry_monitor = odometry_monitor.OdometryMonitor()

    def __iter__(self):
        return iter(self.layer_dict.values())

    def add_layer(self, node):
        self.num_layer = self.num_layer + 1
        new_layer = Layer(node)
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
        self.add_layer('occupancy_map')
        self.show_layer('occupancy_map', 'grid_map')
