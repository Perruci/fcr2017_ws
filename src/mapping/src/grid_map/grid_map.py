import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt

class Layer:

    def __init__(self, name):
        # basic setup
        self.grid_widith = 25
        self.grid_height = 20
        # number of points per meter
        self.resolution = 5
        # get rows and cols
        self.rows = self.grid_height * self.resolution
        self.cols = self.grid_widith * self.resolution
        self.name = name
        # create empty numpy instace
        self.grid = np.ones([self.rows, self.cols], np.float32)/2

    def get_grid(self):
        return self.grid

class GridMap:
    def __init__(self):
        self.layer_dict = {}
        self.num_layer = 0

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

    def show_grid_map(self, node, windowname):
        cv2.imshow(windowname, self.get_grid(node))
        cv2.waitKey(0)

    def save_grid_map(self, node, filename):
        cv2.imwrite(filename, self.get_grid(node))

    def run(self):
        self.add_layer('1')
        self.show_grid_map('1', 'grid_map')
        self.save_grid_map('1', 'grid.png')
