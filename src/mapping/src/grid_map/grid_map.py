import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt

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

    def show_layer(self, node, windowname):
        cv2.imshow(windowname, self.get_grid(node))
        cv2.waitKey(0)

    def save_layer(self, node, filename):
        cv2.imwrite(filename, self.get_grid(node))

    def run(self):
        self.add_layer('1')
        self.show_layer('1', 'grid_map')
        self.save_layer('1', 'grid.png')
