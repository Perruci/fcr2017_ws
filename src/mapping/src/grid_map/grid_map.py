import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt

class GridMap:
    def __init__(self):
        self.grid_widith = 25
        self.grid_height = 20
        # number of points per meter
        self.resolution = 5
        # get rows and cols
        self.rows = self.grid_height * self.resolution
        self.cols = self.grid_widith * self.resolution
        # create empty numpy instace
        self.grid = np.ones([self.rows, self.cols], np.float32)/2

    def print_grid_params(self):
        print 'Grid Created'
        print '\t widith: ',self.grid_widith,'m, height: ',self.grid_height,'m, resolution: ',self.resolution
        print '\t grid shape: ',self.grid.shape

    def show_grid_map(self, windowname):
        cv2.imshow(windowname, self.grid)
        cv2.waitKey(0)

    def save_grid_map(self, filename):
        mat = cv2.cvtColor(self.grid, cv2.COLOR_GRAY2BGR)
        cv2.imwrite(filename, mat)

    def run(self):
        self.print_grid_params()
        self.show_grid_map('grid_map')
