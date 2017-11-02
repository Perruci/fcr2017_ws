import numpy as np
import math

def get_int_distance(value1, value2):
    ''' Returns the absolute distance from two float objects '''
    return int(math.floor(abs(value1 - value2)))

def get_idx_from_polar(angle, radius, resolution):
    ''' Returns x and y integers corresponding to a point in polar coordinates '''
    x = int(math.floor(math.cos(angle) * radius * resolution))
    y = int(math.floor(math.sin(angle) * radius * resolution))
    return x, y

class Layer:
    ''' Default GridMap Layer Instance '''

    def __init__(self, name, region_points):
        '''
            Class Constructor
            params:
                name: string Layer name to b referenced
                region_points: list of two points corresponding to layer widith and height
        '''
        self.name = name
        self.set_borders(region_points[0], region_points[1])
        self.grid = self.init_layer()
        self.first_imshow = True

    def init_layer(self):
        '''
            Layers have, a resolution of 5 points per meter
        '''
        # basic setup
        grid_widith = get_int_distance(self.border_points[0].x, self.border_points[1].x)
        grid_height = get_int_distance(self.border_points[0].y, self.border_points[1].y)
        # number of points per meter
        self.resolution = 5
        # get rows and cols
        self.rows = grid_height * self.resolution
        self.cols = grid_widith * self.resolution
        # create empty numpy instace
        return np.zeros([self.rows, self.cols])

    def get_grid(self):
        ''' Return corresponding grid '''
        return self.grid

    def set_grid(self, matrix):
        ''' Assign a new matrix to layers grid '''
        if matrix.shape == self.grid.shape:
            self.grid = matrix
        else:
            print 'Trying to assign grid layer of a different shape'

    def set_borders(self, pt1, pt2):
        ''' Define border points for layer '''
        self.border_points = [pt1, pt2]

    # Obstacle processing ----------------------------------------------

    def draw_obstacles(self, origin_x, origin_y, orientation, angle_ranges, laser_ranges):
        '''
            Returns a modified grid which:
                value 0 represents an unknown region
                value 1 represents an occupied region
                value -1 represents an free region
            Params:
                origin_x: x coordinate of the polar coordinate space
                origin_y: y coordinate of the polar coordinate space
                angle_ranges: list of radian values corresponding to laser_ranges
                laser_ranges: list of laser distances corresponding to angle_ranges
        '''
        obstacle_value = 1
        free_value = -1

        idx_origin_x = get_int_distance(self.border_points[0].x, origin_x) * self.resolution
        idx_origin_y = get_int_distance(self.border_points[1].y, origin_y) * self.resolution

        for i in xrange(len(laser_ranges)):
            idx_x, idx_y = get_idx_from_polar(angle_ranges[i] + orientation, laser_ranges[i], self.resolution)
            idx_x = idx_origin_x + idx_x
            idx_y = idx_origin_y + idx_y
            if idx_x < self.cols and idx_x > 0:
                if idx_y < self.rows and idx_y > 0:
                    self.grid[idx_y, idx_x] = obstacle_value

            # mark robot
            if idx_origin_x < self.cols and idx_origin_x > 0:
                if idx_origin_y < self.rows and idx_origin_y > 0:
                    self.grid[idx_origin_y, idx_origin_x] = free_value
