import numpy as np
import math

def get_int_difference(value1, value2):
    ''' Returns the absolute distance from two float objects '''
    return int(math.floor(abs(value1 - value2)))

def get_idx_from_polar(angle, radius, resolution):
    ''' Returns x and y integers corresponding to a point in polar coordinates '''
    x = int(math.floor(math.cos(angle) * radius * resolution))
    y = int(math.floor(math.sin(angle) * radius * resolution))
    return x, y

def find_nearest(array,value):
    ''' Find nearest index in a array. Ref: https://stackoverflow.com/questions/2566412/find-nearest-value-in-numpy-array '''
    idx = (np.abs(array-value)).argmin()
    print array[idx]
    return idx

def compute_angle(idx_origin_x, idx_origin_y, idx_x, idx_y):
    ''' Returns the orientation diference between idx_origin and idx '''
    diff_x = idx_x - idx_origin_x
    diff_y = idx_y - idx_origin_y
    return math.atan2(diff_y, diff_x)

def check_for_occupation(laser_value, distance):
    ''' Check if there is an obstacle for pixel given laser_value and distance '''
    tolerance = 0.1

    obstacle_value = 1
    unknown_value = 0
    free_value = -1

    diff_distance = laser_value - distance

    if abs(diff_distance) < tolerance:
        print 'Obstacle!'
        return obstacle_value
    else:
        if diff_distance > 0:
            print 'Free!'
            return free_value
        else:
            print 'unknown_value!'
            return unknown_value


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
        grid_widith = get_int_difference(self.border_points[0].x, self.border_points[1].x)
        grid_height = get_int_difference(self.border_points[0].y, self.border_points[1].y)
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

    def is_inside(self, x, y):
        ''' Check wheather index values x and y are inside grid matrix '''
        if x < self.cols and x > 0:
            if y < self.rows and y > 0:
                return True
        return False

    def compute_distance(self, idx_origin_x, idx_origin_y, idx_x, idx_y):
        diff_x = idx_x - idx_origin_x
        diff_y = idx_y - idx_origin_y
        return math.sqrt(diff_x**2 + diff_y**2) / self.resolution

    def draw_obstacles(self, origin_x, origin_y, orientation, angle_ranges, laser_ranges, min_max_angle):
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
                min_max_angle: numpy list. first element is minimum angle, second is maximum
        '''
        idx_origin_x = get_int_difference(self.border_points[0].x, origin_x) * self.resolution
        idx_origin_y = get_int_difference(self.border_points[1].y, origin_y) * self.resolution

        for (idx_y, idx_x), pixel_value in np.ndenumerate(self.grid):
            theta = compute_angle(idx_origin_x, idx_origin_y, idx_x, idx_y) - orientation
            if theta > min_max_angle[0] and theta < min_max_angle[1]:
                near_idx = find_nearest(angle_ranges, theta)
                self.grid[idx_y, idx_x] = check_for_occupation(laser_ranges[near_idx],
                                                               self.compute_distance(idx_origin_x,
                                                                                     idx_origin_y,
                                                                                     idx_x, idx_y))
