import rospy
from geometry_msgs.msg import Point

import numpy as np
from matplotlib import pyplot as plt

import laser_monitor
import odometry_monitor
import topological_monitor

import layer
import grid_2d

class GridMap:
    ''' GridMap node main Class '''

    def __init__(self):
        ''' GridMap Constructor '''
        self.layer_dict = {}
        self.num_layer = 0
        self.current_id = '0' # default node id
        self.current_position = Point(0,0,0)
        self.current_orientation = 0.0
        # ros setup
        self.laser_monitor = laser_monitor.LaserMonitor()
        self.odometry_monitor = odometry_monitor.OdometryMonitor()
        self.topological_monitor = topological_monitor.TopologicalMonitor()
        self.grid_2d = grid_2d.Grid2D()

    def __iter__(self):
        return iter(self.layer_dict.values())

    def add_layer(self, node):
        ''' Add a Layer to layer_dict '''
        if node not in self.layer_dict:
            self.num_layer = self.num_layer + 1
            new_layer = layer.Layer(node, self.region_points)
            self.layer_dict[node] = new_layer

    def get_layer(self, node):
        ''' Return layer called node '''
        if node in self.layer_dict:
            return self.layer_dict[node]
        else:
            return None

    def set_occupancy_border(self):
        ''' Define layer dimentions using topological_map information '''
        regionPt1, regionPt2 = self.topological_monitor.get_region()
        self.region_points = [regionPt1, regionPt2]

    def node_id_update(self):
        ''' Process any changes in current_id call back
            If finds a new node_id, creates a layer for it'''
        gotten_id = self.topological_monitor.get_id()
        if self.current_id != gotten_id:
            print 'changed topological node', self.current_id, 'to', gotten_id
            self.current_id = gotten_id
            self.set_occupancy_border()
            self.add_layer(self.current_id)

    def node_pose_update(self):
        ''' Updates the private vatiable for current_position '''
        self.current_position = self.odometry_monitor.get_position()
        self.current_orientation = self.odometry_monitor.get_orientation()

    def process_obstacles(self):
        ''' Process laser_monitor messages to detect and process obstacles '''
        obstacles = self.laser_monitor.get_obstacles()
        if obstacles is not None and self.current_id != '0':
            self.get_layer(self.current_id).draw_obstacles(self.current_position.x,
                                                           self.current_position.y,
                                                           self.current_orientation,
                                                           self.laser_monitor.get_angles(),
                                                           obstacles)

    def run(self):
        ''' Class Main Fuction '''
        self.node_id_update()
        self.node_pose_update()
        self.process_obstacles()

    def get_grid(self, node):
        ''' return layer corresponding to node identificator '''
        if node in self.layer_dict:
            return self.layer_dict[node].get_grid()
        else:
            return None

    def show_layer(self, node, windowname):
        ''' Show image corresponding to layer identified by node '''
        if node in self.layer_dict:
            self.grid_2d.show_grid2d(self.get_grid(node), windowname)
        else:
            print 'tried to show an unitialized layer'

    def save_layer(self, node, filename):
        ''' Save layer corresponding to node to a file '''
        if node in self.layer_dict:
            self.grid_2d.save_grid2d(self.get_grid(node), filename)
        else:
            print 'tried to save an unitialized layer'

    def save_all_layers(self, path = './'):
        ''' Save all layers to folder given in path '''
        for node in self.layer_dict:
            self.grid_2d.save_grid2d(self.get_grid(node), node, path)
