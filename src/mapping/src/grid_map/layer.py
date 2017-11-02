import numpy as np

class Layer:
    ''' Default GridMap Layer Instance '''
    def init_layer(self):
        '''
            Layers have, by default 25 meters widith
            and 20 meters height
        '''
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
        ''' Class Constructor '''
        self.name = name
        self.grid = self.init_layer()
        self.border_points = set()

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

class BasicLayer(Layer):
    ''' Basic Grid Map Layer initialize as True for all obstacles '''
    def __init__(self, name):
        Layer.__init__(self, name)
        self.grid = np.ones(self.grid.shape, dtype=bool)
