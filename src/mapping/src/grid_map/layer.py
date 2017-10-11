import numpy as np

class Layer:
    ''' Default GridMap Layer Instance '''
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
        return np.empty([rows, cols], np.float32)

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

class BasicLayer(Layer):
    ''' Basic Grid Map Layer initialize as 0.5 '''
    def __init__(self, name):
        Layer.__init__(self, name)
        self.grid = np.ones(self.grid.shape, self.grid.dtype)/2
