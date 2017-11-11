''' Library file to host probabilities of CIC Map '''
import numpy as np
import matplotlib.pyplot as plt
import plots

def uniform_distribution(array_len):
    ''' Returns a 1D array of uniform probability distribution '''
    return np.array([1./array_len]*array_len)

class PositionProbability(object):
    ''' Class designed to host the location probability '''
    def __init__(self):
        self.position_belief = uniform_distribution(18)

class SensorProbability(object):
    ''' Class designed to host sensor readings static probabilities '''
    def __init__(self):
        self.set_hallway_probability()
        self.set_inner_corners_probability()
        self.set_outer_corners_probability()
        self.set_static_measurements_probability()

    def set_hallway_probability(self):
        '''
            Inner corners probability for each node.
                - 1st row: None parallel lines found probability
                - 2nd row: parallel lines found probability
        '''
        self.hallway_features = 2
        # define arrays shape
        self.prob_hallway = np.ndarray(shape=(2,18))
        # set none probability
        self.prob_hallway[0,:] = uniform_distribution(18)
        # set single corner probability
        self.prob_hallway[1,:] = uniform_distribution(18)
        return self.prob_hallway

    def set_inner_corners_probability(self):
        '''
            Inner corners probability for each node.
                - 1st row: None corners found probability
                - 2nd row: One corner found probability
                - 3rd row: More than one corners found probability
        '''
        self.inner_features = 3
        # define arrays shape
        self.prob_inner_corner = np.ndarray(shape=(3,18))
        # set none probability
        self.prob_inner_corner[0,:] = uniform_distribution(18)
        # set single corner probability
        self.prob_inner_corner[1,:] = uniform_distribution(18)
        # set multi corner probability
        self.prob_inner_corner[2,:] = uniform_distribution(18)
        return self.prob_inner_corner

    def set_outer_corners_probability(self):
        '''
            Outer corners probability for each node.
                - 1st row: None corners found probability
                - 2nd row: One corner found probability
                - 3rd row: More than one corners found probability
        '''
        self.outer_features = 3
        # define arrays shape
        self.prob_outer_corner = np.ndarray(shape=(3,18))
        # set none probability
        self.prob_outer_corner[0,:] = uniform_distribution(18)
        # set single corner probability
        self.prob_outer_corner[1,:] = uniform_distribution(18)
        # set multi corner probability
        self.prob_outer_corner[2,:] = uniform_distribution(18)
        return self.prob_outer_corner

    def set_static_measurements_probability(self):
        '''
            Resturns static measurements probabilities array.
            They're returned in the following order:
                1. hallway probability
                2. inner corners probability
                3. outer corners probability
        '''
        # Define a feature order vector for posterior segmentation
        self.features_order = np.array([self.hallway_features, self.inner_features, self.outer_features])
        # Stack together the measurements probability
        self.measurements_probability = np.vstack((self.prob_hallway, self.prob_inner_corner))
        self.measurements_probability = np.vstack((self.measurements_probability , self.prob_outer_corner))
        return self.measurements_probability

class CIC_Probabilities(PositionProbability, SensorProbability):
    '''
        Class designed to host the initial probabilities of the CIC Map.
        Prior probabilities are already inherited of the subclasses.
        The main variables previously set are:
            self.position_belief: Position Probability
            self.measurements_probability: Sensor static probabilities
    '''
    def __init__(self):
        self.map_len = 18
        self.nodes_array = np.arange(1, self.map_len+1) # 1-18 array
        self.set_prior_probabilities() # call prior constructors

    def set_prior_probabilities(self):
        '''
            Prior probabilities are computed on iherited classes constructors.
        '''
        PositionProbability.__init__(self)
        SensorProbability.__init__(self)
        self.print_belief()
        print self.measurements_probability
        print 'Prior probabilities set'

    def print_belief(self):
        plt.figure()
        plots.bar_plot(self.position_belief, self.nodes_array, (0,0.5))
        plt.title('Position Belief Plot')
        plt.xlabel('Node Ids')
        plt.ylabel('Current Probability')
        # show plot
        plt.ion()
        plt.show()
