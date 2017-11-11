''' Library file to host probabilities of CIC Map '''
import numpy as np
import matplotlib.pyplot as plt
import plots

def uniform_distribution(array_len):
    ''' Returns a 1D array of uniform probability distribution '''
    return np.array([1./array_len]*array_len)

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
        self.measurements_probability = np.vstack((self.prob_hallway, self.prob_inner_corner))
        self.measurements_probability = np.vstack((self.measurements_probability , self.prob_outer_corner))
        return self.measurements_probability

class CIC_Probabilities(SensorProbability):
    ''' Class designed to host the initial probabilities of the CIC Map '''
    def __init__(self):
        super(CIC_Probabilities, self).__init__() # call past constructor
        self.map_len = 18
        self.nodes_array = np.arange(1, self.map_len+1)
        self.set_prior_probabilities()

    def set_prior_probabilities(self):
        '''
            Define a priori probabilities:
                * belief states for the node localization probability
        '''
        self.belief = uniform_distribution(self.map_len)
        self.print_belief()
        print self.measurements_probability.shape
        print 'Prior probabilities set'

    def print_belief(self):
        plt.figure()
        plots.bar_plot(self.belief, self.nodes_array, (0,0.5))
        plt.title('Belief Plot')
        plt.xlabel('Node Ids')
        plt.ylabel('Current Probability')
        # show plot
        plt.ion()
        plt.show()
