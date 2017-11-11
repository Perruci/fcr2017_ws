''' Library file to host probabilities of CIC Map '''
import numpy as np
import matplotlib.pyplot as plt
import plots

def uniform_distribution(array_len):
    ''' Returns a 1D array of uniform probability distribution '''
    return np.array([1./array_len]*array_len)

class CIC_Probabilities:
    ''' Class designed to host the initial probabilities of the CIC Map '''
    def __init__(self):
        self.map_len = 18
        self.nodes_array = np.arange(1, self.map_len+1)
        self.set_prior_probabilities()

    def set_prior_probabilities(self):
        '''
            Define a priori probabilities:
                * belief states for the node localization probability
        '''
        print 'Prior probabilities set'
        self.belief = uniform_distribution(self.map_len)
        print self.belief
        self.print_belief()

    def print_belief(self):
        plots.bar_plot(self.belief, self.nodes_array)
        plt.title('Belief Plot')
        plt.xlabel('Node Ids')
        plt.ylabel('Current Probability')
        # show plot
        plt.ion()
        plt.show()
