import numpy as np
import cic_probabilities

class Localization:
    ''' Main Class for localization node '''
    def __init__(self):
        self.cic_prob = cic_probabilities.CIC_Probabilities()

    def update_belief(self, z):
        ''' Process sensor reading given in vector z '''
        self.cic_prob.update_belief(z)

    def run(self):
        # emulate a sensor reading none features
        reading = np.array([0, 0, 0])
        self.update_belief(reading)
        self.plot_belief()

    def plot_belief(self):
        self.cic_prob.plot_belief()
