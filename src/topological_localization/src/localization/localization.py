import numpy as np
import cic_probabilities

class Localization:
    ''' Main Class for localization node '''
    def __init__(self):
        self.cic_prob = cic_probabilities.CIC_Probabilities()

    def process_sensor_reading(self, z):
        ''' Process sensor reading given in vector z '''
        self.cic_prob.update_belief(z)

    def run(self):
        # emulate a sensor reading none features
        z = np.array([1, 0, 1, 0, 0, 1, 0, 0])
        self.process_sensor_reading(z)
        self.cic_prob.plot_belief()
        print self.cic_prob.get_belief()
