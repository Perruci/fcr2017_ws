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
<<<<<<< HEAD
        reading = np.array([0, 0, 0])
        self.update_belief(reading)
        self.plot_belief()

    def plot_belief(self):
=======
        z = np.array([1, 0, 0, 0, 0, 1, 0, 0])
        self.process_sensor_reading(z)
>>>>>>> e81474520fc3446fd89a4f4ea52b7ec8cc8f247e
        self.cic_prob.plot_belief()
