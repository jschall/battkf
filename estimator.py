import numpy as np
from math import *

class KalmanFilter:
    def update(self):
        pass

    def predict(self):
        pass


class BatteryEstimator(KalmanFilter):
    def __init__(self):
        # state vector: voltage, current, resistance
        # x hat on wikipedia
        self.state = np.matrix([
            [0],
            [0],
            [0]
            ])

        # P on wikipedia
        self.covariance = np.matrix('1 0 0')

    def F(self):
        np.matrix('1 0 0;0 1 0;0 0 1')

    def H(self):
        '''observation model'''
        return np.matrix([
            [ 1, 0, -self.state[1] ],
            [ 0, 1,       0        ]
            ])

    def __str__(self):
        return str(self.state)


x = BatteryEstimator()
x.update()
print x
