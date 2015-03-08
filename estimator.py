import numpy as np
from math import *

class KalmanFilter:
    def update(self, meas):
        P = self.covariance
        H = self.H(meas)
        R = self.R()
        I = np.matrix(np.eye(P.shape[0]))
        x_hat = self.state

        y_tilde = meas - H*x_hat
        S = H * P * H.T + R
        K = P * H.T * S.I
        x_hat = x_hat + K*y_tilde
        P = (I-K*H)*P

        self.state = x_hat
        self.covariance = P

    def predict(self, dt=0, u=None):
        P = self.covariance
        F = self.F(dt)
        q = self.Q(dt)
        B = self.B(dt) if u is not None else None
        x_hat = self.state

        x_hat = F*x_hat
        if B is not None and u is not None:
            x_hat += B*u
        P = F*P*F.T+Q

        self.state = x_hat
        self.covariance = P



    def F(self, dt=0):
        return np.matrix(np.eye(self.state.shape[0]))


class BatteryEstimator(KalmanFilter):
    def __init__(self):
        # state vector: resting voltage, battery resistance
        # x hat on wikipedia
        self.state = np.matrix([
            [ 0 ],
            [ 0 ]
            ])

        # P on wikipedia
        self.covariance = np.matrix([
            [ 0,   0    ],
            [ 0, 0.1**2 ]
            ])

    def R(self):
        meas_noise = np.matrix([
            [ 0.1 ],
            [ 0.1 ]
            ])
        return meas_noise*meas_noise.T

    def Q(self,dt):
        process_noise = np.matrix([
            [  0.005*dt  ],
            [ 0.00001*dt ]
            ])
        return process_noise*process_noise.T

    def H(self, meas):
        return np.matrix([
            [        1        , meas[1] ],
            [ 1/self.state[1] ,    0    ]
            ])

    def __str__(self):
        return str(self.state)
