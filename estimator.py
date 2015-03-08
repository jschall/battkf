import numpy as np
from math import *

class KalmanFilter:
    def update(self, meas):
        meas = np.matrix(meas).T
        P = self.covariance
        H = self.H(meas)
        R = self.R()
        I = np.matrix(np.eye(P.shape[0]))
        x_hat = self.state

        print x_hat

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
        Q = self.Q(dt)
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


class BattEstimator(KalmanFilter):
    def __init__(self):
        # state vector: resting voltage, battery resistance
        # x hat on wikipedia
        self.state = np.matrix([
            [ 16.8 ],
            [ 10 ],
            [ 0.04 ]
            ])

        # P on wikipedia
        self.covariance = np.matrix([
            [ 0 , 0 , 0 ],
            [ 0 , 0 , 0 ],
            [ 0 , 0 , 0 ]
            ])

    def R(self):
        meas_noise = np.matrix([
            [ 0.1 ],
            [ 0.1 ]
            ])
        return meas_noise*meas_noise.T

    def Q(self,dt):
        return np.matrix([
            [ 0.005*dt ,   0  , 0 ],
            [     0    , 1*dt , 0 ],
            [     0    ,   0  , 0.0001*dt ]
            ])

    def H(self, meas):
        state = self.state
        return np.matrix([
            [ 1 , 0 , -state.item(1) ],
            [ 0 , 1 ,       0        ]
            ])

    def resting_voltage_estimate(self):
        return self.state.item(0)

    def current_estimate(self):
        return self.state.item(1)

    def resistance_estimate(self):
        return self.state.item(2)

    def __str__(self):
        return str(self.state)
