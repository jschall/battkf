import numpy as np
from math import *

class ExtendedKalmanFilter:
    def update(self, meas):
        meas = np.matrix(meas).T
        P = self.covariance
        H = self.H()
        R = self.R()
        I = np.matrix(np.eye(P.shape[0]))
        x_hat = self.state

        y_tilde = meas - self.h()
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

        x_hat = self.f(dt, u)
        P = F*P*F.T+Q

        self.state = x_hat
        self.covariance = P

    def h(self, dt):
        return H()*self.state

    def f(self, dt, u):
        B = self.B(dt) if u is not None else None

        x_hat = self.state

        x_hat = F*x_hat
        if B is not None and u is not None:
            x_hat += B*u

        return x_hat


class BattEstimator(ExtendedKalmanFilter):
    def __init__(self):
        # state vector: open-circuit voltage, current, battery resistance
        # x hat on wikipedia
        self.state = np.matrix([
            [ 0 ], #Vo
            [ 0 ], #I
            [ 0 ]  #R
            ])

        # P on wikipedia
        self.covariance = np.matrix([
            [ 20**2 ,   0   ,    0   ],
            [   0   , 1**2  ,    0   ],
            [   0   ,   0   , 0.2**2 ]
            ])

    def f(self, dt, u):
        Vo = self.state.item(0)
        I  = self.state.item(1)
        R  = self.state.item(2)

        if I < 0:
            I = 0

        if R < 0:
            R = 0

        return np.matrix([
            [ Vo ],
            [ I  ],
            [ R  ]
            ])


    def F(self, dt):
        return np.matrix(np.eye(self.state.shape[0]))

    def R(self):
        return np.matrix([
            [ 0.1**2 ,    0   ],
            [    0   , 0.1**2 ]
            ])

    def Q(self,dt):
        V_PNOISE = 0.05
        I_PNOISE = 1400
        R_PNOISE  = 0.05/60.0
        return np.matrix([
            [ (V_PNOISE*dt)**2 ,         0        ,         0        ],
            [         0        , (I_PNOISE*dt)**2 ,         0        ],
            [         0        ,         0        , (R_PNOISE*dt)**2 ]
            ])

    def h(self):
        Vo = self.state.item(0)
        I = self.state.item(1)
        R = self.state.item(2)
        return np.matrix([
            [ Vo-I*R ],
            [   I    ]
            ])

    def H(self):
        Vo = self.state.item(0)
        I = self.state.item(1)
        R = self.state.item(2)
        return np.matrix([
            [ 1 , -R , -I ],
            [ 0 ,  1 ,  0 ]
            ])

    def resting_voltage_estimate(self):
        return self.state.item(0)

    def current_estimate(self):
        return self.state.item(1)

    def resistance_estimate(self):
        return self.state.item(2)

    def __str__(self):
        return str(self.state)
