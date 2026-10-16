"""Constant-velocity Kalman filter for depth.

Predict: prior belief
Update: likelihood
Result: posterior belief

State units: depth in m, velocity in m/s.
"""

import numpy as np


class DepthKalmanFilter:
    def __init__(self, initial_depth=0.0):
        # State: [depth, velocity] -> x ~ N(u, P)
        self.x = np.array([[initial_depth], [0.0]])
        # No correlation and equal uncertainty in the initial estimate
        self.P = np.eye(2)
        # Process noise: how wrong the prediction could be
        self.Q = np.array([[1e-4, 0.0], [0.0, 1e-3]])

        # Measurement noise covariance: z = Hx + v, v ~ N(0, R)
        self.R = 0.01  # Bar30 is very clean, keep this small
        # State transition (dt is filled in on every predict)
        self.F = np.array([[1.0, 0.034], [0.0, 1.0]])
        # Only depth is measured, not velocity: z = Hx
        self.H = np.array([[1.0, 0.0]])

    @property
    def depth(self):
        return self.x[0, 0]

    def predict(self, dt):
        """Propagate the state by dt seconds: depth_new = depth + v * dt."""
        self.F[0, 1] = dt
        self.x = self.F @ self.x
        # if x ~ N(u, P) then Fx ~ N(Fu, F P F^T); add Q for model drift
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.depth

    def correct(self, measured_depth):
        """Fuse a new depth measurement."""
        # Innovation (residual): sensor reading - prediction
        y = measured_depth - (self.H @ self.x)
        # Innovation covariance and Kalman gain
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T / S
        # Combine the sensor with the prediction
        self.x = self.x + K * y
        self.P = (np.eye(2) - K @ self.H) @ self.P
        return self.depth

    def update(self, measured_depth, dt):
        """Predict then correct. Returns the filtered depth."""
        self.predict(dt)
        return self.correct(measured_depth)
