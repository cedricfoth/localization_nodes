#!/usr/bin/env python3

import numpy as np

class ExtendedKalmanFilter:
    def __init__(self, x0: np.ndarray, P0: np.ndarray, Q: np.ndarray, f, F: np.ndarray, x_limits: np.ndarray):
        self.x = x0
        self.P = P0
        self.Q = Q
        self.f = f
        self.F = F
        self.x_limits = x_limits
        self.prev_x = self.x

        self.num_states = len(x0)

    def change_params(self, Q):
        self.Q = Q

    def predict(self, dt: float) -> np.ndarray:
        self.x = self.f(self.x, dt)
        self.P = self.F(self.x, dt) @ self.P @ self.F(self.x ,dt).transpose() + self.Q
        return self.x

    def update(self, z: np.ndarray, h, H, R) -> np.ndarray:
        y = np.nan_to_num(z - h)
        S = H @ self.P @ H.transpose() + R
        K = self.P @ H.transpose() @ np.linalg.inv(S)
        x = self.x + K @ y
        # if (np.all(np.less_equal(x[:-4], self.x_limits[:-1])) and x[2] >= self.x_limits[2] and np.all(np.greater_equal(x[:-3], np.zeros(3)))):
        self.x = x
        self.P = (np.eye(self.num_states) - K @ H) @ self.P
        return x[:-3]

    def get_x(self) -> np.ndarray:
        return self.x