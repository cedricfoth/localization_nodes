#!/usr/bin/env python3

import numpy as np

def get_f_F():
    def f_with_v(x: np.ndarray, dt: float):
        return np.array([x[0] + x[3] * dt, x[1] + x[4] * dt, x[2] + x[5] * dt, x[3], x[4], x[5]])
    
    def F_with_v(x: np.ndarray, dt: float):
        return np.array([[1, 0, 0, dt, 0, 0], [0, 1, 0, 0, dt, 0], [0, 0, 1, 0, 0, dt], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]])

    def f(x: np.ndarray, dt: float):
        return np.array([x[0], x[1], x[2], x[3], x[4], x[5]])
    
    def F(x: np.ndarray, dt: float):
        return np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]])
    
    return f, F