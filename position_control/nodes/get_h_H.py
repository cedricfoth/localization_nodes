#!/usr/bin/env python3

import numpy as np

def get_h_H(x: np.ndarray, r_tag_K: np.ndarray, r_Cam_K: np.ndarray, r_pressure_K: np.ndarray, rho: int, g: float, atmospheric_pressure: int):
    d0 = np.sqrt((x[0] - r_tag_K[0][0] + r_Cam_K[0])**2 + (x[1] - r_tag_K[0][1] + r_Cam_K[1])**2 + (x[2] - r_tag_K[0][2] + r_Cam_K[2])**2)
    d1 = np.sqrt((x[0] - r_tag_K[1][0] + r_Cam_K[0])**2 + (x[1] - r_tag_K[1][1] + r_Cam_K[1])**2 + (x[2] - r_tag_K[1][2] + r_Cam_K[2])**2)
    d2 = np.sqrt((x[0] - r_tag_K[2][0] + r_Cam_K[0])**2 + (x[1] - r_tag_K[2][1] + r_Cam_K[1])**2 + (x[2] - r_tag_K[2][2] + r_Cam_K[2])**2)
    d3 = np.sqrt((x[0] - r_tag_K[3][0] + r_Cam_K[0])**2 + (x[1] - r_tag_K[3][1] + r_Cam_K[1])**2 + (x[2] - r_tag_K[3][2] + r_Cam_K[2])**2)
    p = -(x[2] + r_pressure_K[2]) * rho * g + atmospheric_pressure

    h = np.array([d0, d1, d2, d3])
    
    H = np.array([[(r_Cam_K[0] - r_tag_K[0][0] + x[0])/np.sqrt((r_Cam_K[0] - r_tag_K[0][0] + x[0])**2 + (r_Cam_K[1] - r_tag_K[0][1] + x[1])**2 + (r_Cam_K[2] - r_tag_K[0][2] + x[2])**2), (r_Cam_K[1] - r_tag_K[0][1] + x[1])/np.sqrt((r_Cam_K[0] - r_tag_K[0][0] + x[0])**2 + (r_Cam_K[1] - r_tag_K[0][1] + x[1])**2 + (r_Cam_K[2] - r_tag_K[0][2] + x[2])**2), (r_Cam_K[2] - r_tag_K[0][2] + x[2])/np.sqrt((r_Cam_K[0] - r_tag_K[0][0] + x[0])**2 + (r_Cam_K[1] - r_tag_K[0][1] + x[1])**2 + (r_Cam_K[2] - r_tag_K[0][2] + x[2])**2), 0, 0, 0], [(r_Cam_K[0] - r_tag_K[1][0] + x[0])/np.sqrt((r_Cam_K[0] - r_tag_K[1][0] + x[0])**2 + (r_Cam_K[1] - r_tag_K[1][1] + x[1])**2 + (r_Cam_K[2] - r_tag_K[1][2] + x[2])**2), (r_Cam_K[1] - r_tag_K[1][1] + x[1])/np.sqrt((r_Cam_K[0] - r_tag_K[1][0] + x[0])**2 + (r_Cam_K[1] - r_tag_K[1][1] + x[1])**2 + (r_Cam_K[2] - r_tag_K[1][2] + x[2])**2), (r_Cam_K[2] - r_tag_K[1][2] + x[2])/np.sqrt((r_Cam_K[0] - r_tag_K[1][0] + x[0])**2 + (r_Cam_K[1] - r_tag_K[1][1] + x[1])**2 + (r_Cam_K[2] - r_tag_K[1][2] + x[2])**2), 0, 0, 0], [(r_Cam_K[0] - r_tag_K[2][0] + x[0])/np.sqrt((r_Cam_K[0] - r_tag_K[2][0] + x[0])**2 + (r_Cam_K[1] - r_tag_K[2][1] + x[1])**2 + (r_Cam_K[2] - r_tag_K[2][2] + x[2])**2), (r_Cam_K[1] - r_tag_K[2][1] + x[1])/np.sqrt((r_Cam_K[0] - r_tag_K[2][0] + x[0])**2 + (r_Cam_K[1] - r_tag_K[2][1] + x[1])**2 + (r_Cam_K[2] - r_tag_K[2][2] + x[2])**2), (r_Cam_K[2] - r_tag_K[2][2] + x[2])/np.sqrt((r_Cam_K[0] - r_tag_K[2][0] + x[0])**2 + (r_Cam_K[1] - r_tag_K[2][1] + x[1])**2 + (r_Cam_K[2] - r_tag_K[2][2] + x[2])**2), 0, 0, 0], [(r_Cam_K[0] - r_tag_K[3][0] + x[0])/np.sqrt((r_Cam_K[0] - r_tag_K[3][0] + x[0])**2 + (r_Cam_K[1] - r_tag_K[3][1] + x[1])**2 + (r_Cam_K[2] - r_tag_K[3][2] + x[2])**2), (r_Cam_K[1] - r_tag_K[3][1] + x[1])/np.sqrt((r_Cam_K[0] - r_tag_K[3][0] + x[0])**2 + (r_Cam_K[1] - r_tag_K[3][1] + x[1])**2 + (r_Cam_K[2] - r_tag_K[3][2] + x[2])**2), (r_Cam_K[2] - r_tag_K[3][2] + x[2])/np.sqrt((r_Cam_K[0] - r_tag_K[3][0] + x[0])**2 + (r_Cam_K[1] - r_tag_K[3][1] + x[1])**2 + (r_Cam_K[2] - r_tag_K[3][2] + x[2])**2), 0, 0, 0]])
    
    return h, H