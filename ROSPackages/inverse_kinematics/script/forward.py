#!/usr/bin/env python3

import numpy as np
from math import atan2, acos, sin, cos, sqrt, pi, atan

h1 = 205
l2 = 350
d4 = 305
c4 = 209.5

theta = np.array([0.0, pi/2, 0.0, 0.0, pi, 0.0])

T0_1 = np.array([[cos(theta[0]), -sin(theta[0]), 0, 0],[sin(theta[0]), cos(theta[0]), 0, 0],
                 [0, 0, 1, h1], [0, 0, 0, 1]], dtype=float)

T1_2 = np.array([[cos(theta[1]), -sin(theta[1]), 0, 0], [0, 0, -1, 0], 
                 [sin(theta[1]), cos(theta[1]), 0, 0], [0, 0, 0, 1]], dtype=float)

T2_3 = np.array([[cos(theta[2]), -sin(theta[2]), 0, l2],[sin(theta[2]), cos(theta[2]), 0, 0],
                 [0, 0, 1, 0], [0, 0, 0, 1]], dtype=float)

T3_4 = np.array([[cos(theta[3]), -sin(theta[3]), 0, 0], [0, 0, -1, -d4], 
                 [sin(theta[3]), cos(theta[3]), 0, 0], [0, 0, 0, 1]], dtype=float)

T4_5 = np.array([[cos(theta[4]), -sin(theta[4]), 0, 0], [0, 0, 1, 0], 
                 [-sin(theta[4]), -cos(theta[4]), 0, 0], [0, 0, 0, 1]], dtype=float)

T5_6 = np.array([[cos(theta[5]), -sin(theta[5]), 0, 0], [0, 0, -1, c4], 
                 [sin(theta[5]), cos(theta[5]), 0, 0], [0, 0, 0, 1]], dtype=float)

T0_6 = T0_1 @ T1_2 @ T2_3 @ T3_4 @ T4_5 @ T5_6

print(T0_6)
