#!/usr/bin/env python3

import numpy as np
from math import atan2, acos, sin, cos, sqrt, pi, atan
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt 

def position_part(joint_states, u, e):
    #Robotic arm parameters
    a1 = 0
    a2 = 0
    b = 0
    c1 = 136.5
    c2 = 542.5
    c3 = 535.7
    c4 = 209.5
    c = np.transpose(u)-(c4)*np.matmul(e,np.transpose(np.array([[0,0,1]])))
    #inverse kinematic parameters
    e = e[0]
    print(c)
    nx1 = sqrt(c[0][0]**2+c[0][1]**2-b**2)-a1
    s1 = sqrt(nx1**2+(c[0][2]-c1)**2)
    s2 = sqrt((nx1+2*a1)**2+(c[0][2]-c1)**2)
    k = sqrt(a2**2+c3**2)
    
    theta1 = np.zeros((2,), dtype=float)
    theta2 = np.zeros((2,), dtype=float)
    theta4 = np.zeros((2,), dtype=float)
    theta5 = np.zeros((2,), dtype=float)
    theta6 = np.zeros((2,), dtype=float)

    theta1[0] = atan2(c[0][1], c[0][0])-atan2(b, nx1+a1)
    theta1[1] = atan2(c[0][1], c[0][0])+atan2(b, nx1+a1)-pi
    index1 = np.argmin(abs(joint_states[0]-theta1))
    theta1_min = theta1[index1]
    
    if (index1 == 0):
        theta2[0] = -acos((s1**2+c2**2-k**2)/(2*s1*c2)) + (atan2(nx1, (c[0][2]-c1)))
        theta2[1] = acos((s1**2+c2**2-k**2)/(2*s1*c2)) + (atan2(nx1, (c[0][2]-c1)))
    else:
        theta2[0] = -acos((s2**2+c2**2-k**2)/(2*s2*c2)) - (atan2(nx1+2*a1, (c[0][2]-c1)))
        theta2[1] = acos((s2**2+c2**2-k**2)/(2*s2*c2)) - (atan2(nx1+2*a1, (c[0][2]-c1)))

    index2 =  np.argmin(abs(joint_states[1]-theta2))
    theta2_min = theta2[index2]

    if (index2 == 0):
        theta3 = acos((s1**2-c2**2-k**2)/(2*c2*k))
    else:
        theta3 = -acos((s1**2-c2**2-k**2)/(2*c2*k))

    mp = e[0][2]*sin(theta2_min+theta3)*cos(theta1_min)+e[1][2]*sin(theta2_min+theta3)*sin(theta1_min)+e[2][2]*cos(theta2_min+theta3)
   
    theta4[0] = atan2(e[1][2]*cos(theta1_min)-e[0][2]*sin(theta1_min),
                    e[0][2]*cos(theta2_min+theta3)*cos(theta1_min)+e[1][2]*cos(theta2_min+theta3)*sin(theta1_min)-e[2][2]*sin(theta2_min+theta3))
    theta4[1] = theta4[0]+pi
    index4 = np.argmin(abs(joint_states[3]-theta4))
    theta4_min = theta4[index4]

    theta5[0] = atan2(sqrt(1-mp**2), mp)
    theta5[1] = -theta5[0]
    index5 = np.argmin(abs(joint_states[4]-theta5))
    theta5_min = theta5[index5]

    theta6[0] = atan2(e[0][1]*sin(theta2_min+theta3)*cos(theta1_min)+e[1][1]*sin(theta2_min+theta3)*sin(theta1_min)+e[2][1]*cos(theta2_min+theta3),
                -e[0][0]*sin(theta2_min+theta3)*cos(theta1_min)-e[1][0]*sin(theta2_min+theta3)*sin(theta1_min)-e[2][0]*cos(theta2_min+theta3))
    theta6[1] = theta6[0]-pi
    index6 = np.argmin(abs(joint_states[5]-theta6))
    theta6_min = theta6[index6]

    # print(theta1, "\n", theta2, "\n", theta3, "\n", theta4, "\n", theta5, "\n", theta6)
    return {"theta1":theta1_min,
            "theta2":theta2_min,
            "theta3":theta3,
            "theta4":theta4_min,
            "theta5":theta5_min,
            "theta6":theta6_min}


u = np.array([[535.70+209.5, 0, 136.5+542.5]])
r = R.from_euler("zyx", [[0, 90, 0]], degrees= True)
e = r.as_dcm()
joint_states = np.array([0, 0, 0, 0, 0, 0])

angles = position_part(joint_states, u, e)
print(angles)
