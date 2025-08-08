#!/usr/bin/env python3

import numpy as np
from math import atan2, acos, sin, cos, sqrt, pi
import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

class arm_command():
    def __init__(self):
        
        self.pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.pos_msg = JointState()
        self.rate = rospy.Rate(10)
        self.u = np.array([[209.5+535.7, 0, 136.5+542.5]])
        self.r = R.from_euler("zyx", [[0, 90, 0]], degrees= True)
        self.e = self.r.as_dcm()
        self.joint_states = np.array([0, 0, 0, 0, 0, 0])

    def create_msg(self):

        self.pos_msg.header.stamp = rospy.Time().now()
        self.pos_msg.name = ["joint1","joint2","joint3","joint4","joint5","joint6","front_right_steer_joint", "front_right_wheel_joint",
                             "rear_right_steer_joint", "rear_right_wheel_joint", "front_left_steer_joint", "front_left_steer_joint", "front_left_wheel_joint",
                             "rear_left_steer_joint", "rear_left_wheel_joint", "right_leg_joint", "left_leg_joint"]
        self.pos_msg.position = self.inverse_kinematic(self.joint_states, self.u, self.e)
        self.pos_msg.position = self.pos_msg.position  + [0] * 10
        self.pos_msg.velocity=[]
        self.pos_msg.effort=[]
    
    def inverse_kinematic(self, joint_states, u, e):
        #Robotic arm parameters
        a1 = 0
        a2 = 0
        b = 0
        c1 = 136.5
        c2 = 542.5
        c3 = 535.7
        c4 = 209.5
        
        c = np.transpose(u)-(c4)*np.matmul(e,np.transpose(np.array([[0,0,1]])))
        e = e[0]
        #inverse kinematic parameters
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

        mp = e[0][2]*sin(theta2[index2]+theta3)*cos(theta1[index1])+e[1][2]*sin(theta2[index2]+theta3)*sin(theta1[index1])+e[2][2]*cos(theta2[index2]+theta3)
    
        theta4[0] = atan2(e[1][2]*cos(theta1[index1])-e[0][2]*sin(theta1[index1]),
                        e[0][2]*cos(theta2[index2]+theta3)*cos(theta1[index1])+e[1][2]*cos(theta2[index2]+theta3)*sin(theta1[index1])-e[2][2]*sin(theta2[index2]+theta3))
        theta4[1] = theta4[0]+pi
        index4 = np.argmin(abs(joint_states[3]-theta4))
        theta4_min = theta4[index4]

        theta5[0] = atan2(sqrt(1-mp**2), mp)
        theta5[1] = -theta5[0]
        index5 = np.argmin(abs(joint_states[4]-theta5))
        theta5_min = theta5[index5]

        theta6[0] = atan2(e[0][1]*sin(theta2[index2]+theta3)*cos(theta1[index1])+e[1][1]*sin(theta2[index2]+theta3)*sin(theta1[index1])+e[2][1]*cos(theta2[index2]+theta3),
                    -e[0][0]*sin(theta2[index2]+theta3)*cos(theta1[index1])-e[1][0]*sin(theta2[index2]+theta3)*sin(theta1[index1])-e[2][0]*cos(theta2[index2]+theta3))
        theta6[1] = theta6[0]-pi
        index6 = np.argmin(abs(joint_states[5]-theta6))
        theta6_min = theta6[index6]

        return [theta1_min, theta2_min, theta3, theta4_min, theta5_min, theta6_min]

    def publish(self):
        while not rospy.is_shutdown():
            self.create_msg()
            self.pub.publish(self.pos_msg)
            self.rate.sleep()


if __name__=="__main__":
    rospy.init_node("joint_control")
    arm_cmd = arm_command()
    try:
        arm_cmd.publish()
    except rospy.ROSInterruptException:
        pass