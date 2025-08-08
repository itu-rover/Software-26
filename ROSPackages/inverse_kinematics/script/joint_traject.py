#!/usr/bin/env python3

import numpy as np
from math import atan2, acos, sin, cos, sqrt, pi
from scipy.spatial.transform import Rotation as R
import rospy
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class arm_command():
    def __init__(self):
        self.a1 = 0
        self.a2 = 0
        self.b = 0
        self.c1 = 136.5
        self.c2 = 542.5
        self.c3 = 535.7
        self.c4 = 149.5+60
        self.u = np.array([[150, 150, 100]])
        self.r = R.from_euler("zyx", [[30, 45, 45]], degrees= True)
        self.e = self.r.as_dcm()[0]
        self.jt = JointTrajectory()

        self.pub = rospy.Publisher('/rover_arm_controller/command', JointTrajectory, queue_size=10)
        self.rate = rospy.Rate(10)

    def angles(self):
        c = np.transpose(self.u)-135*np.matmul(self.r.as_dcm(),np.transpose(np.array([[0,0,1]])))

        # Positioning part
        nx1 = sqrt(c[0][0]**2+c[0][1]**2-self.b**2)-self.a1
        s1 = sqrt(nx1**2+(c[0][1]-self.c1)**2)
        s2 = sqrt(nx1**2+(c[0][1]-self.c1)**2)
        k = sqrt(self.a2**2+self.c3**2)

        theta1 = atan2(c[0][1], c[0][0])
        theta2 = acos((s1**2+self.c2**2-k**2)/(2*s1*self.c2)) + atan2(nx1, (c[0][2]-self.c1))
        theta3 = acos((s1**2-self.c2**2-k**2)/(2*self.c2*k))

        #Rotation part
        mp = self.e[0][2]*sin(theta2+theta3)*cos(theta1)+self.e[1][2]*sin(theta2+theta3)*sin(theta1)+self.e[2][2]*cos(theta2+theta3)

        theta4 = atan2(self.e[1][2]*cos(theta1)-self.e[0][2]*sin(theta1),
                    self.e[0][2]*cos(theta2+theta3)*cos(theta1)+self.e[1][2]*cos(theta2+theta3)*sin(theta1)-self.e[2][2]*sin(theta2+theta3))
        theta5 = atan2(sqrt(1-mp**2), mp)
        theta6 = atan2(self.e[0][1]*sin(theta2+theta3)*cos(theta1)+self.e[1][1]*sin(theta2+theta3)*sin(theta1)+self.e[2][1]*cos(theta2+theta3),
                    -self.e[0][0]*sin(theta2+theta3)*cos(theta1)-self.e[1][0]*sin(theta2+theta3)*sin(theta1)-self.e[2][0]*cos(theta2+theta3))
        return [theta1, theta2, theta3, theta4, theta5, theta6]
    
    def create_joint_msg(self):
        joint_angles = self.angles()

        self.jt.header.stamp = rospy.Time().now()
        self.jt.joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]
        p = JointTrajectoryPoint()

        p.positions.append(joint_angles[0])
        p.positions.append(joint_angles[1])
        p.positions.append(joint_angles[2])
        p.positions.append(joint_angles[3])
        p.positions.append(joint_angles[4])
        p.positions.append(joint_angles[5])

        self.jt.points.append(p)

    def publish(self):
        while not rospy.is_shutdown():
            self.create_joint_msg()
            self.pub.publish(self.jt)
            self.rate.sleep()

if __name__=="__main__":
    rospy.init_node("joint_control")
    arm_cmd = arm_command()
    try:
        arm_cmd.publish()
    except rospy.ROSInterruptException:
        pass