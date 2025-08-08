#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String, Float32MultiArray
from moveit_commander.conversions import pose_to_list
import numpy as np
from math import atan2, acos, sin, cos, sqrt, pi, atan
from scipy.spatial.transform import Rotation as R


class MoveGroupPythonIntefaceTutorial(object):

    def __init__(self):

        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('joint_control')

        self.robot = moveit_commander.RobotCommander()

        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "rover_arm"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        rospy.Subscriber("keyboard", Float32MultiArray, self.cllback)
        
        self.planning_frame = self.group.get_planning_frame()

        self.eef_link = self.group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        
        self.u = np.array([[535.70+209.5, 0, 136.5+542.5]])
        r = R.from_euler("zyx", [[0, 90, 0]], degrees= True)
        self.e = r.as_dcm()
        rospy.spin()

    def cllback(self, msg):
        msg = np.array([msg.data], float)
        self.u = self.u + 10*msg
        self.go_to_joint_state()

    def go_to_joint_state(self):
        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        ## thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = self.group.get_current_joint_values()

        joint_goal = self.inverse_kinematic(joint_goal, self.u, self.e)
        print(joint_goal)
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()
        ## END_SUB_TUTORIAL
        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return np.allclose(joint_goal, current_joints, 0.01)
    
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
        if (theta4_min > 3.14):
            theta4_min = 0
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
    
def main():
    try:
        goal_publisher = MoveGroupPythonIntefaceTutorial()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    
if __name__=="__main__":
    main()