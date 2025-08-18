#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

class WheelController:
    def __init__(self):
        rospy.init_node("wheel_controller", anonymous=False)
        rospy.Subscriber("/drive_system_right_motors_feedbacks", Float64MultiArray, self.right_callback)
        rospy.Subscriber("/drive_system_left_motors_feedbacks", Float64MultiArray, self.left_callback)
        rospy.Subscriber("/drive_system/wheel_command", Float64MultiArray, self.wheel_callback)
    
        self.pub = rospy.Publisher("/drive_system/wheel_speed", Float64MultiArray, queue_size=10)
        self.ratio = 1./142.

        self.is_braking = False

        self.errors = np.array([0, 0, 0, 0], dtype = np.float32)          #left_front, left_rear, right_front, right_rear
        self.current_rpms = np.array([0, 0, 0, 0], dtype = np.float32)   #left_front, left_rear, right_front, right_rear

        self.current_target = np.array([0, 0, 0, 0], dtype = np.float32)
        self.rate = rospy.Rate(10)
        self.kp = [0.8, 0.8, 0.8, 0.8]
        self.run()

    def wheel_callback(self, msg):
        if len(msg.data) != 4:
            rospy.warn("Incorrect wheel command!")
        self.current_target[0:4] = np.array(msg.data[0:4]) * self.ratio

    def left_callback(self, msg):
        if len(msg.data) != 2:
            rospy.warn("Incorrect wheel feedback!")
            return
        self.current_rpms[0:2] = msg.data[0:2]

    def right_callback(self, msg):
        if len(msg.data) != 2:
            rospy.warn("Incorrect wheel feedback!")
            return
        self.current_rpms[2:4] = msg.data[0:2]

    def run(self):
        while not rospy.is_shutdown():
            self.loop()
            self.rate.sleep()

    def loop(self):
        self.errors = (self.current_target / self.ratio) - self.current_rpms
        rospy.loginfo(self.errors * self.kp)
        self.pub.publish(data = self.current_target)
        pass


if __name__ == "__main__":
    try:
        WheelController()
    except rospy.ROSInterruptException:
    	pass