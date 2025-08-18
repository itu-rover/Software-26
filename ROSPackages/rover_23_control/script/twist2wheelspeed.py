#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class TeleopJoy:
    def __init__(self):
        rospy.init_node("twist2wheel_speed_node")
        rospy.Subscriber("/joystick/twist", Twist, self.twist_cb)  # Subscribe to joy topic to get joystick data
        self.pub = rospy.Publisher("/drive_system/wheel_command", Float64MultiArray, queue_size=10)  # Publish velocity data to /cmd_vel topic

        self.rate = rospy.Rate(200)  # Publish 25 data every second

        self.wheel_speed_pub()

		self.linear_x = 0
		self.angular_z = 0

		self.rpm_left = 0
		self.rpm_right = 0
		self.left_wheel = 0
		self.right_wheel = 0

        self.WHEEL_RADIUS = 0.135

		self.wheel_speeds = Float64MultiArray()

	def calc(self)
		self.left_wheel = (self.linear_x - self.angular_z) / 2
        self.right_wheel = (self.linear_x + self.angular_z) / 2

        self.rpm_left = (self.left_wheel / (2 * pi * self.WHEEL_RADIUS)) * 60
        self.rpm_right = (self.right_wheel / (2 * pi * self.WHEEL_RADIUS)) * 60
            
        self.rpm_left = float(saturate(map(self.rpm_left)))
        self.rpm_right = float(saturate(map(self.rpm_right)))

    def twist_cb(self, data):
        self.linear_x = data.linear.x
        self.angular_z = data.angular.z

    def wheel_publisher(self):
		while not rospy.is_shutdown():
			self.wheel_speeds.data = [self.rpm_left, self.rpm_left, self.rpm_right, self.rpm_right, abs(self.pitch_change)]		
			self.pub.publish(wheel_speeds_data)
			self.rate.sleep()


if __name__ == "__main__":
    teleop = TeleopJoy()

    rospy.spin()
