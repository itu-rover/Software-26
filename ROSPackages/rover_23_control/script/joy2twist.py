#!/usr/bin/env python3

# @author alpogant
# Date: 9 May 2024

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TeleopJoy:
    def __init__(self):
        rospy.init_node("joy_to_twist_node", anonymous=False)

        self.twist = Twist()
        self.turbo_multiplier = 4

        self.rate = rospy.Rate(25)
        self.lb = 0
        self.rb = 0
        self.init()
        self.run()

    def init(self):
        rospy.loginfo("Joystick teleoperation initialized")
        rospy.Subscriber("/joy", Joy, self.joy_cb)
        self.cmd_pub = rospy.Publisher("/joystick/twist", Twist, queue_size=10)
        self.status_pub = rospy.Publisher("/drive_system/status", String, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            if self.lb:
                self.twist.linear.x = self.linear_axis * 2.0
                self.twist.angular.z = self.angular_axis * 2.0
            elif self.rb:
                self.twist.linear.x = self.linear_axis * self.turbo_multiplier
                self.twist.angular.z = self.angular_axis * self.turbo_multiplier
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0

            self.cmd_pub.publish(self.twist)

            self.pubs = String()
            self.pubs.data = "TELEOP" # If this node is running, rover is teleoperating
            self.status_pub.publish(self.pubs) 

            self.rate.sleep()

    def joy_cb(self, data):
        self.lb = data.buttons[4] # L1 button -> normal mode
        self.rb = data.buttons[5] # R1 button -> turbo mode

        self.angular_axis = data.axes[0] # Left analog stick's right and left axes
        self.linear_axis = data.axes[1] # Left analog stick's up and down axes

if __name__ == "__main__":
    try:
        TeleopJoy()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")
