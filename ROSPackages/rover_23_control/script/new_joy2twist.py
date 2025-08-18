#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

class TeleopJoy:
    def __init__(self):
        rospy.init_node("joy_to_twist_node")
        rospy.Subscriber(
            "/joy", Joy, self.joy_cb
        )  # Subscribe to joy topic to get joystick data
        rospy.Subscriber(
            "/locomove_base/is_reached", String, self.reached_cb
        )  # Subscribe to get is reached info
        self.pub = rospy.Publisher(
            "/joystick/twist", Twist, queue_size=10
        )  # Publish velocity data to /cmd_vel topic

        self.e_stop_pub = rospy.Publisher("/emergency_stop", Bool, queue_size=10)
        self.status_publisher = rospy.Publisher("/drive_system/status", String, queue_size=10)

        rospy.Subscriber("/cmd_vel",Twist, self.twist_cb)
        rospy.Subscriber("/nav_vel", Twist, self.nav_cb)

        self.pub_lora = rospy.Publisher("/lora/gs/transmit/twist", Twist, queue_size=10)

        self.last_joy_data = Joy()

        self.twist = Twist()  # Create empty twist
        self.mux_twist = Twist()
        self.nav_twist = Twist()
        
        self.angular_axis = 0  # Zero is left analog stick's right and left axes
        self.linear_axis = 1  # One is left analog stick's up and down axes
        self.rb = 5  # R1 button -> turbo mode
        self.lb = 4  # L1 button -> normal mode

        self.max_linear_velocity = 1
        self.max_angular_velocity = 1
        self.turbo_multiplier = 2  # Turbo mode velocity multiplier

        self.rate = rospy.Rate(25)  # Publish 25 data every second

        self.mode = "IDLE"
        self.autonomous = False
        self.is_emergency = False

        self.timeout = 0.5

        self.run()


    def switch_mode(self, buttons, axes):
        if(buttons[8]):
            self.mode = "IDLE"

        if(axes[2] == -1 and axes[5] == -1):
            self.is_emergency = True
            
        if(self.mode == "IDLE" and buttons[7]):
            self.mode = "CHOOSE"

        if(self.mode == "CHOOSE" and buttons[0]):
            self.mode = "DRIVE"

        if(self.mode == "CHOOSE" and buttons[3]):
            self.mode = "SLOW_DRIVE"
        
        if(self.mode == "CHOOSE" and buttons[1]):
            self.mode = "SCIENCE"

    # Get joystick data and convert it to velocity data
    # data: Joystick push data
    def joy_cb(self, data):
        if(len(data.buttons) < 5):
            return
        self.last_joy_data = data
        self.last_joy_time = rospy.Time.now().to_sec()
        self.switch_mode(data.buttons, data.axes)

    def reached_cb(self, data):
        if self.autonomous:
            self.autonomous = False
            self.mode = "SUCCESS"

    def nav_cb(self, data):
        self.nav_twist = data
        if(self.nav_twist == self.mux_twist) and (self.mode == "IDLE"):
            self.autonomous = True
            self.mode = "AUTO"

    def twist_cb(self, data):
        self.mux_twist = data

    # Publish velocity data non-stop
    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.mode)

            if self.is_emergency:
                self.status_publisher.publish("EMERGENCY")
                self.e_stop_pub.publish(True)
                if self.mode == "IDLE":
                    self.is_emergency = False

            else:
                self.e_stop_pub.publish(False)
                if self.autonomous:
                    self.status_publisher.publish("AUTO")
                if self.mode == "IDLE":
                    self.is_emergency = False
                    self.status_publisher.publish("IDLE")
                if self.mode == "CHOOSE":
                    self.status_publisher.publish("CHOOSE")
                if self.mode == "SCIENCE":
                    self.status_publisher.publish("SCIENCE")
                if self.mode == "SUCCESS":
                    self.status_publisher.publish("SUCCESS")
                elif self.mode == "DRIVE" or self.mode == "SLOW_DRIVE":
                    if self.mode == "SLOW_DRIVE":
                        self.status_publisher.publish("TELEOP_SLOW")
                    else:
                        self.status_publisher.publish("TELEOP_F")

                    self.multiplier = 0.25 if self.mode == "SLOW_DRIVE" else 1
                    if self.last_joy_data.buttons[self.lb]:
                        self.twist.linear.x = self.last_joy_data.axes[self.linear_axis] * self.max_linear_velocity * self.multiplier
                        self.twist.angular.z = self.last_joy_data.axes[self.angular_axis] * self.max_angular_velocity * self.multiplier
                    elif self.last_joy_data.buttons[self.rb]:
                        self.twist.linear.x = self.last_joy_data.axes[self.linear_axis] * self.max_linear_velocity * self.turbo_multiplier * self.multiplier
                        self.twist.angular.z = self.last_joy_data.axes[self.angular_axis] * self.max_angular_velocity * self.turbo_multiplier * self.multiplier
                    else:  # If RB or LB is not pressed, stop vehicle
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0
                    if(rospy.Time.now().to_sec() - self.last_joy_time < self.timeout):
                        self.pub.publish(self.twist)

            self.rate.sleep()

if __name__ == "__main__":
    teleop = TeleopJoy()

    rospy.spin()
