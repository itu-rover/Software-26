#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool


class LedControl:
    def __init__(self):
        rospy.init_node("led")

        self.pub = rospy.Publisher("/led_topic", String, queue_size=10)

        self.flag_nav = False
        self.time_nav = rospy.Time.now().secs

        self.flag_joy = False

        self.flag_arm_teleop = False
        self.time_arm_teleop = rospy.Time.now().secs
        self.flag_arm_sm = False
        self.time_arm_sm = rospy.Time.now().secs

        rospy.set_param("/mydelay", 1.5)
        self.time_joy = rospy.Time.now().secs
        if rospy.has_param("/mydelay"):
            self.param = rospy.get_param("/mydelay")
        else:
            self.param = 0.5

        rospy.Subscriber("/joystick/twist", Twist, self.joy_cb)
        rospy.Subscriber("/nav_vel", Twist, self.nav_cb)
        rospy.Subscriber("/manipulator/is_state_machine_on", String, self.arm_sm_cb)
        rospy.Subscriber("/manipulator/current_mode", String, self.manipulator_status_cb)

        rospy.Service("/led_control/set_party_mode", SetBool, self.toggle_party_mode)
        rospy.Service("/led_control/set_blink_mode", SetBool, self.toggle_blink_mode)

        self.is_blinking = True
        self.party_mode = False

        rospy.on_shutdown(self.on_shutdown)

        self.rate = rospy.Rate(10)
        self.run()

    def toggle_party_mode(self, req):
        self.party_mode = req.data

    def toggle_blink_mode(self, req):
        self.is_blinking = req.data

    def on_shutdown(self):
        self.pub.publish("T")

    def arm_sm_cb(self, data):
        self.flag_arm_sm = True
        self.time_arm_sm = rospy.Time.now().secs

    def manipulator_status_cb(self, data):
        if data.data == "JOINT" or data.data == "CARTESIAN":
            self.flag_arm_teleop = True
            self.time_arm_teleop = rospy.Time.now().secs
        elif data.data == "IDLE":
            self.flag_arm_teleop = False

    def joy_cb(self, data):
        self.flag_joy = True
        self.time_joy = rospy.Time.now().secs

    def nav_cb(self, data):
        self.flag_nav = True
        self.time_nav = rospy.Time.now().secs

    def run(self):

        while not rospy.is_shutdown():
            if self.party_mode:
                self.pub.publish("R")
                self.rate.sleep()
                if not self.is_blinking:
                    self.rate.sleep()
                    self.rate.sleep()
                self.pub.publish("Y")
                self.rate.sleep()
                self.pub.publish("G")
                self.rate.sleep()
                if not self.is_blinking:
                    self.rate.sleep()
                    self.rate.sleep()
                self.pub.publish("Y")
                self.rate.sleep()
                continue

            if self.is_blinking:
                self.pub.publish("T")

            self.rate.sleep()

            if(self.flag_joy or self.flag_arm_teleop):
                self.pub.publish("G")
                print("Green")

            elif(self.flag_nav or self.flag_arm_sm):
                self.pub.publish("R")
                print("Red")

            else:
                self.pub.publish("Y")
                print("Yellow")

            if(int(rospy.Time.now().secs) - int(self.time_joy) > self.param):
                self.flag_joy = False

            if(int(rospy.Time.now().secs) - int(self.time_nav) > self.param):
                self.flag_nav = False

            if(int(rospy.Time.now().secs) - int(self.time_arm_sm) > self.param):
                self.flag_arm_sm = False

            if(int(rospy.Time.now().secs) - int(self.time_arm_teleop) > self.param):
                self.flag_arm_teleop = False
            self.rate.sleep()


if __name__ == "__main__":
    LedControl()
