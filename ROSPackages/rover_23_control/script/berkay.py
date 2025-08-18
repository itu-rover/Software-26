#!/usr/bin/env python3

# @author alpogant
# Edited by Berkay at 29 May 2024
# Date: 9 May 2024

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

class TeleopJoy:
    def __init__(self):
        rospy.init_node("joy_to_twist_node", anonymous=False)

        self.twist = Twist()
        self.turbo_multiplier = 4

        self.rate = rospy.Rate(50)
        self.lb = 0
        self.rb = 0
        
        self.twist = Twist()  # Create empty twist
        self.mux_twist = Twist()
        self.nav_twist = Twist()
        
        self.mode = "AUTO"
        self.autonomous = False
        self.is_emergency = False
        
        
        self.init()
        self.run()

    def init(self):
        rospy.loginfo("Joystick teleoperation initialized")
        rospy.Subscriber("/joy", Joy, self.joy_cb)
        rospy.Subscriber("/drive_system/twist",Twist, self.twist_cb)
        rospy.Subscriber("/nav_vel", Twist, self.nav_cb)
        rospy.Subscriber("/locomove_base/is_reached", String, self.reached_cb)  # Subscribe to get is reached info
        self.e_stop_pub = rospy.Publisher("/emergency_stop", Bool, queue_size=10)
        self.cmd_pub = rospy.Publisher("/joystick/twist", Twist, queue_size=10)
        self.nav_pub = rospy.Publisher("/nav_vel", Twist,queue_size=10)
        self.status_publisher = rospy.Publisher("/drive_system/status", String, queue_size=10)
        
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


    def joy_cb(self, data):
        self.lb = data.buttons[4] # L1 button -> normal mode
        self.rb = data.buttons[5] # R1 button -> turbo mode

        self.angular_axis = data.axes[0] # Left analog stick's right and left axes
        self.linear_axis = data.axes[1] # Left analog stick's up and down axes
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
        
    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo(self.mode)
    
            if self.is_emergency:
                print("aaaaa")
                self.status_publisher.publish("EMERGENCY")
                self.e_stop_pub.publish(True)
                if self.mode == "IDLE":
                    self.is_emergency = False
    
            else:
                self.e_stop_pub.publish(False)
                if self.autonomous:
                    pass
                    # self.status_publisher.publish("AUTO")
                if self.mode == "IDLE":
                    pass
                    self.is_emergency = False
                    # self.status_publisher.publish("IDLE")
                elif self.mode == "CHOOSE":
                    pass
                    # self.status_publisher.publish("CHOOSE")
                elif self.mode == "SUCCESS":
                    self.nav_twist.linear.x = 0
                    self.nav_twist.angular.z = 0
                    self.nav_pub.publish(self.nav_twist)
                    # self.status_publisher.publish("SUCCESS")
                elif self.mode == "DRIVE" or self.mode == "SLOW_DRIVE":
                    if self.mode == "SLOW_DRIVE":
                        pass
                        # self.status_publisher.publish("TELEOP")
                    else:
                        pass
                        # self.status_publisher.publish("TELEOP")
                    
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
    
    
    
            self.rate.sleep()


if __name__ == "__main__":
    try:
        TeleopJoy()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")
