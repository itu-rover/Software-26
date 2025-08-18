#!/usr/bin/python
import time
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import math

class joyPublisher():
    def __init__(self):
        rospy.init_node("joyPublisher")
        self.y = 0
        self.x = 0
        self.move_cmd = Twist()
        self.drivePublisher = rospy.Publisher("/joystick/twist",Twist,queue_size=10)
        rospy.Subscriber("/joy",Joy,self.callback)
        self.toggle_button = 0
        self.last_time = rospy.Time.now().to_sec()
        self.WHEEL_RADIUS = 0.27 / 2		
        self.multiplier = 2 * math.pi * self.WHEEL_RADIUS
        self.x_limit = 1 * self.multiplier
        self.z_limit = 1 * self.multiplier

    def callback(self,data):
        self.move_cmd.linear.x = max(-self.x_limit, min(data.axes[1], self.x_limit))
        self.move_cmd.angular.z = max(-self.z_limit, min(data.axes[0], self.z_limit))
        if rospy.Time.now().to_sec() - self.last_time > 0.5:
            if data.buttons[5]:
                self.toggle_button  = not self.toggle_button
                self.last_time = rospy.Time.now().to_sec()
        
    def publis(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.toggle_button:
                self.drivePublisher.publish(self.move_cmd)

            rate.sleep()
            

if __name__ == "__main__":
        joyPublisher().publis()
