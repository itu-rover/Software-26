#!/usr/bin/env python3
#author: Emre Demirci

import rospy
from std_msgs.msg import Int16MultiArray, String
import math
import time
import random

class Led:
    def __init__(self):
        #red green blue
        self.r = 0 
        self.g = 0 
        self.b = 255
        self.x = 0
        self.rgb = Int16MultiArray(data=[0, 0, 0])

        #for party mode
        self.colors = (
                       [246, 152, 53], [221, 53, 92], [18, 15, 82],
                       [142, 249, 61], [221, 28, 221], [255, 233, 37],
                       [0, 0, 205], [255, 255, 0], [255, 0, 0], [255, 0, 255]
                       )



        self.mode = "IDLE"

        rospy.init_node('led_publisher', anonymous=True)
        self.sub = rospy.Subscriber("/drive_system/status", String, self.led_cb)
        self.pub = rospy.Publisher("/led_topic", Int16MultiArray, queue_size=10)
        self.rate = rospy.Rate(60)
        self.init()

    def sleep_n_times(self, n):
        for i in range(n):
            self.rate.sleep()

    def light_for_seconds(self, r, g, b, n):
        self.rgb = Int16MultiArray(data=[r, g, b])
        self.pub.publish(self.rgb)
        rospy.sleep(n)

    def init(self):
        while not rospy.is_shutdown():
            if self.mode == "IDLE":
                self.light_for_seconds(255, 255, 0, 0.1)
                
            elif self.mode == "TELEOP_F":
                self.light_for_seconds(0, 0, 255, 0.1)

            elif self.mode == "TELEOP_SLOW":
                self.light_for_seconds(0, 125, 255, 0.1)
                
            elif self.mode == "CHOOSE":
                z = int(abs((self.x % 255) - 112) * 2)
                self.r = z
                self.g = z
                self.x += 4
                self.b = 0
                self.light_for_seconds(self.r, self.g, self.b, 0.001)

            elif self.mode== "SUCCESS":
                self.r = 0
                self.g = int(abs((self.x % 255) - 112) * 2)
                self.x += 4
                self.b = 0
                self.light_for_seconds(self.r, self.g, self.b, 0.001)

            elif self.mode == "PARTY":
                self.r , self.g, self.b = random.choice(self.colors)
                self.light_for_seconds(self.r, self.g, self.b, 0.1)

            elif self.mode == "SMOOTH":
                k = int(abs((self.x % 255) - 127) * 2)
                self.x += 0.5
                self.r = abs(k)
                self.g = 0
                self.b = abs(255 - k)
                self.light_for_seconds(self.r, self.g, self.b, 0.1)
                #rospy.loginfo(k)
            
            elif self.mode == "SIREN":
                self.r = 255
                self.g = 0
                self.b = 0
                self.light_for_seconds(self.r, self.g, self.b, 0.1)
                self.r = 0
                self.g = 0
                self.b = 255
                self.light_for_seconds(self.r, self.g, self.b, 0.1)
                
            elif self.mode == "112":
                self.r = 255
                self.g = 255
                self.b = 255
                self.light_for_seconds(self.r, self.g, self.b, 0.1)
                self.g = 0
                self.b = 0
                self.r = 255
                self.light_for_seconds(self.r, self.g, self.b, 0.1)
            
            elif self.mode == "AUTO" or self.mode == "TELEOP_R":
                self.r = 255
                self.g = 0
                self.b = 0
                self.light_for_seconds(self.r, self.g, self.b, 0.1)
            
            elif self.mode == "EMERGENCY":
                self.r = 255
                self.g = 0
                self.b = 0
                self.light_for_seconds(self.r, self.g, self.b, 0.15)
                self.r = 0
                self.g = 0
                self.b = 255
                self.light_for_seconds(self.r, self.g, self.b, 0.15)
                
            elif self.mode == "SCIENCE":
                self.r = 255 
                self.g = 0
                self.b = 255
                self.light_for_seconds(self.r, self.g, self.b, 0.1)

            	

            self.rate.sleep()

    #Setting the RGB rates
    def led_cb(self, mode):
        self.mode = mode.data  

if __name__ == '__main__':
    try:
        Led()
    except rospy.ROSInterruptException:
        pass
