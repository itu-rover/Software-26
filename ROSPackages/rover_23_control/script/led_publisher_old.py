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
        self.r = 255 
        self.g = 255 
        self.b = 0
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
        self.rate = rospy.Rate(6)
        self.rate_wait = rospy.Rate(12)
        self.init()
        rospy.spin()

    def sleep_n_times(self, n):
        for i in range(n):
            self.rate.sleep()

    def init(self):
        while not rospy.is_shutdown():
            self.rgb = Int16MultiArray(data=[self.r, self.g, self.b])
            self.pub.publish(self.rgb)
            #rospy.loginfo(f"{self.mode}")
            self.rate.sleep()

    #Setting the RGB rates
    def led_cb(self, mode):
        self.mode = mode.data
        #rospy.loginfo(self.mode)
        # print("LED MODU: "+self.mode)
        if self.mode == "IDLE":
            self.r = 255
            self.g = 255
            self.b = 0

        elif self.mode == "TELEOP_F":
            self.r = 0
            self.g = 0
            self.b = 255
            
        elif self.mode == "TELEOP_R":
            self.r = 0
            self.g = 0
            self.b = 255
            
        elif self.mode == "CHOOSE":
            self.r = 255
            self.g = 255
            self.b = 0

        elif self.mode== "SUCCESS":
            self.r = 0
            self.g = int(abs((self.x % 255) - 112) * 2)
            self.x += 1
            self.b = 0
            #rospy.loginfo(self.g)
            rospy.sleep(0.041)
            self.r = 255
            self.g = 255
            self.b = 255
            self.rate.sleep()
            # self.rate.sleep()
            # rospy.sleep(0.05)

        elif self.mode == "PARTY":
            self.r , self.g, self.b = random.choice(self.colors)
            self.sleep_n_times(45)

        elif self.mode == "SMOOTH":
            k = int(abs((self.x % 255) - 127) * 2)
            self.x += 0.5
            self.r = abs(k)
            self.g = 0
            self.b = abs(255 - k)
            #rospy.loginfo(k)
            self.rate.sleep()
        
        elif self.mode == "SIREN":
            self.r = 255
            self.g = 0
            self.b = 0
            rospy.sleep(0.041)
            self.r = 0
            self.g = 0
            self.b = 255
            self.rate.sleep()
            
        elif self.mode == "112":
            self.r = 255
            self.g = 255
            self.b = 255
            rospy.sleep(0.041)
            self.g = 0
            self.b = 0
            self.r = 255
            self.rate.sleep()
        
        elif self.mode == "AUTO":
            self.r = 255
            self.g = 0
            self.b = 0
        
        elif self.mode == "EMERGENCY":
            self.r = 255
            self.g = 0
            self.b = 0
            #rospy.loginfo(self.g)
            self.rate.sleep()
            self.rate.sleep()
            self.rate.sleep()
            self.r = 255
            self.g = 69
            self.b = 0
            self.rate.sleep()  
            self.rate.sleep()
            self.rate.sleep()         

if __name__ == '__main__':
    try:
        Led()
    except rospy.ROSInterruptException:
        pass
