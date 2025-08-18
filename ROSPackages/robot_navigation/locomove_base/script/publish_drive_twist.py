#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

rospy.init_node("newtwist_publisher")


def nav_callback(data:Twist):
    newtwist = data
    
    newtwist.linear.x = 0
    newtwist.linear.y = 0
    newtwist.linear.z = 0

    newtwist.angular.x = 0
    newtwist.angular.y = 0
    newtwist.angular.z =1

    twist_publisher.publish(newtwist)

twist_publisher = rospy.Publisher("/drive_system/twist", Twist, queue_size=10)
nav_sub = rospy.Subscriber("/nav_vel",Twist,callback=nav_callback)

rospy.spin()



