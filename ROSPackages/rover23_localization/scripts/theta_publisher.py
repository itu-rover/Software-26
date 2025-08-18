#!/usr/bin/env python3
import rospy
import tf2_ros
import tf
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32
from math import pi

class Theta():
    def __init__(self):
        rospy.init_node('theta_publisher')

        self.rate = rospy.Rate(10)

        self.theta_pub = rospy.Publisher('/theta', Float32, queue_size=100)
        
        self.done = 0
        self.tf_buffer = tf2_ros.Buffer()
        # self.listener = tf2_ros.TransformListener(self.tf_buffer) 
        self.listener = listener = tf.TransformListener()

        self.run()



    def input(self):
        input1 = int(input())
        if input1 == 1: self.done = 1

    def run(self):
        theta_1 = Float32()
        while not rospy.is_shutdown():
            if not self.done:
                try:
                    self.listener.waitForTransform('/base_link', '/odom', rospy.Time(), rospy.Duration(5.0))
                    rot = self.listener.lookupTransform('/base_link', '/odom', rospy.Time(0))[1]
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logerr("Transform lookup failed")
                    return None
                # if self.tf_buffer.can_transform('odom','base_link',rospy.Time(0)): 
                #     trans:TransformStamped = self.tf_buffer.lookup_transform('odom','base_link',rospy.Time(0))

            theta = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))[2]
            # theta = abs(theta)
            # theta = (theta) if theta <= pi else (pi - theta)

            theta_1.data = theta
            self.theta_pub.publish(theta_1)
            rospy.loginfo((theta * 180 / pi))

            if not self.done:
                self.input()

            self.rate.sleep()


if __name__ == '__main__':
    Theta()

