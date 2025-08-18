#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

class RemoveAntenna:
    def __init__(self):
        rospy.init_node("remove_antenna")

        rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.pub = rospy.Publisher("/scan_filtered", LaserScan, queue_size=10)

        rospy.spin()
        pass

    def scan_cb(self, msg):
        for i, val in enumerate(msg.ranges):
            if val < 1.0:
                rospy.loginfo(msg.ranges[i])
        self.pub.publish(msg)


if __name__ == "__main__":
    try:
        RemoveAntenna()
    except:
        pass