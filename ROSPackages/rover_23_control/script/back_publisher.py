#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Vector3

class Back:
    def __init__(self):
        rospy.init_node("back_node")

        self.twist = Twist()

        self.cmd_pub = rospy.Publisher("/drive_system/twist", Twist, queue_size=10)
        self.run()

        
        

    def run(self):
        start = rospy.Time.now()
        end = rospy.Time.now()
        while (end.to_sec() - start.to_sec()) < 2:
            end = rospy.Time.now()
            self.twist.linear.x = -0.5
            self.twist.angular.z = 0
        
            self.cmd_pub.publish(self.twist)


if __name__ == "__main__":
    try:
        Back()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")