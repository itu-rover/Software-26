#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String, Float64MultiArray, Int32
import time

class Ara_Gaz:
    def __init__(self):
        rospy.init_node("ara_gaz_node")

        self.twist = Twist()
        self.values =[]
        rospy.Subscriber("/gnss_data", Float64MultiArray, self.gnss_cb)

        self.cmd_pub = rospy.Publisher("/drive_system/twist", Twist, queue_size=10)
        self.can_go_pub = rospy.Publisher("/odom_start",Float32,queue_size=10)
        self.status_pub = rospy.Publisher("/drive_system/status", String, queue_size=10)
        self.initial_cog_pub = rospy.Publisher("/initial_cog",Float32,queue_size=10)
        self.switch_pub = rospy.Publisher("/switch", Int32, queue_size=10)
        self.run()

        
    def gnss_cb(self, msg):
        self.values.append(msg.data[3])

    def run(self):
        start = rospy.Time.now()
        end = rospy.Time.now()
        # self.status_pub.publish('AUTO')
        while (end.to_sec() - start.to_sec()) < 2.5:
            end = rospy.Time.now()
            self.twist.linear.x = 1.5
            self.twist.angular.z = 0
            self.status_pub.publish('AUTO')
            self.switch_pub.publish(Int32(2))
            self.cmd_pub.publish(self.twist)
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.cmd_pub.publish(self.twist)
        self.can_go_pub.publish(Float32(1.0))
        avg = 0
        for i in self.values:
            avg += i
        avg = avg/len(self.values)

        rospy.logerr(avg)
        

        while not rospy.is_shutdown():
            self.initial_cog_pub.publish(Float32(avg))
            time.sleep(1)



if __name__ == "__main__":
    try:
        Ara_Gaz()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")