#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler
import tf 
from sensor_msgs.msg import Imu, NavSatFix
from math import pi, sin, cos
from std_msgs.msg import Float64MultiArray
import numpy as np

class Khelm():
    def __init__(self):
        rospy.init_node("khelm_odom")

        self.odom = Odometry()
        self.imu = Imu()
        self.fix = NavSatFix()

        self.odom_broadcaster = tf.TransformBroadcaster()

        self.rate = rospy.Rate(20)
        self.current = rospy.Time.now()
        self.last = rospy.Time.now()
        self.pos = Point(0,0,0)
        self.qua = [0,0,0,0]
        self.lat = 0
        self.long = 0
        self.sog = 0 
        self.cog = 0
        self.yaw = 0
        self.current_cog = 0
        self.last_cog = 0

        rospy.Subscriber("/gnss_data", Float64MultiArray, self.gnss_cb)

        self.odom_pub = rospy.Publisher("/khelm/odom", Odometry, queue_size=10)
        self.fix_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)

        self.run()


    def gnss_cb(self, msg:Float64MultiArray):
        self.lat = msg.data[0]
        self.long = msg.data[1]
        self.yaw = msg.data[2] + (111.2*3.14/180)
        self.cog = msg.data[3]
        self.sog = msg.data[4] - 0.1

        # print(np.rad2deg(self.yaw))

        

    
    def run(self):
        while not rospy.is_shutdown():
            self.current = rospy.Time.now()
            dt = (self.current - self.last).to_sec()
            self.current_cog = self.cog

            self.pos.x += self.sog * cos(self.cog) * dt
            self.pos.y += -self.sog * sin(self.cog) * dt
            
            w = (self.last_cog - self.current_cog) / dt

            [self.qua[0], self.qua[1], self.qua[2], self.qua[3]] = quaternion_from_euler(0, 0, self.yaw)

            # self.odom_broadcaster.sendTransform((self.pos.x, self.pos.y, self.pos.z), 
            #                                     self.qua,
            #                                     self.current,
            #                                     "base_link",
            #                                     "odom")

            self.odom.header.frame_id = "odom"
            self.odom.child_frame_id = "base_link"
            self.odom.header.stamp = self.current
            self.odom.pose.pose = Pose(self.pos, Quaternion(self.qua[0], self.qua[1], self.qua[2], self.qua[3]))
            self.odom.twist.twist = Twist(Vector3(self.sog * cos(self.cog), -self.sog * sin(self.cog), 0), Vector3(0, 0, 0))

            self.odom_pub.publish(self.odom)


            self.fix.latitude = self.lat
            self.fix.longitude = self.long
            self.fix.header.frame_id = 'odom'
            self.fix.header.stamp = self.current
            self.fix_pub.publish(self.fix)

            self.last_cog = self.current_cog
            self.last = self.current
            self.rate.sleep()



if __name__ == '__main__':
    Khelm()