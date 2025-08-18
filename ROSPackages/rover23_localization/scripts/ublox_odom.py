#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf 
from sensor_msgs.msg import Imu, NavSatFix
from math import pi, sin, cos
from std_msgs.msg import Float64MultiArray, String, Float32, Int32
import numpy as np

class Ublox():
    def __init__(self):
        rospy.init_node("ublox_odom", anonymous=True)

        self.odom = Odometry()
        self.imu = Imu()
        self.fix = NavSatFix()

        self.odom_broadcaster = tf.TransformBroadcaster()

        self.rate = rospy.Rate(10)
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
        self.direction = 0 
        self.rpy = [0,0,0]
        self.listener = tf.TransformListener()

        self.can_go = rospy.wait_for_message("/odom_start", Float32)

        rospy.Subscriber("/gnss_data", Float64MultiArray, self.gnss_cb)
        rospy.Subscriber("/imu/data", Imu, self.imu_cb) 
        rospy.Subscriber("/drive_system/vcu_data", String, self.wheel_cb)

        self.odom_pub = rospy.Publisher("/ublox/odom", Odometry, queue_size=10)
        self.fix_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
        self.initial_rpy_pub = rospy.Publisher("/initial_yaw", Float32, queue_size=10)
        

        self.listener.waitForTransform('/odom', '/base_link', rospy.Time(), rospy.Duration(5.0))

        #odom start was here
        
        self.run()


    def gnss_cb(self, msg:Float64MultiArray):
        self.lat = msg.data[0]
        self.long = msg.data[1]
        self.yaw = msg.data[2]
        self.cog = msg.data[3] # + (0.72*180/3.14)
        self.sog = msg.data[4] if msg.data[4] > 0.05 else 0

        # print(np.rad2deg(self.yaw))


    def imu_cb(self, msg:Imu):        
        self.rpy = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]) 

    def wheel_cb(self, msg:String): 
        self.direction = 1 if msg.data[-8] == "1" else -1
    
    def run(self):
        initial_imu:Imu = rospy.wait_for_message("/imu/data", Imu)
        initial_rpy = self.rpy = euler_from_quaternion([initial_imu.orientation.x, initial_imu.orientation.y, initial_imu.orientation.z, initial_imu.orientation.w]) 
        value = Float32()
        value.data = initial_rpy[2]
        self.initial_rpy_pub.publish(value)
        
        while not rospy.is_shutdown():
            print(self.sog)
            self.current = rospy.Time.now()
            dt = (self.current - self.last).to_sec()
            self.current_cog = self.cog
            cog = self.cog*3.14/180

            rot = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))[1]
            theta = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))[2]

            # rospy.loginfo(((theta*180)/3.14))
            # rospy.loginfo((self.cog-180)/360 + (theta_deg-180)/360)
            # rospy.loginfo((cog-theta)*180/3.14)
            # rospy.loginfo(f"\n\n{np.sign(cos(cog-theta))} ------------- {np.sign(sin(cog-theta))} \n\n")
            # rospy.loginfo(self.direction)
        

            self.pos.x += self.sog * cos(theta) * self.direction * dt
            self.pos.y += self.sog * sin(theta) * self.direction * dt
            
            w = (self.last_cog - self.current_cog) / dt

            [self.qua[0], self.qua[1], self.qua[2], self.qua[3]] = quaternion_from_euler(0, 0, (self.rpy[2]-initial_rpy[2]))

            # self.odom_broadcaster.sendTransform((self.pos.x, self.pos.y, self.pos.z), 
            #                                     self.qua,
            #                                     self.current,
            #                                     "base_link",
            #                                     "odom")

            self.odom.header.frame_id = "odom"
            self.odom.child_frame_id = "base_link"
            self.odom.header.stamp = self.current
            self.odom.pose.pose = Pose(self.pos, Quaternion(self.qua[0], self.qua[1], self.qua[2], self.qua[3]))
            self.odom.twist.twist = Twist(Vector3(self.sog * cos(theta) * self.direction, self.sog * sin(theta) * self.direction, 0), Vector3(0, 0, 0))

            self.odom_pub.publish(self.odom)


            self.fix.latitude = self.lat
            self.fix.longitude = self.long
            self.fix.header.frame_id = 'odom'
            self.fix.header.stamp = self.current
            self.fix_pub.publish(self.fix)

            self.last_cog = self.current_cog
            self.last = self.current
            self.sog = 0
            self.rate.sleep()



if __name__ == '__main__':
    Ublox()