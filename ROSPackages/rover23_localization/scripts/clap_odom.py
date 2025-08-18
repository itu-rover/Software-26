#!/usr/bin/env python3
import rospy
from clap_b7_driver.msg import ClapIns, ClapHeading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler
import tf 
from sensor_msgs.msg import Imu
from math import pi, sin, cos

class Clap():
    def __init__(self):
        rospy.init_node("clap_odom")

        self.ins = ClapIns()
        self.odom = Odometry()
        self.imu = Imu()
        self.heading = 0

        self.odom_broadcaster = tf.TransformBroadcaster()

        self.pos = Point(0,0,0)
        self.qua = [0,0,0,0]

        self.rate = rospy.Rate(10)
        self.current = rospy.Time.now()
        self.last = rospy.Time.now()

        self.ins_sub = rospy.Subscriber("/clap/clap_ins", ClapIns, callback=self.ins_cb)
        self.heading_sub = rospy.Subscriber("/clap/heading", ClapHeading, callback=self.heading_cb)
        self.imu_sub = rospy.Subscriber("/clap/ros/imu", Imu, callback=self.imu_cb)

        self.odom_pub = rospy.Publisher("/clap/odom", Odometry, queue_size=10)
        #self.imu_pub = rospy.Publisher("imu/data", Imu, queue_size=10)

        self.calc()


    def ins_cb(self, msg:ClapIns):
        self.ins = msg

    def imu_cb(self, msg:Imu):
        self.imu = msg
        #self.imu_pub.publish(msg)

    def heading_cb(self, msg:ClapHeading):
        self.heading = (msg.heading) * (pi / 180)
        #print("yaw:", (360-msg.heading))


    def calc(self):
        while not rospy.is_shutdown():
            self.current = rospy.Time.now()
            dt = (self.current - self.last).to_sec()

            self.pos.y += self.ins.north_velocity*dt
            self.pos.x += self.ins.east_velocity*dt

            [self.qua[0], self.qua[1], self.qua[2], self.qua[3]] = quaternion_from_euler(0, 0, pi/2-self.heading)

            # self.odom_broadcaster.sendTransform((self.pos.x, self.pos.y, self.pos.z), 
            #                                     self.qua,
            #                                     self.current,
            #                                     "base_link",
            #                                     "odom")

            self.odom.header.frame_id = "odom"
            self.odom.child_frame_id = "base_link"
            self.odom.header.stamp = self.current

            self.odom.pose.pose = Pose(self.pos, Quaternion(self.qua[0], self.qua[1], self.qua[2], self.qua[3]))
            self.odom.twist.twist = Twist(Vector3(self.ins.north_velocity, self.ins.east_velocity, 0), Vector3(self.imu.angular_velocity.x, self.imu.angular_velocity.y, self.imu.angular_velocity.z))

            self.odom_pub.publish(self.odom)

            self.last = self.current
            self.rate.sleep()



if __name__ == '__main__':
    Clap()
