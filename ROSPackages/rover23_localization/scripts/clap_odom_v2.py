#!/usr/bin/env python3
import rospy
from clap_b7_driver.msg import ClapIns, ClapHeading, ClapECEF
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu
from math import pi, sin, cos
from sensor_msgs.msg import NavSatFix
import pyproj

class Clap():
    def __init__(self):
        rospy.init_node("clap_odom")

        self.ins = ClapIns()
        self.ecef = ClapECEF()
        self.odom = Odometry()
        self.imu = Imu()
        self.heading = 0

        self.pos = Point(0,0,0)
        self.qua = [0,0,0,0]

        self.rate = rospy.Rate(10)
        self.current = rospy.Time.now()
        self.last = rospy.Time.now()

        self.ins_sub = rospy.Subscriber("/clap/clap_ins", ClapIns, callback=self.ins_cb)
        self.heading_sub = rospy.Subscriber("/clap/heading", ClapHeading, callback=self.heading_cb)
        self.odom_pub = rospy.Publisher("/clap/odom", Odometry, queue_size=10)
        rospy.Subscriber('/ublox_gps/fix', NavSatFix, self.gps_callback)
        self.gps_data = NavSatFix()
        self.ecef = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
        self.lla = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')
        self.calc

    def gps_callback(self, msg):
        self.gps_data = msg

    def ecef_cb(self, msg:ClapECEF):
        self.ecef = msg
    def ins_cb(self, msg:ClapIns):
        self.ins = msg

    def heading_cb(self, msg:ClapHeading):
        self.heading = (msg.heading) * (pi / 180)

    def calc(self):
        for i in range(10):
            x_ref, y_ref,_= pyproj.transform(self.lla, self.ecef, self.gps_data.longitude, self.gps_data.latitude, self.gps_data.altitude)
            self.rate.sleep()

        while not rospy.is_shutdown():
            
            x, y,_ = pyproj.transform(self.lla, self.ecef, self.gps_data.longitude, self.gps_data.latitude, self.gps_data.altitude)

            self.pos.x = y-y_ref
            self.pos.y = -(x-x_ref)

            [self.qua[0], self.qua[1], self.qua[2], self.qua[3]] = quaternion_from_euler(0, 0, 3/2*pi-self.heading)

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
