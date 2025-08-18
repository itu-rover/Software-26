#!/usr/bin/env python3

from mimetypes import init
import tf
import rospy
from sensor_msgs.msg import Imu
import numpy as np

class YawDiff:
    def __init__(self):
        rospy.init_node("yaw_publisher_node")
        self.sub = rospy.Subscriber("/imu/data", Imu, self.imu_cb)
        

        self.curr_imu_yaw = 0
        self.last_imu_yaw = 0
        self.curr_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.init_time = rospy.Time.now()
        self.dt = 0
        self.delta = 0
        
        self.rate = rospy.Rate(2)
        self.run()

    def imu_cb(self,data):
        orientation = data.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, self.curr_imu_yaw) = tf.transformations.euler_from_quaternion(orientation_list)


        
    def run(self):
        while (not rospy.is_shutdown()):
            try:
                self.curr_time = rospy.Time.now()
                self.dt = (self.curr_time - self.last_time).to_sec()
                self.last_time = self.curr_time
                self.dyaw = self.curr_imu_yaw - self.last_imu_yaw
                self.last_imu_yaw = self.curr_imu_yaw

                self.d = self.dyaw / self.dt
                if 59 < (self.last_time - self.init_time).to_sec() < 61:
                    rospy.loginfo(f"time: {self.}")
                self.rate.sleep()
            except Exception as e:
                print(e)
                pass
        


if __name__ == '__main__':
    try:
        YawDiff()
    except rospy.ROSInterruptException:
        pass

                    