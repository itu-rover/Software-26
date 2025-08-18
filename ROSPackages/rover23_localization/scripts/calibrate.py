#!/usr/bin/python3
import rospy

import tf

import numpy as np
from sensor_msgs.msg import Imu



class Odom():
  def __init__(self):
    rospy.init_node('odom_publisher')
    
    self.delta_yaw = 0
    self.delta_time = 0
    self.delta_delta = 0
    self.avg = 0
    
    self.last_yaw = 0
    self.start_yaw = 0
    self.curr_imu_yaw = 0
    self.current_time = 0

    self.total_change = []
    self.rate = rospy.Rate(1)

    self.start_time = rospy.Time.now().to_sec()
    self.time_passed = 0


    self.initialized = False
    rospy.Subscriber("imu/data", Imu, self.imu_callback)

    while not rospy.is_shutdown():
        self.time_passed = rospy.Time.now().to_sec() - self.start_time
        if (self.time_passed) > 300:
            rospy.loginfo(f"curr yaw(degree): {self.curr_imu_yaw / 3.14 * 180}")
        else:
            rospy.loginfo(f"curr yaw(degree): {self.curr_imu_yaw / 3.14 * 180} time passed(s): {int(self.time_passed)}")
        
        self.rate.sleep()

  def imu_callback(self, data):
    orientation = data.orientation
    orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]

    if self.curr_imu_yaw != 0:
        self.last_yaw = self.curr_imu_yaw
    (roll, pitch, self.curr_imu_yaw) = tf.transformations.euler_from_quaternion(orientation_list)

    if self.last_yaw != 0:
        self.current_time = rospy.Time.now().to_sec()
        self.delta_time = self.current_time - self.start_time
        self.delta_yaw = self.curr_imu_yaw-self.last_yaw
        self.total_change.append(self.delta_yaw)



  
def ros_main():
  gpsGoal = Odom()
  

if __name__ == '__main__':
  ros_main()
