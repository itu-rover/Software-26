#!/usr/bin/python3
# Author: B. Burak Payzun

import time
import rospy
from sensor_msgs.msg import Joy, Imu
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
import tf
import math
import numpy as np

class IMULogger:
    def __init__(self):
        rospy.init_node('imu_logger')
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.drivePublisher = rospy.Publisher("/cmd_hand_drive",Twist,queue_size=10)
        self.yaw_data = Float32()
        self.array = []
        
        self.gt = []
        self.ft = []
        self.last_time = rospy.Time.now().to_sec()
        
    def imu_callback(self, msg):
        orientation = msg.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        # rospy.loginfo("Yaw difference: %f", yaw - self.yaw_data.data)
        
        current_time = rospy.Time.now().to_sec()
        delta_time = current_time - self.last_time
        self.last_time = current_time
        delta_yaw = (yaw - self.yaw_data.data) / delta_time
        delta_yaw = msg.angular_velocity.z
        self.array.append(delta_yaw)
        self.array = self.array[-20:]
        # rospy.loginfo("Yaw difference (mean): %f", np.mean(self.array))
        self.yaw_data.data = yaw
        
        self.last_time = rospy.Time.now().to_sec()
        self.current_time = rospy.Time.now().to_sec()

    def stop(self):
        self.move_cmd.angular.z = 0.0
        self.drivePublisher.publish(self.move_cmd)
        
    def run(self):
        self.move_cmd = Twist()
        self.move_cmd.angular.z = 0.0
        self.array = []
        self.drivePublisher.publish(self.move_cmd)
        rospy.loginfo("Sent stop command.")
        rospy.sleep(1)
        
        for i in range(1, 21):
            rotate_speed = i / 20.
            self.move_cmd.angular.z = rotate_speed
            self.array = []
            self.drivePublisher.publish(self.move_cmd)
            rospy.loginfo("Rotating at %f", rotate_speed)
            rospy.sleep(1)
            yaw_diff = np.mean(self.array)
            # print(self.array)
            rospy.loginfo("Yaw difference (mean): %f [%f]", yaw_diff, len(self.array))
            self.gt.append(yaw_diff)
            
            self.move_cmd.angular.z = 0.0
            self.array = []
            self.drivePublisher.publish(self.move_cmd)
            rospy.sleep(0.5)


            self.array = []
            self.move_cmd.angular.z = -rotate_speed
            self.drivePublisher.publish(self.move_cmd)
            rospy.loginfo("Rotating at %f", -rotate_speed)
            rospy.sleep(1)
            yaw_diff = np.mean(self.array)
            # print(self.array)
            rospy.loginfo("Yaw difference (mean): %f [%f]", yaw_diff, len(self.array))
            self.ft.append(yaw_diff)
        
        print(self.gt)
        print(self.ft)
        self.stop()
        
        
        
if __name__ == '__main__':
    try:
        logger = IMULogger()
        logger.run()
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        
