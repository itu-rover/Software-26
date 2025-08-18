#!/usr/bin/env python3
#
#Author: Emre Demirci  
#This node creates a new frame that we can use as a local frame for ERC23 navigation/science tasks with
#X axis directing forward, Y axis directing left according to the starting position of the rover


import tf
import rospy
import sys
import math
from sensor_msgs.msg import Imu


class FakeOdomTf:
    def __init__(self):
        
        rospy.init_node("fake_odom_tf_publisher")
        self.yaw_file_path = rospy.get_param('~yaw_file_path', default="/home/iturover/iturover23_ws/src/rover23_localization/data/yaw_diff.csv")
        self.liorf_yaw_file_path = rospy.get_param('~liorf_yaw_file_path', default="/home/iturover/iturover23_ws/src/rover23_localization/data/liorf_yaw_diff.csv")
        
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.imu_listener = rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        
        self.init_yaw_diff = 0     
        self.last_liorf_yaw = 0
        self.curr_imu_yaw = 0
        self.yaw_diff = 0  
             
    
        self.rate = rospy.Rate(20)
        self.run()
    
    def imu_callback(self, data):
        orientation = data.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (_, _, self.curr_imu_yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        
    def update_last_odom(self):
        try:
            with open(self.liorf_yaw_file_path, 'rb') as file:
                # Move the file pointer to the end
                file.seek(-2, 2)
                
                # Find the position of the last newline character
                while file.read(1) != b'\n':
                    file.seek(-2, 1)
                
                # Read the last line 
                last_liorf_yaw_diff = file.readline().decode().strip()
                return float(last_liorf_yaw_diff)
            
        except Exception as e:
            rospy.logfatal("An error occurred while reading old translations' file:", e)
            sys.exit()
            
    def update_yaw_diff(self):
        try:
            with open(self.yaw_file_path, 'rb') as file:
                # Move the file pointer to the end
                file.seek(-2, 2)
                
                # Find the position of the last newline character
                while file.read(1) != b'\n':
                    file.seek(-2, 1)
                
                # Read the last line 
                yaw = file.readline().decode().strip()
                return float(yaw)
            
        except Exception as e:
            rospy.logfatal("An error occurred while reading yaw diff file:", e)
            sys.exit()
            
    def run(self):
        self.last_liorf_yaw_diff = self.update_last_odom()
        self.pub_translation = (0., 0., 0.) 
        self.init_yaw_diff = self.update_yaw_diff()    
        self.pub_yaw = self.last_liorf_yaw_diff + self.init_yaw_diff
        self.pub_quaternion = tf.transformations.quaternion_from_euler(0, 0, self.pub_yaw) 
        
        while not rospy.is_shutdown():
            try:
                # Listen current liorf odometry
                (_, self.sub_quaternion) = self.tf_listener.lookupTransform(
                                                            "odom",
                                                            "lidar_link", 
                                                            rospy.Time(0)
                                                            )

                # Calculate yaw drift between imu and liorf odometry
                _, _, curr_liorf_yaw = tf.transformations.euler_from_quaternion(self.sub_quaternion)
                self.liorf_yaw_diff = self.curr_imu_yaw - curr_liorf_yaw + self.last_liorf_yaw_diff
                
                # Set up yaw according to the starting yaw and remove drift 
                self.yaw_diff = self.init_yaw_diff + self.liorf_yaw_diff
                rospy.loginfo(f"yaw diff(degrees): {self.yaw_diff / math.pi * 180}") 
                        
                with open(self.liorf_yaw_file_path, "a") as file:
                    # şu anki liorf yaw diffi dosyaya yaz
                    file.write("\n" + f"{self.liorf_yaw_diff}" )
                rospy.loginfo(f"yaw_diff: {self.liorf_yaw_diff}")      
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.rate.sleep()
                pass
                                

            self.tf_broadcaster.sendTransform(
                self.pub_translation,
                self.pub_quaternion,
                rospy.Time.now(),
                "fake_odom",
                "odom"
            )
            self.rate.sleep()


if __name__ == '__main__':
    try:
        FakeOdomTf()
    except rospy.ROSInterruptException:
        pass


