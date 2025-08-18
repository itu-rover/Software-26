#!/usr/bin/env python3
#
#Author: Emre Demirci  
#This node creates a new frame that we can use as a global frame for ERC23 navigation/science tasks

import tf
import rospy
import sys
import math
from sensor_msgs.msg import Imu


x_diff = []     
y_diff = []

class FakeOdomTf:
    def __init__(self):
        
        rospy.init_node("fake_map_tf_publisher")
        
        self.odom_file_path = rospy.get_param('~odom_file_path', default="/home/iturover/iturover23_ws/src/rover23_localization/data/odom.csv")

        self.max_velocity = 100
        
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        self.init_x, self.init_y = 0. , 0.

        self.last_liorf_yaw_diff = 0
        
        self.last_x = 0
        self.last_y = 0
        self.curr_x = 0
        self.curr_y = 0   
    
        self.rate = rospy.Rate(20)
        self.run()
        
    def update_last_odom(self):
        try:
            with open(self.odom_file_path, 'rb') as file:
                # Move the file pointer to the end
                file.seek(-2, 2)
                
                # Find the position of the last newline character
                while file.read(1) != b'\n':
                    file.seek(-2, 1)
                
                # Read the last line 
                x, y = file.readline().decode().strip().split()
                return float(x), float(y)
            
        except Exception as e:
            rospy.logfatal("An error occurred while reading old translations' file:", e)
            sys.exit()

            
    def run(self):
        self.init_x, self.init_y = self.update_last_odom()
        self.pub_translation = (-self.init_x, -self.init_y, 0.)    
        
        self.last_time = rospy.Time.now().to_sec()
        self.curr_time = rospy.Time.now().to_sec()
        
        self.last_x = self.init_x
        self.last_y = self.init_y
        self.curr_x = self.init_x
        self.curr_y = self.init_y
        
        while not rospy.is_shutdown():
            try:
                # Listen transform between fake_odom and lidar_link which is calculated from liorf odometry
                (self.sub_translation, _) = self.tf_listener.lookupTransform(
                                                            "fake_odom",
                                                            "lidar_link", 
                                                            rospy.Time(0)
                                                            )
                
                self.pub_quaternion = tf.transformations.quaternion_from_euler(0, 0, 0) 
                self.tf_broadcaster.sendTransform(
                    self.pub_translation,
                    self.pub_quaternion,
                    rospy.Time.now(),
                    "fake_map",
                    "fake_odom"
                                )
                self.curr_time = rospy.Time.now().to_sec()
                self.delta_t = self.curr_time - self.last_time
                if self.delta_t < 0.05:
                    pass
                else:
                    self.last_time = self.curr_time
                    
                    # 
                    self.curr_liorf_x = float(self.sub_translation[0]) + self.init_x
                    self.curr_liorf_y = float(self.sub_translation[1]) + self.init_y
                    
                    self.delta_x = self.curr_liorf_x - self.last_x
                    self.delta_y = self.curr_liorf_y - self.last_y
                    
                    self.last_x = self.curr_liorf_x
                    self.last_y = self.curr_liorf_y

                    
                    self.delta_total = ((self.delta_x) ** 2 + (self.delta_y) ** 2) ** 0.5
                    self.dtotaldt = self.delta_total / self.delta_t
                    
                    if self.dtotaldt > self.max_velocity:
                        rospy.loginfo("Liorf - large velocity" * 30) 
                        x_diff.append(self.delta_x)
                        y_diff.append(self.delta_y)
                    else:
                        # if not large velocity, updare curr x y
                        self.curr_x += self.delta_x
                        self.curr_y += self.delta_y
                    
            
                            
                    with open(self.odom_file_path, "a") as file:
                        # Save current x y
                        file.write("\n" + f"{self.curr_x} {self.curr_y}" )
                    rospy.loginfo(f"x: {self.curr_x}, y: {self.curr_y}")   
                    rospy.loginfo(f"x diff: {sum(x_diff)}")
                    rospy.loginfo(f"y diff: {sum(y_diff)}") 
                
                # Publish current x y to /fake/odometry topic
                # Yapılacak ############################################################################################
                  
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.rate.sleep()
                pass

            self.rate.sleep()


if __name__ == '__main__':
    try:
        FakeOdomTf()
    except rospy.ROSInterruptException:
        pass

        


