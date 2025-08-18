#!/usr/bin/env python3
#author: Emre Demirci  

import tf
import rospy
import sys 

class YawDiff:
    def __init__(self):
        rospy.init_node("yaw_diff_node")
        self.tf_listener = tf.TransformListener()
        self.is_received = False
        self.yaw_file_path = rospy.get_param('~yaw_file_path', default="/home/iturover/iturover23_ws/src/rover23_localization/data/yaw_diff.csv")


        self.yaw_diff = 0
        self.rate = rospy.Rate(5)
        self.run()

        
    def run(self):
        while (not rospy.is_shutdown()) and (not self.is_received):
            try:

                (_, self.sub_orientation) = self.tf_listener.lookupTransform(
                                                                            "odom",
                                                                            "lidar_link", 
                                                                            rospy.Time(0)
                                                                            )
                
                _, _, self.yaw_diff = tf.transformations.euler_from_quaternion(
                                                                            (self.sub_orientation[0],
                                                                            self.sub_orientation[1],
                                                                            self.sub_orientation[2],
                                                                            self.sub_orientation[3])
                                                                            )
                rospy.loginfo("Tf from odom to base_link received.")
                rospy.loginfo(f"Yaw offset is: {self.yaw_diff}")
                self.is_received = True

                for _ in range(3):      
                    with open(self.yaw_file_path, "a") as file:
                        file.write(f"{self.yaw_diff}" + "\n")
                rospy.loginfo("New line added successfully.")  
                rospy.sleep(2)
                sys.exit()   

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.rate.sleep()
                pass
            self.rate.sleep()


if __name__ == '__main__':
    try:
        YawDiff()
    except rospy.ROSInterruptException:
        pass

                        