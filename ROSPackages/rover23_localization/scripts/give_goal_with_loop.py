#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
import argparse
import actionlib
import tf
import math
import csv

class Goal:
    def __init__(self):
        rospy.init_node("goal_node")

        self.sub_translation = (0., 0., 0.)
        self.sub_quaternion = [0., 0., 0., 1]
        self.yaw = 0
        self.tf_listener = tf.TransformListener()


        self.goal_x = 0
        self.goal_y = 0

        self.rate = rospy.Rate(1)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def read_csv_to_list(self, filename):
        data = []
        with open(filename, 'r', newline='') as csvfile:
            csvreader = csv.reader(csvfile)
            for row in csvreader:
                data.append(row)
        return data
    
    def move_base_goal(self):
        while self.yaw == 0:
            try:
                (self.sub_translation, self.sub_quaternion) = self.tf_listener.lookupTransform(
                                                            "odom",
                                                            "fake_map", 
                                                            rospy.Time(0)
                                                            )
                _, _, self.yaw = tf.transformations.euler_from_quaternion(self.sub_quaternion)
                
                self.new_x = math.cos(self.yaw) * self.goal_x - math.sin(self.yaw) * self.goal_y + self.sub_translation[0]
                self.new_y = math.cos(self.yaw) * self.goal_y + math.sin(self.yaw) * self.goal_x + self.sub_translation[1]
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "odom"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = float(self.new_x)
                goal.target_pose.pose.position.y = float(self.new_y)
                goal.target_pose.pose.orientation.w = 1.0

                self.client.send_goal(goal)
                wait = self.client.wait_for_result()
                if not wait:
                    rospy.logerr("Action server not available!")
                    rospy.signal_shutdown("Action server not available!")
                else:
                    return self.client.get_result()
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                self.rate.sleep()
                pass
                    

if __name__ == '__main__':
    try:
        goal = Goal()
        result = goal.move_base_goal()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.logwarn("An error occured.")