#!/usr/bin/env python3

import rospy
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient

class GoalPublisher:
    def __init__(self):
        rospy.init_node('goal_publisher_node', anonymous=True)
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def publish_goal(self, x=0, y=0, z=0, yaw=0, roll=0, pitch=0):
        # Create move_base goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = rospy.get_param('~frame_id', 'odom')
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        rospy.loginfo("Sending goal: x=%f, y=%f, z=%f, yaw=%f", x, y, z, yaw)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        rospy.loginfo("Goal reached: %s", result)

if __name__ == '__main__':
    try:
        goal_publisher = GoalPublisher()
        while not rospy.is_shutdown():
            x = float(input("Enter x coordinate: "))
            y = float(input("Enter y coordinate: "))
            # Z, yaw, roll, pitch values can also be taken as input or set as constants
            z = 0
            yaw = 0
            roll = 0
            pitch = 0

            goal_publisher.publish_goal(x, y, z, yaw, roll, pitch)
    except rospy.ROSInterruptException:
        pass

