#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3, PoseStamped
from tf.transformations import quaternion_from_euler
import tf 
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Int32
from math import pi, sin, cos, atan2, sqrt

class Approach():
    def __init__(self):
        rospy.init_node("aruco_approach")

        rospy.Subscriber("/konum/taha", Twist, self.pos_cb)
        rospy.Subscriber('/ublox/odom', Odometry, self.odom_cb)
        rospy.Subscriber("/locomove_base/is_reached", String, callback=self.done_cb)
        

        self.pos_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=100)
        self.status_pub = rospy.Publisher("/drive_system/status", String, queue_size=10)
        self.switch_pub = rospy.Publisher("/switch", Int32, queue_size=10)

        self.pos = Point(0,0,0)
        self.aruco_listener = tf.TransformListener()
        self.goalpos = Point(0,0,0)
        self.aruco = Point()
        self.toggle = 1
        self.done = 0

        self.rate = rospy.Rate(10)
        self.run()


    def pos_cb(self, msg:Twist):
        self.aruco.x = msg.linear.x #msg.linear.z 
        self.aruco.y = msg.linear.y 

        self.goalpos.x = self.pos.x + self.aruco.x 
        self.goalpos.y = self.pos.y + self.aruco.y 

    def done_cb(self, msg):
        # self.set_goal(self.pos.x,self.pos.y) 
        if (not self.toggle) and (not self.done):  
            rospy.logwarn("REACHED GOAL")
            self.status_pub.publish('SUCCESS')
            self.switch_pub.publish(Int32(0))
            self.done = 1


    def odom_cb(self, msg:Odometry):
        self.pos = msg.pose.pose.position

    def set_goal(self, x, y):
        
        yaw = atan2(y-self.pos.y, x-self.pos.x) 
        goal_quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)

        goal = PoseStamped()
        goal.header.frame_id = 'odom' 
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.x = goal_quaternion[0]
        goal.pose.orientation.y = goal_quaternion[1]
        goal.pose.orientation.z = goal_quaternion[2]
        goal.pose.orientation.w = goal_quaternion[3]

        self.pos_pub.publish(goal)
        self.status_pub.publish('AUTO')
        rospy.loginfo(f"\n\n Going to x = {self.goalpos.x} y = {self.goalpos.y}")


    def run(self):
        while not rospy.is_shutdown():
            if (not (self.aruco.x == 0 and self.aruco.y == 0)) and self.toggle:
                self.switch_pub.publish(Int32(1))
                self.set_goal(self.goalpos.x, self.goalpos.y)
                self.toggle = 0
                    

            self.rate.sleep()



if __name__ == '__main__':
    Approach()
