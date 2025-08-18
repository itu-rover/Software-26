#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Twist, Quaternion
import tf
class Odom:
    def __init__(self):
        rospy.init_node("Test_odom")
        rospy.Subscriber("/odometry/gps", Odometry, self.nav_callback)
        rospy.Subscriber("/odometry/filtered", Odometry, self.ekf_callback)
        self.odom_pub = rospy.Publisher("/test/odom/", Odometry, queue_size=10)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.rate = rospy.Rate(10)
        self.odom = Odometry()
        self.pos = Point(0,0,0)
        self.quat = Quaternion(0,0,0,0)
        self.vels = Twist()
        
    def nav_callback(self, msg):
        self.pos = msg.pose.pose.position
        #rospy.loginfo(msg.pose.pose.position)

    def ekf_callback(self, msg):
        self.quat = msg.pose.pose.orientation
        #rospy.logerr(msg.twist)
        self.vels = msg.twist.twist

    def main(self):
        while not rospy.is_shutdown():
            self.odom.header.frame_id = "odom"
            self.odom.child_frame_id = "base_footprint"
            self.odom.header.stamp = rospy.Time.now()
            
            self.odom.pose.pose.position = self.pos
            self.odom.pose.pose.orientation = self.quat
            self.odom.twist.twist = self.vels
            self.odom_broadcaster.sendTransform((self.pos.x, self.pos.y, self.pos.z), 
                                                (self.quat.x, self.quat.y, self.quat.z, self.quat.w),
                                                rospy.Time.now(),
                                                "base_footprint",
                                                "odom")
            #rospy.logwarn(self.odom)
            self.odom_pub.publish(self.odom)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        Odom().main()
    except rospy.ROSInterruptException:
        pass