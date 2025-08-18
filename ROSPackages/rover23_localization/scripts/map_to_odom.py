#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from std_msgs.msg import Float32

class Class():
    def __init__(self):
        rospy.init_node("map_to_odom_node", anonymous=True)
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.angle = 0

        rospy.Subscriber("/angle_offset",Float32, self.angle_offset_callback)
        self.rate = rospy.Rate(10)

        self.run()

    def angle_offset_callback(self, msg:Float32):
        self.angle = msg.data
        static_transform = geometry_msgs.msg.TransformStamped()
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = "map"
        static_transform.child_frame_id = "odom"

        # No translation
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0

        # Rotation: yaw = 90 degrees = pi/2 radians
        quat = tf_conversions.transformations.quaternion_from_euler(0, 0, self.angle)  # roll, pitch, yaw
        static_transform.transform.rotation.x = quat[0]
        static_transform.transform.rotation.y = quat[1]
        static_transform.transform.rotation.z = quat[2]
        static_transform.transform.rotation.w = quat[3]

        # Broadcast once
        self.broadcaster.sendTransform(static_transform)

        rospy.loginfo(f"Published static transform: map → odom ({self.angle*180/3.14}° yaw)")


    def run(self):
        while not rospy.is_shutdown():

            self.rate.sleep()



if __name__ == '__main__':
    Class()
