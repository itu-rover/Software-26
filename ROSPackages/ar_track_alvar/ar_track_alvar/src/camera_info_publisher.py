#!/usr/bin/env python3.8

# @author: ahmetuyuklu
# @date: 2023-04-08

import rospy
from sensor_msgs.msg import CameraInfo


def publish_camera_info_yaml():
    # Create a CameraInfo message
    camera_info_msg = CameraInfo()

    # Set the camera parameters
    camera_info_msg.header.frame_id = "camera_frame"
    camera_info_msg.width = 640
    camera_info_msg.height = 480
    camera_info_msg.distortion_model = "plumb_bob"
    camera_info_msg.D = [0.126946, -0.187288, 0.002625, -0.005277, 0.0]
    camera_info_msg.K = [631.537902, 0.0, 323.959792, 0.0, 635.626923, 254.572146, 0.0, 0.0, 1.0]
    camera_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    camera_info_msg.P = [645.225159, 0.0, 320.621683, 0.0, 0.0, 650.739502, 255.199355, 0.0, 0.0, 0.0, 1.0, 0.0]

    # Publish the CameraInfo message on the appropriate topic
    camera_info_pub = rospy.Publisher('/camera_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        camera_info_pub.publish(camera_info_msg)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('camera_info_publisher')
    publish_camera_info_yaml()
