#!/usr/bin/env python3

# @author: buorq3io
# @date: 2023-09-14

import os
import cv2
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


class CameraSignalHandler:

    def __init__(self):
        rospy.init_node(node_name := 'iturover_ui_cam_handler')
        rospy.loginfo(f"NODE ({node_name}) IS SET!")

        self.count = 0
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("/take_photo_signal", String, self.main_callback)

    def main_callback(self, callback_data: String):
        message = rospy.wait_for_message(callback_data.data, CompressedImage,
                                         timeout=rospy.Duration(1))

        cv2_img = self.bridge.compressed_imgmsg_to_cv2(message, "bgr8")

        home_directory = os.path.expanduser("~")
        directory = os.path.join(home_directory, "rover_camera_captures")

        if not os.path.isdir(directory):
            os.makedirs(directory)

        img_name = f"camera_image_{self.count}_{int(rospy.Time.now().to_sec())}.jpeg"
        cv2.imwrite(os.path.join(directory, img_name), cv2_img)

        rospy.loginfo(f"SAVED {img_name}!")
        self.count += 1


if __name__ == '__main__':
    CameraSignalHandler()
    rospy.spin()
