#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import numpy as np

def camera_publisher():
    # Initialize the ROS node
    rospy.init_node('camera_publisher', anonymous=True)
    
    # Create a publisher for the Image message type
    image_pub = rospy.Publisher('/usb_cam/comp/compressed', CompressedImage, queue_size=10)
    
    # Set the rate at which to publish the images
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a CvBridge object for converting between ROS and OpenCV images
    bridge = CvBridge()
    
    # Open the USB camera (usually /dev/video0)
    cap = cv2.VideoCapture("/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_B71A9FCF-video-index0")
    
    if not cap.isOpened():
        rospy.logerr("Unable to open USB camera")
        return
    else:
        print("Opened Camera")

    while not rospy.is_shutdown():
        # Capture a frame from the camera
        ret, frame = cap.read()
        
        
        if not ret:
            rospy.logerr("Failed to capture image")
            continue
        
        # Convert the OpenCV image (BGR format) to a ROS Image message
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
        # print(type(msg))
        image_pub.publish(msg)
                
        rate.sleep()

    # Release the camera when done
    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
