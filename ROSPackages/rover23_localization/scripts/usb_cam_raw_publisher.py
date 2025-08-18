#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def camera_publisher():
    # Initialize the ROS node
    rospy.init_node('camera_publisher', anonymous=True)
    
    # Create a publisher for the Image message type
    image_pub = rospy.Publisher('/usb_cam/image_raw/raw', Image, queue_size=10)
    
    # Set the rate at which to publish the images
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a CvBridge object for converting between ROS and OpenCV images
    bridge = CvBridge()
    
    # Open the USB camera (usually /dev/video0)
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        rospy.logerr("Unable to open USB camera")
        return

    while not rospy.is_shutdown():
        # Capture a frame from the camera
        ret, frame = cap.read()
        
        if not ret:
            rospy.logerr("Failed to capture image")
            continue
        
        # Convert the OpenCV image (BGR format) to a ROS Image message
        ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        # Publish the image
        image_pub.publish(ros_image)
        
        # Sleep to maintain the set publishing rate
        rate.sleep()

    # Release the camera when done
    cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
