#!/usr/bin/env python3

import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import math
import rospy
from std_msgs.msg import String
import tf
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import multiprocessing
import socket
import struct



buffer_size = 65536
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("192.168.1.5", 5555))
aruco_marker_side_length = 0.150   



class ArucoSheesh:

    def __init__(self):
        # Initialize ROS node and publishers/subscribers
        # self.cap_front = cv2.VideoCapture("/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_B71A9FCF-video-index0")
        # self.cap_right = cv2.VideoCapture("platform-3610000.xhci-usb-0:4.3:1.0-video-index0")
        self.cap_left = cv2.VideoCapture("platform-3610000.xhci-usb-0:4.1:1.0-video-index0")
        self.cap_right = cv2.VideoCapture("/dev/video0")
        self.cap_left = cv2.VideoCapture("/dev/video2")
        # self.mtx_front = np.array([[631.537902, 0.0, 323.959792],
        #                         [0.0, 635.626923, 254.572146],
        #                         [0.0, 0.0, 1.0]], dtype=np.float32)    #logitech c922
        
        self.mtx_right =np.array([[435.103843, 0.0, 335.619927],
                                [0.0, 436.047543, 316.171525],
                                [0.0, 0.0, 1.0]], dtype=np.float32)  #imx
         
        self.mtx_left = np.array([[435.103843, 0.0, 335.619927],
                                [0.0, 436.047543, 316.171525],
                                [0.0, 0.0, 1.0]], dtype=np.float32)  #imx
        
        self.dst_front = np.array([0.126946, -0.187288, 0.002625, -0.005277, 0.0], dtype=np.float32)

        self.dst_right = np.array([-0.324697 , 0.064736, -0.009324, -0.003908, 0.0], dtype=np.float32)

        self.dst_left = np.array([-0.324697 , 0.064736, -0.009324, -0.003908, 0.0], dtype=np.float32)

        
        self.bridge = CvBridge()
        # self.rate = rospy.Rate(10)
        rospy.loginfo("ArucoSheesh node initialized")
        self.run_parallel()

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = min(max(t2, -1.0), 1.0)  # clamp t2 to the range [-1, 1]
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z
    
    
    def aruco_tfFinder(self, mtx:np.array,dst:np.array ,img): #-> tuple[bool, int, float, float, float]:

        marker_id = 0
        this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
        this_aruco_parameters = cv2.aruco.DetectorParameters()
        this_aruco_parameters.adaptiveThreshWinSizeMin = 3
        this_aruco_parameters.adaptiveThreshWinSizeMax = 23
        this_aruco_parameters.adaptiveThreshWinSizeStep = 10
        this_aruco_parameters.adaptiveThreshConstant = 7
        this_aruco_parameters.minMarkerPerimeterRate = 0.03
        this_aruco_parameters.maxMarkerPerimeterRate = 4.0
        this_aruco_parameters.polygonalApproxAccuracyRate = 0.03
        this_aruco_parameters.minCornerDistanceRate = 0.05
        this_aruco_parameters.minDistanceToBorder = 3
        this_aruco_parameters.minMarkerDistanceRate = 0.05
        this_aruco_parameters.errorCorrectionRate = 0.6
        try:
            img_undistorted = cv2.undistort(img, mtx, dst)
            corners, marker_ids, rejected = cv2.aruco.detectMarkers(
                img_undistorted, this_aruco_dictionary, parameters=this_aruco_parameters)
            if marker_ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, aruco_marker_side_length, mtx, dst
                )
                if len(img.shape) == 2:
                    # Grayscale image
                    height, width = img.shape
                    channels = 1
                else:
                    # Color image
                    height, width, channels = img.shape
                center_x = width // 2
                center_y = height // 2

                for i, marker_id in enumerate(marker_ids):
                    marker_corners = corners[i][0]  
                    aruco_center_x = int(np.mean(marker_corners[:, 0]))
                    aruco_center_y = int(np.mean(marker_corners[:, 1]))
                    ratio_x = aruco_center_x / width
                    ratio_y = aruco_center_y / height
                    ratio_y_extended = int(ratio_y * 1000)
                    
                    transform_translation_x = tvecs[i][0][0]
                    transform_translation_y = tvecs[i][0][1]
                    transform_translation_z = tvecs[i][0][2]

                    rotation_matrix = np.eye(4)
                    rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
                    r = R.from_matrix(rotation_matrix[0:3, 0:3])
                    quat = r.as_quat()   
                    
                    transform_rotation_x = quat[0] 
                    transform_rotation_y = quat[1] 
                    transform_rotation_z = quat[2] 
                    transform_rotation_w = quat[3] 

                    roll_x, pitch_y, yaw_z = self.euler_from_quaternion(
                        transform_rotation_x, transform_rotation_y, transform_rotation_z, transform_rotation_w
                    )
                    
                    roll_x = math.degrees(roll_x)
                    pitch_y = math.degrees(pitch_y)
                    yaw_z = math.degrees(yaw_z)
                    if marker_id != 0:
                        return [True, marker_id, transform_translation_x, transform_translation_y, transform_translation_z]
                    else:
                        return [False, 0, 0, 0, 0]
            else:
                return [False, 0, 0, 0, 0]
        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")
    
    def process_camera(self, cap:cv2.VideoCapture, mtx, distortion, angle, dopub):

        rospy.init_node('sheesh', anonymous=True)
        self.pub_ratio = rospy.Publisher('Ratio', String, queue_size=10)
        pub_position = rospy.Publisher('/konum/taha', Twist, queue_size=10)
        self.front_cam_pub = rospy.Publisher("/front_cam", Image, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.aruco_listener = tf.TransformListener()
        
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                data = self.aruco_tfFinder(mtx, distortion, frame)
                if data[0]:
                    rot = self.aruco_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))[1]
                    gamma = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))[2]
                    x_tr = -(data[4] * math.cos(angle+gamma) - data[2] * math.sin(angle+gamma))
                    y_tr = data[4] * math.sin(angle+gamma) + data[2] * math.cos(angle+gamma)
                    rospy.loginfo(f"Marker ID: {data[1]}")
                    rospy.loginfo(f"Center of ArUco marker: x={x_tr}, y={y_tr}, z={data[3]}")
                    konum = Twist()
                    konum.linear.x = x_tr
                    konum.linear.y = y_tr
                    konum.linear.z = data[3]
                    konum.angular.x = data[1]
                    pub_position.publish(konum)
            if dopub:
                push = self.bridge.cv2_to_imgmsg(frame,encoding="bgr8")
                push.header.stamp = rospy.Time.now()
                self.front_cam_pub.publish(push)

    def udp_listener_shit(self):
        rospy.init_node('sheesh', anonymous=True)
        aruco_listener = tf.TransformListener()
        aruco_listener.waitForTransform('/odom', '/base_link', rospy.Time(), rospy.Duration(10.0))
        pub_position = rospy.Publisher('/konum/taha', Twist, queue_size=10)
        while not rospy.is_shutdown():
            data, addr = sock.recvfrom(1024)  # Maks 1024 byte al
            # Gelen veriyi double dizisine çevir
            count = len(data) // 8  # double = 8 byte
            doubles = struct.unpack(f'{count}d', data)
            print(doubles)
            marker_idi = int(doubles[0])
            rot = aruco_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))[1]
            gamma = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))[2]
            print(gamma)
            z_distance = (doubles[3] - 3.75)*0.9
            x_distance = -doubles[1]
            rospy.logwarn(f"yatirmadan once: X {z_distance} Y {x_distance}")
            x_tri = -(z_distance * math.cos(gamma) + x_distance * -math.sin(gamma))
            y_tri = -(z_distance * math.sin(gamma) + x_distance * math.cos(gamma))
            rospy.logwarn(f"before change: x: {x_tri}, y: {y_tri}")

            #angle = math.atan(x_tri/y_tri)
            # print(f"angle is {angle}")
            # y_tri -= 2* math.sin(gamma)
            # x_tri -= 2*math.cos(gamma)

            z_tri = doubles[3]
            konuma = Twist()
            konuma.linear.x = x_tri
            konuma.linear.y = y_tri
            konuma.linear.z = z_tri
            konuma.angular.x = marker_idi
            rospy.logwarn(f"mesafe: {math.sqrt(x_tri**2 + y_tri**2)}")
            rospy.logwarn(f"x: {x_tri}, y: {y_tri}")
            # print(konuma)
            pub_position.publish(konuma)
        sock.close()
        print("socket closed.")


    def run_parallel(self):
        front_process = multiprocessing.Process(target=self.udp_listener_shit, args=())
        #front_process = multiprocessing.Process(target=self.process_camera, args=(self.cap_front, self.mtx_front,self.dst_front ,0, True))
        #right_process = multiprocessing.Process(target=self.process_camera, args=(self.cap_right, self.mtx_right, self.dst_right,-1.57, False))
        #left_process = multiprocessing.Process(target=self.process_camera, args=(self.cap_left, self.mtx_left, self.dst_left ,1.57, False))
        
        front_process.start()
        #right_process.start()
        #left_process.start()
        
        front_process.join()
        #right_process.join()
        #left_process.join()
        


if __name__ == '__main__':
    try:
        ArucoSheesh = ArucoSheesh()
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")
