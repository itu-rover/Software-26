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
from geometry_msgs.msg import Point 

aruco_marker_side_length = 0.150   

class ArucoSheesh:

    def __init__(self):
        # Initialize ROS node and publishers/subscribers
        rospy.init_node('Dronelanding', anonymous=True)
        self.pub_ratio = rospy.Publisher('Ratio', String, queue_size=10)
        self.pub = rospy.Publisher('/konum/taha', Twist, queue_size=10)
        self.front_cam_pub = rospy.Publisher("/front_cam", CompressedImage, queue_size=10)
        self.subbfront = rospy.Subscriber('/usb_cam/comp/compressed', Image, callback=self.nefront)
        self.subbright = rospy.Subscriber('/camera_right/image_raw', Image, callback=self.neright)
        self.subbleft = rospy.Subscriber('/camera_left/image_raw', Image, callback=self.neleft)
        self.mtx_front = np.array([[641.022726, 0.0, 315.01726],
                                [0.0, 641.022726, 229.900095],
                                [0.0, 0.0, 1.0]], dtype=np.float32)    #logitech c922
        self.mtx_right = np.array([[528.433756558705, 0.0, 320.5],
                                [0.0, 528.433756558705, 240.5],
                                [0.0, 0.0, 1.0]], dtype=np.float32)   
        self.mtx_left = np.array([[528.433756558705, 0.0, 320.5],
                                [0.0, 528.433756558705, 240.5],
                                [0.0, 0.0, 1.0]], dtype=np.float32) 
        self.dst = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.aruco_listener = tf.TransformListener()
        rospy.Rate(10)
        rospy.loginfo("ArucoSheesh node initialized")

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
        rlist = [False, 0, 0, 0, 0]
        img = self.bridge.imgmsg_to_cv2(img)
        marker_id = 0
        this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
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
                print("a")
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, aruco_marker_side_length, mtx, dst
                )
                print("b")
                if len(img.shape) == 2:
                    # Grayscale image
                    height, width = img.shape
                    channels = 1
                else:
                    # Color image
                    height, width, channels = img.shape
                print("c")
                center_x = width // 2
                center_y = height // 2

                for i, marker_id in enumerate(marker_ids):
                    print("a")
                    marker_corners = corners[i][0]  
                    print("a")
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
                        rlist = [True, marker_id, transform_translation_x, transform_translation_y, transform_translation_z]

            return rlist

        except Exception as e:
            rlist = [False, 0, 0, 0, 0]
            rospy.logerr(f"An error occurred: {e}")
            return rlist
    
    def nefront(self, thing):
        data = self.aruco_tfFinder(self.mtx_front, self.dst, thing)
        if data[0]:
                    rot = self.aruco_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))[1]
                    gamma = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))[2]
                    x_tr = data[4] * math.cos((0+gamma)) + data[2] * -math.sin((0+gamma))
                    y_tr = data[4] * -math.sin((0+gamma)) - data[2] * math.cos((0+gamma))
                    rospy.loginfo(f"Marker ID: {data[1]} \t Front")
                    rospy.loginfo(f"Center of ArUco marker: x={x_tr}, y={y_tr}, z={data[3]}")
                    konum = Twist()
                    konum.linear.x = x_tr #data[4]
                    konum.linear.y = y_tr #-data[2]
                    konum.linear.z = data[3]
                    konum.angular.x = data[1]
                    self.pub.publish(konum)
        thing = self.bridge.imgmsg_to_cv2(thing)
        push=self.bridge.cv2_to_compressed_imgmsg(thing)
        self.front_cam_pub.publish(push)

    def neright(self, thing):
        data = self.aruco_tfFinder(self.mtx_right, self.dst, thing)
        if data[0]:
                    rot = self.aruco_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))[1]
                    gamma = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))[2]
                    x_tr = data[4] * -math.cos((-1.57+gamma)) + data[2] * math.sin((-1.57+gamma))
                    y_tr = data[4] * math.sin((-1.57+gamma)) - data[2] * -math.cos((-1.57+gamma))
                    rospy.loginfo(f"Marker ID: {data[1]} \t Right")
                    rospy.loginfo(f"Center of ArUco marker: x={x_tr}, y={y_tr}, z={-data[3]}")
                    
                    konum = Twist()
                    konum.linear.x = x_tr
                    konum.linear.y = y_tr
                    konum.linear.z = -data[3]
                    konum.angular.x = data[1]
                    self.pub.publish(konum)


    def neleft(self, thing):
        data = self.aruco_tfFinder(self.mtx_front, self.dst, thing)
        if data[0]:
                    rot = self.aruco_listener.lookupTransform('/base_link', '/odom', rospy.Time(0))[1]
                    gamma = tf.transformations.euler_from_quaternion((rot[0], rot[1], rot[2], rot[3]))[2]
                    x_tr = data[4] * -math.cos((1.57+gamma)) + data[2] * math.sin((1.57+gamma))
                    y_tr = data[4] * math.sin((1.57+gamma)) - data[2] * -math.cos((1.57+gamma))
                    rospy.loginfo(f"Marker ID: {data[1]} \t Left")
                    rospy.loginfo(f"Center of ArUco marker: x={x_tr}, y={y_tr}, z={-data[3]}")
                    konum = Twist()
                    konum.linear.x = x_tr
                    konum.linear.y = y_tr
                    konum.linear.z = -data[3]
                    konum.angular.x = data[1]
                    self.pub.publish(konum)



if __name__ == '__main__':
    try:
        ArucoSheesh = ArucoSheesh()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")