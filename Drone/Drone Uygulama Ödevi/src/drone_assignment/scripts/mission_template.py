#!/usr/bin/env python3
"""ITU Rover Drone - uygulamalı ödev başlangıç dosyası.

Görev:
1. Drone'u hedef irtifaya çıkar.
2. +X yönünde ilerleyerek alanı tara.
3. Kırmızı topu tespit ettiğinde bir kez ROS_INFO mesajı yazdır.
4. ArUco ID 0 bulunduğunda işaretçiyi görüntü merkezine al.
5. İşaretçi merkezdeyken kontrollü biçimde alçal ve inişi tamamla.
"""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image


class MissionNode:
    TAKEOFF = "TAKEOFF"
    SEARCH = "SEARCH"
    ALIGN_AND_LAND = "ALIGN_AND_LAND"
    DONE = "DONE"

    def __init__(self):
        rospy.init_node("drone_assignment_mission")

        self.bridge = CvBridge()
        self.image = None
        self.position = None
        self.state = self.TAKEOFF
        self.red_reported = False

        self.takeoff_altitude = rospy.get_param("~takeoff_altitude", rospy.get_param("takeoff_altitude", 2.5))
        self.search_speed = rospy.get_param("~search_speed", rospy.get_param("search_speed", 0.40))
        self.search_stop_x = rospy.get_param("~search_stop_x", rospy.get_param("search_stop_x", 7.0))
        self.altitude_kp = rospy.get_param("~altitude_kp", rospy.get_param("altitude_kp", 0.8))
        self.max_vertical_speed = rospy.get_param("~max_vertical_speed", rospy.get_param("max_vertical_speed", 0.7))
        self.pixel_kp = rospy.get_param("~pixel_kp", rospy.get_param("pixel_kp", 0.0025))
        self.max_horizontal_speed = rospy.get_param("~max_horizontal_speed", rospy.get_param("max_horizontal_speed", 0.45))
        self.center_tolerance_px = rospy.get_param("~center_tolerance_px", rospy.get_param("center_tolerance_px", 28))
        self.landing_speed = rospy.get_param("~landing_speed", rospy.get_param("landing_speed", 0.22))
        self.landing_height = rospy.get_param("~landing_height", rospy.get_param("landing_height", 0.32))
        self.red_min_area = rospy.get_param("~red_min_area", rospy.get_param("red_min_area", 250))
        self.aruco_id = rospy.get_param("~aruco_id", rospy.get_param("aruco_id", 0))
        self.pixel_y_to_body_x_sign = rospy.get_param(
            "~pixel_y_to_body_x_sign", rospy.get_param("pixel_y_to_body_x_sign", -1.0)
        )
        self.pixel_x_to_body_y_sign = rospy.get_param(
            "~pixel_x_to_body_y_sign", rospy.get_param("pixel_x_to_body_y_sign", -1.0)
        )

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/bottom_cam/camera/image", Image, self.image_callback, queue_size=1)
        rospy.Subscriber("/ground_truth/state", Odometry, self.odom_callback, queue_size=1)

        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, "DetectorParameters_create"):
            self.aruco_parameters = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_parameters = cv2.aruco.DetectorParameters()

        self.aruco_detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.aruco_detector = cv2.aruco.ArucoDetector(
                self.aruco_dictionary, self.aruco_parameters
            )

        rospy.on_shutdown(self.stop)
        rospy.loginfo("Ödev düğümü hazır. Motorları /enable_motors servisiyle çalıştırınız.")

    def image_callback(self, message):
        try:
            self.image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logerr_throttle(2.0, "Görüntü dönüştürülemedi: %s", error)

    def odom_callback(self, message):
        self.position = message.pose.pose.position

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def altitude_command(self, target_altitude):
        error = target_altitude - self.position.z
        return self.clamp(
            self.altitude_kp * error,
            -self.max_vertical_speed,
            self.max_vertical_speed,
        )

    def detect_red_ball(self, frame):
        """Kırmızı top merkezini ve alanını döndürün.

        Beklenen çıktı:
            Top yoksa: (None, 0.0)
            Top varsa: ((center_x, center_y), contour_area)

        İpucu:
        - BGR -> HSV dönüşümü yapın.
        - Kırmızı renk HSV uzayının iki ucunda bulunduğu için iki maske kullanın.
        - Maskeleri birleştirip en büyük konturu seçin.
        - Alan red_min_area değerinden küçükse gürültü kabul edin.
        """
        # TODO 1: Kırmızı top tespitini yazınız.
        return None, 0.0

    def detect_aruco(self, frame):
        """İstenen ArUco işaretçisinin merkezini döndürün.

        Beklenen çıktı:
            İşaretçi yoksa: None
            İşaretçi varsa: (center_x, center_y)

        İpucu:
        - Yeni OpenCV sürümlerinde self.aruco_detector.detectMarkers(gray)
        - ROS Noetic ile gelen eski sürümlerde cv2.aruco.detectMarkers(...)
        """
        # TODO 2: ArUco ID kontrolünü ve köşe ortalamasını yazınız.
        return None

    def align_and_land_command(self, marker_center, image_shape):
        """Piksel hatasından yatay hız ve iniş komutu üretin."""
        command = Twist()
        image_height, image_width = image_shape[:2]
        image_center_x = image_width / 2.0
        image_center_y = image_height / 2.0

        error_x = marker_center[0] - image_center_x
        error_y = marker_center[1] - image_center_y

        # TODO 3:
        # - error_y değerini linear.x hızına,
        # - error_x değerini linear.y hızına dönüştürün.
        # - Hızları max_horizontal_speed ile sınırlandırın.
        # - Her iki hata center_tolerance_px içindeyse linear.z = -landing_speed yapın.
        # - Merkezde değilse irtifayı koruyacak linear.z komutu verin.

        return command

    def step(self):
        command = Twist()

        if self.image is None or self.position is None:
            return command

        red_center, red_area = self.detect_red_ball(self.image)
        marker_center = self.detect_aruco(self.image)

        if red_center is not None and not self.red_reported:
            self.red_reported = True
            rospy.loginfo("Kırmızı top tespit edildi. Alan: %.0f piksel", red_area)

        if self.state == self.TAKEOFF:
            command.linear.z = self.altitude_command(self.takeoff_altitude)
            if abs(self.position.z - self.takeoff_altitude) < 0.15:
                self.state = self.SEARCH
                rospy.loginfo("Hedef irtifaya ulaşıldı; tarama başladı.")

        elif self.state == self.SEARCH:
            command.linear.x = self.search_speed
            command.linear.z = self.altitude_command(self.takeoff_altitude)

            if marker_center is not None:
                self.state = self.ALIGN_AND_LAND
                rospy.loginfo("ArUco bulundu; hizalama ve iniş başladı.")
            elif self.position.x > self.search_stop_x:
                command.linear.x = 0.0
                rospy.logwarn_throttle(2.0, "ArUco bulunamadı; tarama sınırına ulaşıldı.")

        elif self.state == self.ALIGN_AND_LAND:
            if marker_center is None:
                command.linear.z = self.altitude_command(self.takeoff_altitude)
                rospy.logwarn_throttle(1.0, "ArUco geçici olarak kaybedildi.")
            else:
                command = self.align_and_land_command(marker_center, self.image.shape)

            if self.position.z <= self.landing_height:
                self.state = self.DONE
                command = Twist()
                rospy.loginfo("İniş tamamlandı.")

        return command

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            command = self.step()
            self.cmd_pub.publish(command)
            rate.sleep()

    def stop(self):
        self.cmd_pub.publish(Twist())


if __name__ == "__main__":
    try:
        MissionNode().run()
    except rospy.ROSInterruptException:
        pass
