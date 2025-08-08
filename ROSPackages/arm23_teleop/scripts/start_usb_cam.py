#!/usr/bin/env python3

import rospy
import subprocess
import os

def start_usb_cam():
    # Parametreler
    video_device = "/dev/video0"
    camera_info_url = "file:///home/iturover/logitech_c920.yaml"

    # rosrun komutu
    cmd = [
        "rosrun",
        "usb_cam",
        "usb_cam_node",
        "_video_device:={}".format(video_device),
        "_camera_info_url:={}".format(camera_info_url)
    ]

    try:
        rospy.loginfo("USB kamera başlatiliyor...")
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError as e:
        rospy.logerr("Kamera başlatilirken hata oluştu: {}".format(e))
    except OSError as e:
        rospy.logerr("rosrun bulunamadi: {}".format(e))

if __name__ == "__main__":
    rospy.init_node("usb_cam_starter", anonymous=True)
    start_usb_cam()
