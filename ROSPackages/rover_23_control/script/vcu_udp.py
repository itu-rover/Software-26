#!/usr/bin/env python3

# @authors alpogant & berkay & chatgpt
# Date: 9 May 2024 - 14 May 2024

#yaw angle and x velocity are sending to embedded system via serial
#wheel angular velocities are receiving from embedded system via serial
#indicator commands are sending to embedded system via serial

import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String, Float32
import serial
import struct


class Murtaza:
    def __init__(self):
        rospy.init_node("vcu_udp_node", anonymous=False)

        self.angular_z = 0
        self.linear_x = 0
        self.angular_dir = 0
        self.linear_dir = 0

        self.status = "TELEOP"
        self.led_color = "100"
        self.toggle = 0
        # self.serial = serial.Serial("/dev/ttyACM0", 115200, timeout=0.02)
        self.send_msg = ""
        self.recv_msg = ""
        self.wheel_speeds = [0,0,0,0]

        self.rate = rospy.Rate(10)
        self.init()
        self.run()

    def init(self):
        rospy.loginfo("VCU communucation is established")
        rospy.Subscriber("/drive_system/twist", Twist, self.twist_cb)
        rospy.Subscriber("/drive_system/status", String, self.status_cb)
        rospy.Subscriber("/manipulator/current_mode", String, self.manipulator_status_cb)     
        self.rpm_publisher = rospy.Publisher("/drive_system/wheel_angular_velocities", Float64MultiArray, queue_size=10)
        self.weight_publisher = rospy.Publisher("weight_data", Float32, queue_size=10)
        self.vcu_publisher = rospy.Publisher("/drive_system/vcu_data", String, queue_size=10)


    def run(self):
        rospy.loginfo("OPERATION Started")
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.status == "SUCCESS":
                if self.toggle == 0:
                    self.led_color = "010"
                    self.toggle = 1
                else:
                    self.led_color = "000"
                    self.toggle = 0

            if self.angular_z is not None:
                self.send_msg = ""  # Clear send_msg before appending new message
                # self.send_msg += "0009"  # Start of message
                self.send_msg += str(self.angular_dir) # Yaw direction (0 or 1)
                self.send_msg += str(abs(int(float("{:.2f}".format(self.angular_z)) * 100))).zfill(3) # Yaw
                self.send_msg += str(self.linear_dir) # X direction (0 or 1)
                self.send_msg += str(abs(int(float("{:.3f}".format(self.linear_x)) * 1000))).zfill(4) # X
                self.send_msg += self.led_color  # bgr format 3 digits e.g. "010" -> green

                data = String()
                data.data = self.send_msg

                self.vcu_publisher.publish(data)

                # self.get_feedback()

    def get_feedback(self):
        self.recv_msg = self.serial.readline().decode().strip()

        if self.recv_msg.startswith("S") and self.recv_msg.endswith("X"):
            # Wheels angular velocities (rpm)
            self.wheel_speeds[0] = struct.unpack(">i", bytes.fromhex(self.recv_msg[1:9]))[0] #RIGHT FRONT
            self.wheel_speeds[1] = struct.unpack(">i", bytes.fromhex(self.recv_msg[9:17]))[0] #RIGHT REAR
            self.wheel_speeds[2] = struct.unpack(">i", bytes.fromhex(self.recv_msg[17:25]))[0] #LEFT FRONT
            self.wheel_speeds[3] = struct.unpack(">i", bytes.fromhex(self.recv_msg[25:33]))[0] #LEFT REAR
            self.tarti = struct.unpack(">i", bytes.fromhex(self.recv_msg[33:41]))[0] / 1000.0

            self.weight_publisher.publish(self.tarti)
            self.pubm = Float64MultiArray()
            self.pubm.data = self.wheel_speeds
            self.rpm_publisher.publish(self.pubm)     

    def twist_cb(self,data):
        self.linear_x = data.linear.x
        self.angular_z = data.angular.z

        if self.angular_z <= 0:
            self.angular_dir = 0
        else:
            self.angular_dir = 1
        if self.linear_x <= 0:
            self.linear_dir = 0
        else:
            self.linear_dir =1

    def status_cb(self,data):
        self.status = data.data

        if self.status == "TELEOP":
            self.led_color = "100"
        elif self.status == "AUTO":
            self.led_color = "001"
        elif self.status == "SUCCESS":
            self.led_color = "010"
        elif self.status == "YELLOW":
            self.led_color == "011"
        elif self.status == "OFF":
            self.led_color == "000"
        else:
            self.led_color = "4"

    def manipulator_status_cb(self,data):
        self.status = data.data

        if self.status == "JOINT":
            self.led_color = "6"
        elif self.status == "IDLE":
            self.led_color = "4"
        elif self.status == "POSE":
            self.led_color = "1"



if __name__ == "__main__":
    try:
        Murtaza()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")
