#!/usr/bin/env python3

# @authors alpogant & berkay & chatgpt
# Date: 9 May 2024 - 14 May 2024

#yaw angle and x velocity are sending to embedded system via serial
#wheel angular velocities are receiving from embedded system via serial
#TODO watchdog timer resetting

import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import serial
from binascii import unhexlify
import struct


class Murtaza:
    def __init__(self):
        rospy.init_node("vcu_serial_node", anonymous=False)

        self.angular_z = None
        self.linear_x = None
        self.angular_dir = None
        self.linear_dir = None

        self.serial = serial.Serial("/dev/yedek_yurur1", 115200, timeout=0.02)
        self.send_msg = ""
        self.recv_msg = ""
        self.wheel_speeds = [0,0,0,0]

        self.rate = rospy.Rate(10)
        self.init()
        self.run()

    def init(self):
        rospy.loginfo("VCU communucation is established")
        rospy.Subscriber("/drive_system/twist", Twist, self.twist_cb)
        self.rpm_publisher = rospy.Publisher("/drive_system/wheel_angular_velocities", Float64MultiArray, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            # rospy.loginfo("OPERATION MODE")
            self.rate.sleep()

            if self.angular_z is not None:
                self.send_msg = ""  # Clear send_msg before appending new message
                self.send_msg += "S"
                self.send_msg += str(self.angular_dir)
                self.send_msg += str(abs(int(float("{:.2f}".format(self.angular_z)) * 100))).zfill(3)
                self.send_msg += str(self.linear_dir)
                self.send_msg += str(abs(int(float("{:.3f}".format(self.linear_x)) * 1000))).zfill(4)
                self.send_msg += "F"

            # self.send_msg = ""  # Clear send_msg before appending new message
            # self.send_msg += "S100015000F"
   
                print(self.send_msg)
                self.serial.write(self.send_msg.encode())
                self.get_feedback()


    def get_feedback(self):
        # Reading feedback message
        self.recv_msg = self.serial.readline().decode().strip()
        if self.recv_msg.startswith("S") and self.recv_msg.endswith("X"):
            # Wheels angular velocities (rpm)

            #self.wheel_speeds[0] = int.from_bytes(unhexlify(self.recv_msg[1:9]), byteorder="big")
            #self.wheel_speeds[1] = int.from_bytes(unhexlify(self.recv_msg[9:17]), byteorder="big")
            #self.wheel_speeds[2] = int.from_bytes(unhexlify(self.recv_msg[17:25]), byteorder="big")
            #self.wheel_speeds[3] = int.from_bytes(unhexlify(self.recv_msg[25:33]), byteorder="big")

            self.wheel_speeds[0] = struct.unpack(">i", bytes.fromhex(self.recv_msg[1:9]))[0] #RIGHT FRONT
            self.wheel_speeds[1] = struct.unpack(">i", bytes.fromhex(self.recv_msg[9:17]))[0] #RIGHT REAR
            self.wheel_speeds[2] = struct.unpack(">i", bytes.fromhex(self.recv_msg[17:25]))[0] #LEFT FRONT
            self.wheel_speeds[3] = struct.unpack(">i", bytes.fromhex(self.recv_msg[25:33]))[0] #LEFT REAR
            
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

if __name__ == "__main__":
    try:
        Murtaza()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")
