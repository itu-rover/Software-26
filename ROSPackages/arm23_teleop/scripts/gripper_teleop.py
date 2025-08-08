#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# @authors: yemreakar, buorq3io
# @date: 2023-09-06

import rospy
import serial
import threading
from queue import Queue
from binascii import unhexlify

from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


class GripperTeleop:

    def __init__(self):
        rospy.init_node(node_name := "naim_gripper_teleop")
        rospy.loginfo(f"NODE {node_name} IS STARTED!")

        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.02)
        self.read_check, self.write_check = False, False
        self.button, self.command, self.volt = 0, 0, 0

        self.joy_queue = Queue()
        joy = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.voltage_pub = rospy.Publisher("voltage_value", Float32, queue_size=10)

    def joy_callback(self, data):
        self.joy_queue.put(data)
        rospy.loginfo(f"BUTTON: {self.button}\n"
                      f"COMMAND: {self.command}\n"
                      f"VOLTAGE: {self.volt}\n")

    def main_write(self):
        laser, magnet = 0, 0
        last_laser, last_magnet = 0, 0
        while not rospy.is_shutdown():

            if self.joy_queue.empty():
                continue

            data = self.joy_queue.get()

            if data.axes[-2] == 1 and last_magnet != 1:
                magnet = 1 if magnet == 0 else 0
                rospy.logerr(f"MAGNET {'ON' if magnet == 1 else 'OFF'}!")

            if data.axes[-1] == 1 and last_laser != 1:
                laser = 1 if laser == 0 else 0
                rospy.logerr(f"LASER {'ON' if laser == 1 else 'OFF'}!")

            value = str(int(max(600, min(750 + (data.axes[4] * data.buttons[6]) * 750, 900))))
            grip_msg_to_send = F"S{value}{magnet}{laser}F".encode()

            rospy.logwarn(grip_msg_to_send)
            self.ser.write(grip_msg_to_send)

            last_laser, last_magnet = data.axes[-1], data.axes[-2]

        self.write_check = True

    def main_read(self):
        end_marker = b"Y"
        start_marker = b"X"
        package_size = 17

        first_data = self.ser.read(package_size * 2)
        self.ser.read(first_data.find(start_marker))

        while not rospy.is_shutdown():
            data = self.ser.read(package_size)
            if len(data) != package_size:
                continue

            if data[0:1] == start_marker and data[-1:] == end_marker:
                command_hex = int.from_bytes(unhexlify(data[1:5].decode()), byteorder="big")
                voltage_val = int.from_bytes(unhexlify(data[5:13].decode()), byteorder="big") / 100
                other_vals = data[13:-1].decode()

                self.button, self.command, self.volt = other_vals[2], command_hex, voltage_val
                self.voltage_pub.publish(voltage_val)

            else:
                rospy.logerr("Something went wrong while reading voltage, "
                             "can't publish voltage value.")

                temp_data = self.ser.read(package_size * 2)
                self.ser.read(temp_data.find(start_marker))

        self.read_check = True

    def main(self):
        threads = [threading.Thread(target=self.main_read),
                   threading.Thread(target=self.main_write)]

        for thread in threads:
            thread.start()

        for thread in threads:
            thread.join()

        if self.read_check is True and self.write_check is True:
            self.ser.close()
            rospy.logwarn("SERIAL CONNECTION IS CLOSED!")


if __name__ == "__main__":
    GripperTeleop().main()