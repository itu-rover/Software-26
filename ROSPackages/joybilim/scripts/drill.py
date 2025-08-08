#!/usr/bin/env python3

import rospy
import numpy as np
import serial
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, String
import time


class drill():
    def __init__(self):
        rospy.init_node("joy_driller")
        self.i = 0
        self.serial = serial.Serial("/dev/serial/by-path/platform-3610000.xhci-usb-0:4.2.2:1.0-port0", 115200, timeout=0.02)
        rospy.Subscriber("/joy", Joy, self.callback)
        rospy.Publisher("/drill_message", String, queue_size=10)
        self.message_to_send = ""
        self.rate = rospy.Rate(10)
        self.main()


    def callback(self, data):
        self.i +=1
        if data.axes[5] < 0:
            left_joy = -data.axes[1]
            self.stop_button = data.axes[2]
            right_joy = -data.axes[4]
            self.rtt_value = int(left_joy*499 + 500)
            self.lnr_value = int(right_joy*499 + 500)
            self.message_to_send = "S" + str(self.rtt_value) + str(self.lnr_value) + str(self.i%2) +"F"
            self.serial.write(self.message_to_send.encode())
            if data.buttons[2] == 1:
                self.otonom_drill()
            # elif data.buttons[1] == 1:
            #     self.otonom_karot()
            elif data.buttons[3] == 1:
                self.otonom_out()
                

    def otonom_drill(self):
        start_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            if self.stop_button < 0:
                break
            if elapsed_time < 50:
                self.i += 1
                self.rtt_value = 750
                self.lnr_value = 750
                self.message_to_send = "S" + str(self.rtt_value) + str(self.lnr_value) + str(self.i%2) +"F"
            else:
                return
            
    # def otonom_karot(self):
    #     start_time = time.time()
    #     while True:
    #         current_time = time.time()
    #         elapsed_time = current_time - start_time
    #         if elapsed_time < 20:
    #             self.rtt_value = 500
    #             self.lnr_value = 750
    #             self.message_to_send = "S" + str(self.rtt_value) + str(self.lnr_value) + str(self.i%2) +"F"
    #         else:
    #             return

    def otonom_out(self):
        start_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - start_time
            if self.stop_button < 0:
                break
            if elapsed_time < 50:
                self.i += 1
                self.rtt_value = 250
                self.lnr_value = 250
                self.message_to_send = "S" + str(self.rtt_value) + str(self.lnr_value) + str(self.i%2) +"F"
            else:
                return
            
    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
        


if __name__=="__main__":
    try:
        drill()
        rospy.loginfo("finished")
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")
