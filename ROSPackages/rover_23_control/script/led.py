#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
import serial
import threading

class Murtaza:
    def __init__(self):
        rospy.init_node("vcu_serial_node", anonymous=False)

        self.status = "IDLE"
        self.led_color = "5"

        self.serial = serial.Serial("/dev/ttyACM0", 115200, timeout=0.02)
        
        self.lock = threading.Lock()
        self.rate = rospy.Rate(25)

        self.messages = ["S0000000005F", "S0000000006F", "S0000000001F"]
        self.current_message_index = 0

        self.init()
        self.start_sending_messages()
        self.run()

    def init(self):
        rospy.Subscriber("/drive_system/status", String, self.status_cb)

    def status_cb(self, msg):
        self.status = msg.data

    def send_next_message(self):
        with self.lock:
            message = self.messages[self.current_message_index]
            rospy.loginfo(f"Sending message: {message}")
            self.serial.write(message.encode())
            
            # Move to the next message or cycle back to the start
            self.current_message_index = (self.current_message_index + 1) % len(self.messages)
            
        # Schedule the next message to be sent in 5 seconds
        threading.Timer(5.0, self.send_next_message).start()

    def start_sending_messages(self):
        self.send_next_message()

    def run(self):
        rospy.loginfo("OPERATION Started")
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == "__main__":
    try:
        Murtaza()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")

