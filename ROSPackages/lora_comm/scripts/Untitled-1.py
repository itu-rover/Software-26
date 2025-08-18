#!/usr/bin/env python
# Author:   B. Burak Payzun
# Date:     16-05-23

import socket
import rospy
import threading 
import struct
from std_msgs.msg import String, Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
This node receives from ground station:
16 bits X
16 bits Yaw
8 bits GPS target index
8 bits operation mode
8 bits CRC
"""

"""
This node sends to ground station:
16 bits X
16 bits Y
16 bits Yaw
8 bits GPS target index
8 bits current mode
8 bits CRC
"""

GPS_MAP = [
    "NOT DEFINED",  # 0
    "NOT DEFINED",  # 1
    "NOT DEFINED",  # 2
    "NOT DEFINED",  # 3
    "NOT DEFINED",  # 4
    "NOT DEFINED",  # 5
    "NOT DEFINED",  # 6
    "NOT DEFINED",  # 7
    "NOT DEFINED",  # 8
    "NOT DEFINED",  # 9
    "NOT DEFINED",  # 10
    "NOT DEFINED",  # 11
    "NOT DEFINED",  # 12
    "NOT DEFINED",  # 13
    "NOT DEFINED",  # 14
    "NOT DEFINED",  # 15
]

OPMODE_MAP = [
    "NOT DEFINED",  # 0
    "NOT DEFINED",  # 1
    "NOT DEFINED",  # 2
    "NOT DEFINED",  # 3
    "NOT DEFINED",  # 4
    "NOT DEFINED",  # 5
    "NOT DEFINED",  # 6
    "NOT DEFINED",  # 7
    "NOT DEFINED",  # 8
    "NOT DEFINED",  # 9
    "NOT DEFINED",  # 10
    "NOT DEFINED",  # 11
    "NOT DEFINED",  # 12
    "NOT DEFINED",  # 13
    "NOT DEFINED",  # 14
    "NOT DEFINED",  # 15
]

class LoraRover:
    def __init__(self):
        self.HOST = "192.168.1.5"
        self.PORT = 8887

        # General
        self.is_connected = False
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn = None
        self.addr = None

        self.twist_publisher = rospy.Publisher("/lora/rover/received/twist", Twist, queue_size=10)
        self.gps_publisher = rospy.Publisher("/lora/rover/received/gps_target", String, queue_size=10)
        self.mode_publisher = rospy.Publisher("/lora/rover/received/current_mode", String, queue_size=10)
        
        self.odom_sub = rospy.Subscriber("/lora/rover/transmit/odometry", Odometry, self.odometry_callback)
        self.gps_target_sub = rospy.Subscriber("/lora/rover/transmit/gps_target", Int32, self.gps_target_callback)
        self.gop_mode_sub = rospy.Subscriber("/lora/rover/transmit/current_mode", Int32, self.op_mode_callback)

        # Receive
        self.receive_message = {
            'X': 0,
            'yaw': 0,
            'gps_index': 0,
            'current_mode': 0,
            'CRC': 0
        }

        self.receive_str = 'eeBBB'
        self.receive_size = 7

        # Transmit
        self.transmit_message = {
            'X': 0,
            'Y': 0,
            'yaw': 0,
            'gps_index': 0,
            'current_mode': 0,
            'CRC': 0
        }

        self.transmit_str = 'eeeBBB'

    def odometry_callback(self, data):
        self.transmit_message['X'] = data.pose.pose.position.x
        self.transmit_message['Y'] = data.pose.pose.position.y

        q = data.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.transmit_message['yaw'] = yaw

    def gps_target_callback(self, data):
        self.transmit_message['gps_index'] = data.data

    def op_mode_callback(self, data):
        self.transmit_message['current_mode'] = data.data

    def loop_a(self):
        while True:
            self.write()
            self.write_rate.sleep()

    def loop_b(self):
        while True:
            self.read()
            self.read_rate.sleep()

    def run(self):
        rospy.init_node("lora_comm_rover", disable_signals=True)
        self.rate = rospy.Rate(1)
        self.read_rate = rospy.Rate(2)
        self.write_rate = rospy.Rate(2)
        rospy.loginfo("LoRa communication started: Rover")
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.socket.bind((self.HOST, self.PORT))
        except:
            rospy.loginfo("Address already in use")
        rospy.loginfo(f"Host binded on {self.HOST}:{self.PORT}")
        self.socket.listen()
        rospy.loginfo(f"Listening...")

        self.conn, self.addr = self.socket.accept()
        rospy.loginfo(f"Connected by {self.addr[0]}:{self.addr[1]}")


        p1 = threading.Thread(target=self.loop_a)
        p2 = threading.Thread(target=self.loop_b)

        p1.start()
        p2.start()

        rospy.spin()

    def write(self):
        try:
            packed_message = struct.pack(self.transmit_str, 
                                self.transmit_message['X'],
                                self.transmit_message['Y'],
                                self.transmit_message['yaw'],
                                self.transmit_message['gps_index'],
                                self.transmit_message['current_mode'],
                                self.transmit_message['CRC'])
        except:
            return

        self.transmit_message['CRC'] = (self.transmit_message['CRC'] + 1) % 255
        try:                         
            rospy.loginfo(f"Sent message with size: {len(packed_message)}")                                                                                
            self.conn.sendall(packed_message)
        except:
            self.conn, self.addr = self.socket.accept()
            rospy.loginfo(f"Reconnected by {self.addr[0]}:{self.addr[1]}")

    def read(self):
        try:
            self.conn.settimeout(2.0)
            data = self.conn.recv(64)
            err = 0

        except Exception as e:
            rospy.loginfo(f"Timeout in receive. Error: {e}")
            err = err+1
            if err == 5:
                self.socket.close()
                rospy.loginfo("Timeout error 5 times in a row, restarting connection procedure")
                self.socket.bind((self.HOST, self.PORT))
                rospy.loginfo(f"Host binded on {self.HOST}:{self.PORT}")
                self.socket.listen()
                rospy.loginfo(f"Listening...")
                self.conn, self.addr = self.socket.accept()
                rospy.loginfo(f"Reconnected by {self.addr[0]}:{self.addr[1]}")

            return
        if not data:
            return
        
        try:
            if len(data) == self.receive_size:
                msg = struct.unpack(self.receive_str, data)
            else:
                rospy.loginfo(f"Incorrect receive size: {len(data)} Expected: {self.receive_size}")
                return
        except:
            return
            
        self.received_message = {
            'X': msg[0],
            'yaw': msg[1],
            'gps_index': msg[2],
            'current_mode': msg[3],
            'CRC': msg[4]
        }
        twist = Twist()
        twist.linear.x = msg[0]
        twist.angular.z = msg[1]

        self.twist_publisher.publish(twist)
        self.gps_publisher.publish(String(data=GPS_MAP[msg[2]]))
        self.mode_publisher.publish(String(data=OPMODE_MAP[msg[3]]))

        rospy.loginfo(msg)

    def __del__(self):
        self.socket.close()

if __name__ == "__main__":
    try:
        lora_comm = LoraRover().run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
