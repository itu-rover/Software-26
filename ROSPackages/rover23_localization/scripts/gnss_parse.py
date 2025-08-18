#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Float64MultiArray, String

class Read_Serial():
    def __init__(self):
        rospy.init_node("gnss_parse")

        self.rate = rospy.Rate(40)
        self.serial = serial.Serial("/dev/yedek_stm0", 115200, timeout=0.5)

        self.pub = rospy.Publisher("/gnss_data", Float64MultiArray, queue_size=10)
        self.str = [b'L',b'L',b'L',b'L',b'L',b'L',b'L',b'L',b'L',b'L']
        self.bool = 0

        self.float = Float64MultiArray()
        self.float.data = [0,0,0,0,0]

        self.run()

    def read_serial(self):
        # Reading feedback message
        self.recv_msg = self.serial.read()      #readline().decode().strip()
        if self.recv_msg == b'S':
            self.str = self.serial.read(15)
            

        # self.str.append(self.recv_msg)

        # self.str = bytearray(self.recv_msg) #[bytes([b]) for b in self.recv_msg]

        # if self.bool:
        #     self.val.append(self.recv_msg)



    def run(self):
        while not rospy.is_shutdown():
            self.read_serial()
            self.bool = 0
            for byte in self.str:
                if byte == 83:
                    self.bool = 1
                elif byte == 88:
                    self.bool = 0
                    # print(self.str)
                    # rospy.logwarn(len(self.str)-2)

                    if (len(self.str)-1) >= 13:
                        self.float.data[0] = ((self.str[0] * 2**24) + (self.str[1] * 2**16) + (self.str[2] * 2**8) + (self.str[3])) * (1e-7)
                        self.float.data[1] = (((self.str[4] * 2**24) + (self.str[5] * 2**16) + (self.str[6] * 2**8) + (self.str[7]))) * (1e-7)  
                        self.float.data[2] = -(self.str[8] * 2**8 + self.str[9]) * (1e-4)
                        self.float.data[3] = (self.str[11] * 2**8 + self.str[10]) * (1e-4)
                        self.float.data[4] = (self.str[13] * 2**8 + self.str[12]) * (1e-2)
                        rospy.loginfo(f"\n\n lat = {self.float.data[0]:.10f} \n long = {self.float.data[1]:.10f} \n yaw° = {self.float.data[2]*180/3.14} \n sog = {self.float.data[4]} \n cog° = {(self.float.data[3])*180/3.14} \n")
                        self.pub.publish(self.float)
                
                    self.float.data = [0,0,0,0,0]
                    self.str = []


            self.rate.sleep()


if __name__ == "__main__":
    try:
        Read_Serial()
    except:
        rospy.logwarn("Node Terminated")



