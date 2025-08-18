#!/usr/bin/env python3
# author: Emre Demirci


import rospy
import serial
from std_msgs.msg import String, Float32


class Led:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.r = 0
        self.g = 0
        self.yrg = "S000000255F\n"

        self.weight = 0
        self.mode = "IDLE"

        rospy.init_node('led_publisher', anonymous=True)
        self.sub = rospy.Subscriber("/drive_system/status", String, self.led_cb)
        self.weight_pub = rospy.Publisher("/weight_data", Float32, queue_size=10)

        self.port = "/dev/ttyACM0"
        self.baudrate = 115200
        self.pub = serial.Serial(self.port,
                                 self.baudrate,
                                 bytesize=serial.EIGHTBITS,
                                 parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE,
                                 timeout=0.03,
                                 )
        self.rate = rospy.Rate(60)
        self.init()

    def sleep_n_times(self, n):
        for i in range(n):
            self.rate.sleep()

    def light_for_seconds(self, y, r, g, n):
        self.yrg = ("S" + y + r + g + "F").encode()
        # rospy.loginfo(self.yrg)
        self.pub.write(self.yrg)

        rospy.sleep(n)

    def init(self):
        end_marker = b"Y"
        start_marker = b"X"
        package_size = 10

        first_data = self.pub.read(package_size * 2)
        self.pub.read(first_data.find(start_marker))

        while not rospy.is_shutdown():
            if self.mode == "IDLE":
                self.light_for_seconds("255", "255", "255", 0.1)
                
            elif self.mode == "TELEOP_F":
                self.light_for_seconds("255", "000", "000", 0.1)

            elif self.mode == "TELEOP_SLOW":
                self.light_for_seconds("255", "000", "000", 0.1)
                
            elif self.mode == "CHOOSE":
                z = int(abs((self.x % 255) - 112) * 2)
                z = "{:03d}".format(z)
                self.r = z
                self.x += 4
                self.light_for_seconds(self.r, self.r, self.r, 0.001)

            elif self.mode == "SUCCESS":
                a = int(abs((self.x % 255) - 112) * 2)
                self.g = "{:03d}".format(a)
                self.x += 4
                self.light_for_seconds("000", "000", self.g, 0.001)
            
            elif self.mode == "AUTO" or self.mode == "TELEOP_R":
                self.light_for_seconds("000", "255", "000", 0.1)
            
            elif self.mode == "EMERGENCY":
                b = int(abs((self.x % 255) - 112) * 2)           
                self.r = "{:03d}".format(b)
                self.x += 4
                self.light_for_seconds("000", self.r, "000", 0.0001)
                
            elif self.mode == "SCIENCE":
                self.light_for_seconds("255", "000", "255", 0.1)

            elif self.mode == "ERC_FINAL_VIDEO":
                self.light_for_seconds("000", "255", "000", 2)
                self.light_for_seconds("255", "000", "000", 2)
                self.light_for_seconds("000", "000", "255", 2)

            data = self.pub.read(package_size)
            if len(data) != package_size:
                continue

            if data[0:1] == start_marker and data[-1:] == end_marker:
                self.weight = int(data[1:-1].decode(), 16) / 1000
                self.weight_pub.publish(data=self.weight)
                rospy.loginfo(self.weight)

            else:
                rospy.logerr("Something went wrong while reading...")
                temp_data = self.pub.read(package_size * 2)
                self.pub.read(temp_data.find(start_marker))

            self.rate.sleep()

    # Setting the yrg rates
    def led_cb(self, mode):
        self.mode = mode.data  


if __name__ == '__main__':
    try:
        Led()
    except rospy.ROSInterruptException:
        pass
