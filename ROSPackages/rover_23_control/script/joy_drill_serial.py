#!/usr/bin/env python3

# @author: buorq3io
# @date: 2023-09-07

import rospy
import serial
from queue import Queue
from numpy import interp
from sensor_msgs.msg import Joy


class JoySerial:

    X, B, AXIS = 2, 1, 1
    BUTTONS = X, B

    def __init__(self):
        # SET THESE VALUES
        baud_rate = 115200
        port = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0"

        self.serial_conn = serial.Serial(port, baud_rate)
        self.serial_conn.flush()

        rospy.init_node(node_name := "joy_drill_node")
        rospy.loginfo(f"NODE ({node_name}) IS SET!")

        self.joy_queue = Queue()
        self.ALLOW_MULTIPLE_MOTORS = True
        self.joy_sub = rospy.Subscriber("/joy_x", Joy, self.joy_callback)

    @staticmethod
    def transform(num: float) -> bytes:
        if num <= 0:
            map_num = str(round(interp(num, [-1, 0], [1000, 0])))
        else:
            map_num = str(round(interp(num, [0, 1], [1001, 2000])))

        str_to_ret = '0' * max(0, 4 - len(map_num)) + map_num
        return str_to_ret.encode()

    def joy_callback(self, data: Joy):
        self.joy_queue.put(data)

    def mainloop(self):
        base_deg = self.transform(0)

        while not rospy.is_shutdown():
            if self.joy_queue.empty():
                continue

            data: Joy = self.joy_queue.get()
            axis_deg = self.transform(data.axes[self.AXIS])

            velocities = b""
            if self.ALLOW_MULTIPLE_MOTORS:
                for button in self.BUTTONS:
                    velocities += axis_deg if data.buttons[button] == 1 else base_deg

            velocities += velocities[:4]

            data_to_write = b"S" + velocities + b"F"
            self.serial_conn.write(data_to_write)
            rospy.logwarn(data_to_write)

        else:
            self.serial_conn.close()
            rospy.logwarn("SERIAL PORT IS CLOSED!")


if __name__ == '__main__':
    JoySerial().mainloop()
