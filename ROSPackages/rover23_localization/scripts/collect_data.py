#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import MagneticField
import numpy as np

magnetometer_data = []

def mag_callback(msg:MagneticField):
    global magnetometer_data
    mag_x = msg.magnetic_field.x
    mag_y = msg.magnetic_field.y
    magnetometer_data.append((mag_x, mag_y))

def collect_data():
    rospy.init_node('magnetometer_data_collector')
    rospy.Subscriber('/imu/mag', MagneticField, mag_callback)
    rospy.spin()

if __name__ == '__main__':
    collect_data()
    # Save data to file or process it
    np.save('/home/iturover/iturover23_ws/src/rover23_localization/scripts/magnetometer_data.npy', magnetometer_data)