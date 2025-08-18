#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

joystick_array = Float64MultiArray()

def joyCallback(joy_data):
    joystick_array.data = [joy_data.axes[1], joy_data.axes[0], joy_data.axes[4], joy_data.axes[3], joy_data.buttons[0], joy_data.buttons[1], joy_data.buttons[3], joy_data.buttons[2],   joy_data.buttons[5], joy_data.buttons[4], joy_data.axes[5], joy_data.axes[2]]

def run():
    rospy.init_node('science_joy', anonymous=True)
    rate=rospy.Rate(10)
    rospy.Subscriber('joy', Joy, joyCallback)
    joystick_pub = rospy.Publisher('science_joystick', Float64MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        joystick_pub.publish(joystick_array)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__': 
    try:
        run()
    except rospy.ROSInterruptException:
        pass
