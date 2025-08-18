#!/usr/bin/env python3
import rospy
from pynput import keyboard
from std_msgs.msg import Int32, String
import os


class Switcher():
    def __init__(self):
        rospy.init_node('switcher')

        
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher("/switch", Int32, queue_size=10)
        self.status_pub = rospy.Publisher("/drive_system/status", String, queue_size=10)
        self.num = Int32()
        self.alt = 0
        
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start() 
        self.listener.join() 
        os.environ['DISPLAY'] = '0'
        
        self.run()

    def on_press(self, key):
        if key == keyboard.Key.f9:
            return False  # stop listener
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # other keys
        
        if k == 'pause':
            rospy.logwarn(k)
            self.num.data = 1
            return False
        elif k == 'alt':
            rospy.logwarn(k)
            self.alt = 1
        elif (k == '1') and self.alt:
            rospy.loginfo("Switching to AUTO")
            self.status_pub.publish('AUTO')
            self.num.data = 0
        elif (k == '2') and self.alt:
            rospy.loginfo("Switching to SUCCESS")
            self.status_pub.publish('SUCCESS')
            self.num.data = 0
        elif (k == '3') and self.alt:
            rospy.loginfo("Switching to TELEOP")
            self.status_pub.publish('TELEOP')
            self.num.data = 0
        elif (k == '4') and self.alt:
            rospy.loginfo("Switching to BLUE")
            self.status_pub.publish('BLUE')
            self.num.data = 0
        elif (k == '9') and self.alt:
            rospy.loginfo("Switching to OFF")
            self.status_pub.publish('OFF')
            self.num.data = 0
        else:
            self.num.data = 0
            self.alt = 0

    def on_release(self, key):
        if key == keyboard.Key.f9:
            return False  # stop listener
        try:
            k = key.char  # single-char keys
        except:
            k = key.name  # other keys
        
        if k == 'alt':
            rospy.logerr("Released alt")
            self.alt = 0

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.num)

            self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
            self.listener.start() 
            self.listener.join() 

            self.rate.sleep()


if __name__ == '__main__':
    Switcher()

