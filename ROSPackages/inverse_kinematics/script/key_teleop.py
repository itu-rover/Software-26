#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
import curses
import getpass

class Teleop():
    def __init__(self) :
        self.pos = [0, 0, 0]
        self.pub = rospy.Publisher("keyboard", Float32MultiArray, queue_size=10)
        self.rate = rospy.Rate(10)
        self.stdscr = curses.initscr()
        #Pressing button without displaying
        curses.noecho()
        #Except input without pressing enter 
        curses.cbreak()
        
    def read_button(self):
        c = self.stdscr.getch()

        if (c == ord("q")):
            self.pos[0] = 1
        if (c == ord("a")):
            self.pos[0] = -1
        if (c == ord("w")):
            self.pos[1] = 1
        if (c == ord("s")):
            self.pos[1] = -1
        if (c == ord("e")):
            self.pos[2] = 1
        if (c == ord("d")):
            self.pos[2] = -1    
    
    def publisher(self):
        msg = Float32MultiArray()
        while not rospy.is_shutdown():
            self.read_button()
            msg.data = self.pos
            self.pos = [0, 0, 0]
            self.pub.publish(msg)
            self.rate.sleep()

def main():
    try:
        rospy.init_node("key_teleop")
        app = Teleop()
        app.publisher()
        curses.endwin()
    except rospy.ROSInterruptException():
        pass

if __name__=="__main__":
    main()