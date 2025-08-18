#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
import curses

def main(stdscr):
    rospy.init_node('keyboard_to_joy_with_gears')
    pub = rospy.Publisher('/joy', Joy, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    # Initialize curses
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.refresh()

    # Gear settings
    gears = [0.5, 1.0, 1.5, 2.0]
    current_gear = 1  # start with gear index 1 (1.0 multiplier)

    stdscr.addstr(0, 0, "Use arrow keys to move. Use 1-4 to change gears. Press q to quit.")
    stdscr.addstr(1, 0, f"Current gear: {gears[current_gear]}")

    while not rospy.is_shutdown():
        key = stdscr.getch()
        joy_msg = Joy()
        joy_msg.axes = [0, 0]
        joy_msg.buttons = [0, 0, 0, 0]

        if key == ord('q'):
            break
        elif key == ord('1'):
            current_gear = 0
        elif key == ord('2'):
            current_gear = 1
        elif key == ord('3'):
            current_gear = 2
        elif key == ord('4'):
            current_gear = 3

        stdscr.addstr(1, 0, f"Current gear: {gears[current_gear]}   ")

        if key == curses.KEY_UP:
            joy_msg.axes[1] = gears[current_gear]
        elif key == curses.KEY_DOWN:
            joy_msg.axes[1] = -gears[current_gear]
        elif key == curses.KEY_LEFT:
            joy_msg.axes[0] = -gears[current_gear]
        elif key == curses.KEY_RIGHT:
            joy_msg.axes[0] = gears[current_gear]

        pub.publish(joy_msg)
        rate.sleep()

if __name__ == '__main__':
    curses.wrapper(main)

