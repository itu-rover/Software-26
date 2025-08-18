#!/usr/bin/env python3

# @author alpogant
# Date: 9 May 2024

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped, Vector3
from actionlib_msgs.msg import GoalID 
from std_msgs.msg import String, Int32

class TeleopJoy:
    def __init__(self):
        rospy.init_node("joy_to_twist_node")

        self.twist = Twist()
        self.nav = Twist()

        self.turbo_multiplier = 4

        self.rate = rospy.Rate(25)
        self.lb = 0
        self.rb = 0
        self.autonomous = 0
        self.done = 0
        self.joy_enabled = 0
        self.init()
        self.run()

        

    def init(self):
        rospy.loginfo("Joystick teleoperation initialized")
        rospy.Subscriber("/joy", Joy, self.joy_cb)
        rospy.Subscriber("/locomove_base/is_reached", String, callback=self.done_cb)
        rospy.Subscriber("/locomove_base/current_goal", PoseStamped, callback=self.goal_cb)
        rospy.Subscriber('/nav_vel', Twist, self.nav_callback)
        rospy.Subscriber("/switch", Int32, self.switch_cb)
        self.nav_pub = rospy.Publisher("/nav_vel1", Twist, queue_size=10)
        self.goal_abort_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=100)
        self.cmd_pub = rospy.Publisher("/drive_system/twist", Twist, queue_size=10)
        self.status_pub = rospy.Publisher("/drive_system/status", String, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            if self.lb and self.rb:
                self.twist.linear.x = self.linear_axis * (self.turbo_multiplier/2)
                self.twist.angular.z = self.angular_axis * (self.turbo_multiplier/2)
            elif self.lb:
                self.twist.linear.x = self.linear_axis 
                self.twist.angular.z = self.angular_axis 
            elif self.rb:
                self.twist.linear.x = self.linear_axis * self.turbo_multiplier
                self.twist.angular.z = self.angular_axis * self.turbo_multiplier 
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
            
            if (self.autonomous and self.done):
                vel = Twist()
                vel.linear = Vector3(0,0,0)
                vel.angular = Vector3(0,0,0)
                self.nav_pub.publish(vel)
            elif (self.autonomous and (not self.done)):
                self.nav_pub.publish(self.nav)

            if (not self.autonomous):
                rospy.loginfo(f"OPERATION MODE {self.joy_enabled}")
                if self.joy_enabled:
                    self.cmd_pub.publish(self.twist)
            else:
                rospy.logwarn(f"AUTONOMOUS MODE {self.joy_enabled}")


            self.rate.sleep()

    def done_cb(self, msg:String):
        self.done = 1

    def goal_cb(self, msg:PoseStamped):
        self.done = 0

    def nav_callback(self, msg:Twist):
        self.nav = msg
        self.nav.angular.z = self.nav.angular.z * 1.2

    def switch_cb(self, msg:Int32):
        if (not self.autonomous) and (msg.data==1):
            self.autonomous = 1
            self.joy_enabled = 0
        elif (self.autonomous) and (msg.data==0):
            self.autonomous = 0
            self.joy_enabled = 1
        elif msg.data == 2:
            self.joy_enabled = 0


    def joy_cb(self, data:Joy):
        self.lb = data.buttons[4] # L1 button -> normal mode
        self.rb = data.buttons[5] # R1 button -> turbo mode

        self.angular_axis = data.axes[0] # Left analog stick's right and left axes
        self.linear_axis = data.axes[1] # Left analog stick's up and down axes

        if (self.autonomous == 0 and data.buttons[8]):
            self.autonomous = 1
            self.status_pub.publish("AUTO")
        elif data.buttons[7]:   #(self.autonomous == 1 and data.buttons[7]):   
            self.autonomous = 0 
            self.status_pub.publish("TELEOP")
            abort = GoalID()
            self.goal_abort_pub.publish(abort)

        if(data.axes[2] == -1 and not self.joy_enabled):
           self.joy_enabled = 1
        elif (data.axes[5] == -1 and self.joy_enabled):
            self.joy_enabled = 0

if __name__ == "__main__":
    try:
        TeleopJoy()
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")