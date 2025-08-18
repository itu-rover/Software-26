#!/usr/bin/env python

import rospy 
from std_msgs.msg import Float64MultiArray, String, Int16MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Imu, JointState
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time

def saturate(data):
    if(data > 1 ):
        return 1
    elif data < -1:
        return -1
    else: 
        return data

def map(data):
    return (data) / 100

class SerialNoSteer:
    def __init__(self):
        # MODES OF SERIAL     
        rospy.init_node("Serial_node" ,anonymous=False)   
        self.is_idle = False
        self.choose_mode = False
        self.is_drive = False
        self.autonomous = False
        self.goal_reached = False
        self.emergency  = False
        self.rk = False
        self.science = False 

        self.time = rospy.Time.now().secs
        
        # örümcek rover megataskı için gerekli imu değişkenleri
        self.init_pitch = 0  # Initial pitch angle of the rover
        self.last_pitch = 0
        self.curr_pitch = 0
        self.pitch_change = 0
        self.pitch_counter = 0
        self.flag = 0 
        
        # Constants
        self.WHEEL_RADIUS = 0.14
        
        # Data taken from subscribers
        self.axes = None
        self.buttons = None 
       
        self.angular_z = None
        self.lineer_x = None
        
        #Twist taken from twist_mux inputs and outputs
        self.nav_twist = Twist()
        self.joy_twist = Twist()
        self.cmd_twist = Twist()
        self.nav_twist.angular.x = 1

        self.currentx = 0
        self.currenty = 0
        self.goalx = 0
        self.goaly = 0

        self.wheel_speeds = Float64MultiArray()
        self.scienceSpeeds = Int16MultiArray()        

        self.rate = rospy.Rate(6)
        self.init()
        self.run()
        
    def init(self):
        rospy.loginfo("Serial is started")
        self.wheel_speeds.data = [0,0,0,0,0]
        self.scienceSpeeds.data = [0,0,0,0,0]
        rospy.Subscriber("/joy",Joy,self.joy_cb)
        rospy.Subscriber("/cmd_vel",Twist, self.twist_cb)
        rospy.Subscriber("/imu1/data", Imu, self.imu_cb)
        rospy.Subscriber("/nav_vel", Twist, self.nav_cb)
        rospy.Subscriber("/joystick/twist", Twist, self.joystick_cb)
        rospy.Subscriber("/locomove_base/locomove_base/is_reached", String, self.is_reached_cb)
        self.publisher1 = rospy.Publisher("/drive_system/wheel_speed", Float64MultiArray,queue_size=10) # Drive System RPM data array ?? 
        self.publisher2 = rospy.Publisher("/drive_system/status", String, queue_size=10)
        self.publisher3 = rospy.Publisher("/e__stop", Bool, queue_size=10)
        self.publisher4 = rospy.Publisher("/joystick/science", Int16MultiArray, queue_size=10)
       
    def switch_mode(self):
        if(self.buttons[8]):
            self.emergency = False
            self.is_idle = True
            self.is_drive = False
            self.choose_mode = False
            self.autonomous = False
            self.science = False
            
        if(self.is_idle== True and self.buttons[7]):
            self.choose_mode = True
            self.is_idle = False
            self.is_drive = False
            self.autonomous = False

        if(self.choose_mode and self.buttons[0]):
            self.is_drive = True
            self.choose_mode = False
            self.is_idle = False
            self.autonomous = False

        # if(self.choose_mode and self.buttons[3]):
        #     self.choose_mode = False
        #     self.is_idle = False
        #     self.is_drive = False
        #     self.autonomous = False
        
        if(self.axes[2] == -1 and self.axes[5] == -1):
            self.choose_mode = False
            self.is_idle = False
            self.is_drive = False
            self.autonomous = False
            self.goal_reached = False
            self.emergency = True
        
        if(self.choose_mode and (self.buttons[1] or self.buttons[2])):
            self.is_idle = False
            self.choose_mode = False
            self.is_drive = False
            self.autonomous = False
            self.goal_reached = False
            self.emergency  = False
            self.rk = False
            self.science = True            

    def run(self):
        while not rospy.is_shutdown():
            if(not self.emergency):
                self.publisher3.publish(False)
            if(self.is_idle): 
                self.goal_reached = False
                self.autonomous = False
                self.is_drive = False
                rospy.loginfo("IDLE MODE!")
                self.publisher2.publish("IDLE")
            if(self.is_drive and self.autonomous == False):
                self.goal_reached = False
                rospy.loginfo("OPERATION MODE")
                self.publisher1.publish(self.wheel_speeds)
                self.publisher2.publish("TELEOP_F")
            if(self.choose_mode):
                rospy.loginfo("CHOOSE A MODE!")
                self.publisher2.publish("CHOOSE")
            if(self.autonomous):
                self.goal_reached = False
                self.is_idle = False
                self.is_drive = False
                self.publisher1.publish(self.wheel_speeds)
                rospy.loginfo("AUTONOMOUS MODE")
                self.publisher2.publish("AUTO")
            if(self.goal_reached):
                rospy.loginfo("GOAL REACHED")
                self.publisher2.publish("SUCCESS")
            if(self.emergency):
                self.publisher3.publish(True)
                rospy.loginfo("EMERGENCY")
                self.publisher2.publish("EMERGENCY")
            if(self.science):
                self.publisher4.publish(self.scienceSpeeds)
                rospy.loginfo("SCIENCE")
                self.publisher2.publish("SCIENCE")
            if(rospy.Time.now().secs - self.time > 1 ):
                self.wheel_speeds.data = [0,0,0,0,abs(self.pitch_change)]
            self.rate.sleep()
            
    def joy_cb(self,data):
        self.axes = data.axes
        self.buttons = data.buttons
        self.switch_mode()

        if data.buttons[1]:
            self.scienceSpeeds.data[0] = int((data.axes[1])*255)
            self.scienceSpeeds.data[1] = int((data.axes[4])*255)
            self.scienceSpeeds.data[4] = int(data.axes[7])
        
        elif data.buttons[2]:
            self.scienceSpeeds.data[2] = int((data.axes[1])*255)
            self.scienceSpeeds.data[3] = int((data.axes[4])*255)
            self.scienceSpeeds.data[4] = int(data.axes[7])

        else:
            self.scienceSpeeds.data = [0,0,0,0,0]
                            
    def imu_cb(self,data):
        self.last_pitch = self.curr_pitch

        # Read quaternion orientation from IMU and convert to euler angles
        [self.curr_pitch, self.curr_roll, self.curr_yaw] = euler_from_quaternion(
            [
                data.orientation.x,
                data.orientation.y,
                data.orientation.z,
                data.orientation.w,
            ]
        )  # Convert quaternion to euler angles

        #rospy.loginfo("{} {} {}".format((self.curr_roll/3.14)*180, (self.curr_pitch/3.14)*180, (self.curr_yaw/3.14)*180))

        # Calculate initial pitch angle by taking average of first 5 measurements
        if (self.flag == 0) and self.pitch_counter < 5:
            self.pitch_counter += 1
            self.init_pitch += self.curr_pitch

            if self.pitch_counter == 5:
                self.init_pitch /= 5
                self.flag = 1
        else:
            self.pitch_change = (self.curr_pitch - self.init_pitch)/-3.14*180
            print(abs(self.pitch_change))

    def twist_cb(self,data):
        self.cmd_twist = data
        self.lineer_x = data.linear.x
        self.angular_z = data.angular.z
        self.time = rospy.Time.now().secs
        self.wheel_speed_calc()


    def nav_cb(self,data):
        self.nav_twist = data
        if(self.nav_twist == self.cmd_twist):
            self.autonomous = True
        elif(self.cmd_twist == self.joy_twist and self.joy_twist.linear.x != 0 and self.joy_twist.angular.z != 0):
            self.autonomous = False
            self.is_drive = True
            
    def joystick_cb(self,data):
        self.joy_twist = data
        if(self.goal_reached):
            self.goal_reached = False
            self.is_drive = True
    
    def is_reached_cb(self, data):
        if(data.data == "done"):
            print("bitti")
            self.goal_reached = True
            self.autonomous = False
            self.is_drive = False
            

    def wheel_speed_calc(self):
        if(self.lineer_x != None and self.angular_z != None ):
            
            self.left_wheel = (self.lineer_x - self.angular_z) / 2
            self.right_wheel = (self.lineer_x + self.angular_z) / 2

            self.rpm_left = (self.left_wheel / (2 * pi * self.WHEEL_RADIUS)) * 60
            self.rpm_right = (self.right_wheel / (2 * pi * self.WHEEL_RADIUS)) * 60
            
            self.rpm_left = float(saturate(map(self.rpm_left)))
            self.rpm_right = float(saturate(map(self.rpm_right)))
            
            self.wheel_speeds.data = [self.rpm_left, self.rpm_left, self.rpm_right, self.rpm_right, abs(self.pitch_change)]
                
            # if(self.autonomous):
            #         self.wheel_speeds.data = [self.rpm_left*2,self.rpm_right*2, abs(self.pitch_change)]


#left_wheel + right_wheel = linear_x
#right_wheel - left_wheel = angular_z

if __name__ == "__main__":
    try:
    	SerialNoSteer()
    except rospy.ROSInterruptException:
    	pass
        
