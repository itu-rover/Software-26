#!/usr/bin/python3
import rospy
from std_msgs.msg import Float32MultiArray, String, Float32
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
import math
import serial
from numpy import interp

class NaimDrive():
    def __init__(self):
        rospy.init_node("Moteus_rk_node")
        self.pub = rospy.Publisher("joints_feedback", Float32MultiArray, queue_size=20)
        self.volt_pub = rospy.Publisher("volt_value", Float32, queue_size=10)
        rospy.Subscriber("joint_commands", Float32MultiArray, self.interface_callback)
        rospy.Subscriber("velocity_commands", String, self.velocity_callback)

        self.feedback_pos = Float32MultiArray()
        self.pos_command=[math.nan]*6
        self.velocity_command = ""

        self.allow_write_service = rospy.Service("hardware_interface/allow_write", SetBool, self.allow_write_handler)
        #self.reset_position_service = rospy.Service("hardware_interface/reset_pos", Trigger, self.reset_handler)
        self.reset_timeout = rospy.Service("hardware_interface/reset_timeout", Trigger, self.timeout_handler)
        self.velocity_service = rospy.Service("hardware_interface/velocity_mode", SetBool, self.allow_velocity_mode)

        self.ALLOW_WRITE = False
        self.VELOCITY_MODE = False
        self.reset_timeout = False
        rospy.loginfo("Services started")
        self.zero_pos = [0,0,0,0,0,0]
        self.motor_pos = ""
        self.joint_pos = [0,0,0,0,0,0]
        self.rate = rospy.Rate(50)

        self.com_check_byte = 0

        self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.05) # Edit this for appropriate symbolic link

    def interface_callback(self, msg):
        self.pos_command = msg.data

    def velocity_callback(self, msg):
        self.velocity_command = msg.data

    def allow_velocity_mode(self, req):
        self.VELOCITY_MODE = req.data
        return SetBoolResponse(success=True, message=f"Velocity command {req.data}")

    def allow_write_handler(self, req):
        self.ALLOW_WRITE = req.data
        return SetBoolResponse(success=True, message=f"Allow write command {req.data}")

    def reset_handler(self, req):
        self.zero_pos = self.joint_pos
        rospy.logerr("Zero position reset")
        return TriggerResponse(success=True, message="reset")
    
    def timeout_handler(self, req):
        self.reset_timeout = True
        return TriggerResponse(success=True, message="reset")
    
    def to_serial(v):
        return '{:04d}'.format(abs(int(v)))
    
    def clamp(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def main(self):
        rospy.loginfo("Node started")
        
        end_marker = b"X"
        start_marker = b"S"
        package_size = 31

        first_data = self.ser.read(package_size * 2)
        self.ser.read(first_data.find(start_marker))

        while not rospy.is_shutdown():
            
            eps = 0

            self.motor_pos = self.ser.read(package_size)
            
            if len(self.motor_pos) != package_size:           
                continue

            if self.reset_timeout:
                pass
                
            #print(f"{self.ser.readline()} + readline\n")
	    
            if self.VELOCITY_MODE:
                rospy.logerr_once("VELOCITY")
                msg = self.velocity_command

            elif not self.ALLOW_WRITE:
                rospy.logerr_once("FEEDBACK")
                msg = "S555555F"

            else:
                rospy.logerr_once("POSITION")

                for i in range(6):
                    eps += abs(self.pos_command[i] - self.joint_pos[i])/6

                if eps > 0.1: 
                    pass
                else:
                    msg = "S555555F"
                print(self.pos_command)
                msg = "S"
                msg += str(round(interp(self.pos_command[0], [-3.14, 3.14], [0, 9999])))
                msg += str(round(interp(self.pos_command[1], [-2.5, 2.5], [9999, 0])))
                msg += str(round(interp(self.pos_command[2], [-2.2, 2.2], [0, 9999])))
                msg += str(round(interp(self.pos_command[3], [-3.14, 3.14], [0, 9999])))
                msg += str(round(interp(self.pos_command[4], [-3.2, 3.2], [9999, 0])))
                msg += str(round(interp(self.pos_command[5]-self.pos_command[4]*0.115, [-6.28, 6.28], [0, 9999])))
                msg += "F"
                rospy.loginfo(msg)    
 
            self.ser.write(msg.encode())

            gear1 = (40./12.)*(60./12.)
            gear2 = (41./1.)
            gear3 = (30./18.)*(30./1.)
            gear4 = (28./1)
            gear5 = (10./1.)*(49./22.)
            gear6 = (49./22.)*(42./12.)

            # 1.eksen 4. eksen 5.eksen 
            
            
            if self.motor_pos[0:1] == start_marker and self.motor_pos[-1:] == end_marker:
                self.joint_pos[0] = interp(int(self.motor_pos[1:5], 16), [0, 9999], [-3.14, 3.14])
                self.joint_pos[1] = interp(int(self.motor_pos[5:9], 16), [0, 9999], [2.5, -2.5])
                self.joint_pos[2] = interp(int(self.motor_pos[9:13], 16), [0, 9999], [-2.2, 2.2])
                self.joint_pos[3] = interp(int(self.motor_pos[13:17], 16), [0, 9999], [-3.14, 3.14])
                self.joint_pos[4] = interp(int(self.motor_pos[17:21], 16), [0, 9999], [3.2, -3.2])
                self.joint_pos[5] = interp(int(self.motor_pos[21:25], 16), [0, 9999], [-6.28, 6.28])
                try:
                    volt_value = (int(self.motor_pos[25:30], 16))/1000.0
                    self.volt_pub.publish(volt_value)
                except:
                    pass

            else:
                rospy.logerr("Something went wrong")
                temp_data = self.ser.read(package_size * 2)
                self.ser.read(temp_data.find(start_marker))

            self.feedback_pos.data = self.joint_pos
            self.pub.publish(self.feedback_pos)
            rospy.Rate(50).sleep()

if __name__=="__main__":
    try:
        NaimDrive().main()
        rospy.loginfo("finished")
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")
