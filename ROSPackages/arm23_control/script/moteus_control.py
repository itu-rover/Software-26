#!/usr/bin/python3
"""
    Author: Yunus Emre Akar / Edited by buorq3io on 31/08/23
    Last Update: 31/08/2023
"""

import rospy
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
import math
import moteus
import asyncio
import numpy as np
from typing import Callable


class NaimDrive:
    def __init__(self):
        rospy.init_node("Moteus_rk_node")
        self.pub = rospy.Publisher("joints_feedback", Float32MultiArray, queue_size=10)
        rospy.Subscriber("joint_commands", Float32MultiArray, self.interface_callback)
        rospy.Subscriber("velocity_commands", Float32MultiArray, self.velocity_callback)
        self.feedback_pos = Float32MultiArray()
        self.joint_vels = 0.4
        self.joint_accels = 0.8
        self.pos_command=[math.nan]*6
        self.velocity_command = [0,0,0,0,0,0]
        self.acc_limit = 1
        self.allow_write_service = rospy.Service("hardware_interface/allow_write", SetBool, self.allow_write_handler)
        self.reset_position_service = rospy.Service("hardware_interface/reset_pos", Trigger, self.reset_handler)
        self.reset_timeout = rospy.Service("hardware_interface/reset_timeout", Trigger, self.timeout_handler)
        self.velocity_service = rospy.Service("hardware_interface/velocity_mode", SetBool, self.allow_velocity_mode)
        self.ALLOW_WRITE = False
        self.VELOCITY_MODE = False
        self.reset_timeout = False
        self.zero_pos = [0,0,0,0,0,0]
        self.motor_pos = [0,0,0,0,0,0]
        self.rate = rospy.Rate(50)
        
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
        self.zero_pos = self.motor_pos
        return TriggerResponse(success=True, message="reset")
    
    def timeout_handler(self, req):
        self.reset_timeout = True
        return TriggerResponse(success=True, message="reset")

    async def main(self):
        rospy.loginfo("Node started")
        pr = moteus.PositionResolution()
        pr.position = moteus.INT16

        self.fdcanusb = moteus.Fdcanusb(path="/dev/ttyACM0", debug_log=None, disable_brs=True)

        self.c1 = moteus.Controller(id = 1, position_resolution=pr, transport=self.fdcanusb)
        self.c2 = moteus.Controller(id = 2, position_resolution=pr, transport=self.fdcanusb)
        self.c3 = moteus.Controller(id = 3, position_resolution=pr, transport=self.fdcanusb)
        self.c4 = moteus.Controller(id = 4, position_resolution=pr, transport=self.fdcanusb)
        self.c5 = moteus.Controller(id = 5, position_resolution=pr, transport=self.fdcanusb)
        self.c6 = moteus.Controller(id = 6, position_resolution=pr, transport=self.fdcanusb)

        await self.c1.set_stop()
        await self.c2.set_stop()
        await self.c3.set_stop()
        await self.c4.set_stop()
        await self.c5.set_stop()
        await self.c6.set_stop()

        rospy.loginfo("moteus started")
        start_time = rospy.Time.now().to_sec()
        temp_time = -1
        while not rospy.is_shutdown():
            curr_time = rospy.Time.now().to_sec()
            elapsed_time = 0 if temp_time == -1 else curr_time - temp_time

            if self.reset_timeout:
                await self.c1.set_stop()
                await self.c2.set_stop()
                await self.c3.set_stop()
                await self.c4.set_stop()
                await self.c5.set_stop()
                await self.c6.set_stop()
                self.reset_timeout = False

            if self.VELOCITY_MODE:
                rospy.loginfo("VELOCITY")
                command1 = await self.c1.set_position(position=math.nan, velocity=self.velocity_command[0], accel_limit=self.acc_limit, maximum_torque = 4, query=True)
                command2 = await self.c2.set_position(position=math.nan, velocity=self.velocity_command[1], accel_limit=self.acc_limit, maximum_torque = 3, query=True)
                command3 = await self.c3.set_position(position=math.nan, velocity=self.velocity_command[2], accel_limit=self.acc_limit, maximum_torque = 3, query=True)
                command4 = await self.c4.set_position(position=math.nan, velocity=self.velocity_command[3], accel_limit=self.acc_limit, maximum_torque = 3, query=True)
                command5 = await self.c5.set_position(position=math.nan, velocity=self.velocity_command[4], accel_limit=self.acc_limit, maximum_torque = 3, query=True)
                command6 = await self.c6.set_position(position=math.nan, velocity=self.velocity_command[5], accel_limit=self.acc_limit, maximum_torque = 3, query=True)

            elif not self.ALLOW_WRITE:
                rospy.loginfo("FEEDBACK")
                command1 = await self.c1.set_position(position=math.nan,
                                                    query = True)
                command2 = await self.c2.set_position(position=math.nan,
                                                    query = True)
                command3 = await self.c3.set_position(position=math.nan,
                                                    query = True)
                command4 = await self.c4.set_position(position=math.nan,
                                                    query = True)
                command5 = await self.c5.set_position(position=math.nan,
                                                    query = True)
                command6 = await self.c6.set_position(position=math.nan,
                                                    query = True)

            else:
                rospy.loginfo("POSITION")
                command1 = await self.c1.set_position(position=math.nan,
                                                velocity=self.joint_vels,
                                                stop_position=self.pos_command[0]*gear1/2+self.zero_pos[0], 
                                                accel_limit=0.7,
                                                velocity_limit = 0.5,
                                                maximum_torque = 4,
                                                query = True)
                command2 = await self.c2.set_position(position=math.nan,
                                                velocity=0.7,
                                                stop_position=(-self.pos_command[1]*gear2/2)+self.zero_pos[1], 
                                                velocity_limit= 0.6,
                                                maximum_torque = 3,
                                                accel_limit=0.8,
                                                query = True)
                command3 = await self.c3.set_position(position=math.nan,
                                                velocity=self.joint_vels,
                                                stop_position=(self.pos_command[2]*gear3/2)+self.zero_pos[2], 
                                                velocity_limit=0.6,
                                                maximum_torque = 3,
                                                accel_limit=0.8,
                                                query = True)
                command4 = await self.c4.set_position(position=math.nan,
                                                velocity=0.8,
                                                stop_position=(self.pos_command[3]*gear4/2)+self.zero_pos[3], 
                                                velocity_limit=1.0,
                                                maximum_torque = 3,
                                                accel_limit=1,
                                                query = True)
                command5 = await self.c5.set_position(position=math.nan,
                                                velocity=self.joint_vels*2,
                                                stop_position=(-self.pos_command[4]*gear5/2)+self.zero_pos[4], 
                                                velocity_limit=1.0,
                                                maximum_torque = 3,
                                                accel_limit=1,
                                                query = True)
                
                command6 = await self.c6.set_position(position=math.nan,
                                                velocity=self.joint_vels*2,
                                                stop_position=(-self.pos_command[5]*gear6/2 + self.pos_command[4] * 0.115)+self.zero_pos[5], 
                                                velocity_limit=1.0,
                                                maximum_torque = 3,
                                                accel_limit=1,
                                                query = True)
                
            gear1 = (40./12.)*(60./12.)
            gear2 = (41./1.)
            gear3 = (30./18.)*(30./1.)
            gear4 = (28./1)
            gear5 = (10./1.)*(49./22.)
            gear6 = (49./22.)*(42./12.)

            self.motor_pos = [(command1.values[moteus.Register.POSITION]), 
                            (command2.values[moteus.Register.POSITION]), 
                            (command3.values[moteus.Register.POSITION]),
                            (command4.values[moteus.Register.POSITION]),
                            (command5.values[moteus.Register.POSITION]),
                            (command6.values[moteus.Register.POSITION])]
            
            self.temperatures = [(command1.values[moteus.Register.TEMPERATURE]),
                                 (command2.values[moteus.Register.TEMPERATURE]),
                                 (command3.values[moteus.Register.TEMPERATURE]),
                                 (command4.values[moteus.Register.TEMPERATURE]),
                                 (command5.values[moteus.Register.TEMPERATURE]),
                                 (command6.values[moteus.Register.TEMPERATURE])]

            self.moteus_modes= [(command1.values[moteus.Register.MODE]), 
                            (command2.values[moteus.Register.MODE]), 
                            (command3.values[moteus.Register.MODE]),
                            (command4.values[moteus.Register.MODE]),
                            (command5.values[moteus.Register.MODE]),
                            (command6.values[moteus.Register.MODE])]
            
            #rospy.logerr(self.temperatures)

            self.pos1 = (self.motor_pos[0]-self.zero_pos[0])/gear1*6.28
            self.pos2 = -(self.motor_pos[1]-self.zero_pos[1])/gear2*6.28
            self.pos3 = (self.motor_pos[2]-self.zero_pos[2])/gear3*6.28
            self.pos4 = (self.motor_pos[3]-self.zero_pos[3])/gear4*6.28
            self.pos5 = -(self.motor_pos[4]-self.zero_pos[4])/gear5*6.28
            self.pos6 = -(self.motor_pos[5]-((self.motor_pos[4]+self.zero_pos[4]) * 0.115)-self.zero_pos[5])/gear6*6.28

            self.feedback_pos.data = [self.pos1, self.pos2, self.pos3, self.pos4, self.pos5, self.pos6]
            self.pub.publish(self.feedback_pos)
            temp_time = curr_time
            await asyncio.sleep(0.02)

        else:
            energies_lst = list(self.energies)
            rospy.loginfo(f"SPENT TOTAL ENERGIES OF THE CONTROLLERS: {energies_lst}")

            with open("energy_info_arm.txt", "a") as file:
                data_to_write = f"{start_time},{rospy.Time.now().to_sec()}"
                for item in energies_lst:
                    data_to_write += "," + str(item)

                data_to_write += "\n"
                file.write(data_to_write)


if __name__=="__main__":
    try:
        asyncio.run(NaimDrive().main())
        rospy.loginfo("finished")
    except asyncio.CancelledError as e:
        rospy.signal_shutdown("Keyboard Interrupt")
    except KeyboardInterrupt:
        rospy.signal_shutdown("Keyboard Interrupt")
