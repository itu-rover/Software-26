#!/usr/bin/env python3
# Author:   B. Burak Payzun / buorq3io
# Date:     13-09-23
#

import os
import asyncio
import math
import moteus
import rospy
import datetime as dt
from contextlib import suppress
from std_msgs.msg import Float32MultiArray, Float32, String, Bool
from geometry_msgs.msg import Twist
from typing import Callable
import numpy as np

moteus_modes = [
    "stopped",  # 0
    "fault",  # 1
    "preparing to operate",  # 2
    "preparing to operate",  # 3
    "preparing to operate",  # 4
    "PWM mode",  # 5
    "voltage mode",  # 6
    "voltage FOC",  # 7
    "voltage DQ",  # 8
    "current",  # 9
    "position",  # 10
    "timeout",  # 11
    "zero velocity",  # 12
    "stay within",  # 13
    "measure inductance",  # 14
    "brake"  # 15
]

moteus_faults = {
        0: "Success",
        1: "DmaStreamTransferError",
        2: "DmaStreamFifoError",
        3: "UartOverrunError",
        4: "UartFramingError",
        5: "UartNoiseError",
        6: "UartBufferOverrunError",
        7: "UartParityError",
        32: "Calibration Fault - The encoder was not able to sense a magnet during calibration",
        33: "Motor Driver Fault - The most common reason for this is undervoltage, "
            "moteus attempted to draw more current than the supply could provide.",
        34: "Over Voltage - The bus voltage exceeded servo.max_voltage. This can happen due to misconfiguration, or if "
            "the controller regenerated power with a supply that cannot sink power and no flux braking was configured.",
        35: "Encoder Fault - The encoder readings are not consistent with a magnet being present.",
        36: "Motor Not Configured - The moteus_tool --calibrate procedure has not been run on this motor.",
        37: "PWM Cycle Overrun - An internal firmware error",
        38: "Over Temperature - The maximum configured temperature has been exceeded",
        39: "Outside Limit - An attempt was made to start position control while outside the bounds "
            "configured by servopos.position_min and servopos.position_max.",
        40: "Under Voltage - The voltage was too low",
        41: "Config Changed - A configuration value was changed during operation that requires a stop",
        42: "Theta Invalid - No valid commutation encoder is available",
        43: "Position Invalid - No valid output encoder is available",
        44: "DriverEnableFault"
}


class MoteusController:
    def __init__(self):
        rospy.init_node("moteus_controller")

        self.left_motor = 0
        self.right_motor = 0

        self.x = 0
        self.z = 0
        
        self.set_stop = False

        # Acceleration limit of motors
        self.accel_limit = 180

        # Robot specific parameters used in twist to motor speed calculations
        #
        # Linear and twist ratio values can be calculated in theory
        # But somehow empirical values result in better performance
        #
        # twist_ratio can be auto-tuned using calibrate_twist.py

        self.WHEEL_RADIUS = 0.27 / 2
        self.reduction = 10
        self.divisor = 2 * math.pi * self.WHEEL_RADIUS
        self.linear_ratio = 1
        self.twist_ratio = 0.7507005913268557 * 0.9769641620236444 * 1.020279066730332 

        self.imu_yaw = 0
        rospy.Subscriber("/cmd_vel", Twist, self.twist_to_wheel_speed_callback)
        self.wheel_position_publisher = rospy.Publisher("/rover/wheel_position", Float32MultiArray, queue_size=10)
        self.wheel_speed_publisher = rospy.Publisher("/rover/wheel_speed", Float32MultiArray, queue_size=10)
        self.set_stop_sub = rospy.Subscriber('/set_stop', Bool, self.set_stop_cb)
        
        self.temperature_publisher = rospy.Publisher("/rover/diagnostics/moteus_temperature",
                                                     Float32MultiArray, queue_size=10)
        
        self.mode_publisher = rospy.Publisher("/rover/diagnostics/moteus_mode", String, queue_size=10)
        self.current_publisher = rospy.Publisher("/rover/diagnostics/moteus_current", Float32MultiArray, queue_size=10)

        # Voltage readings
        self.voltage_publisher = rospy.Publisher("/rover/diagnostics/voltage", Float32, queue_size=10)
        self.voltage_array = [0]  # For moving average filter
        rospy.Timer(rospy.Duration(1), self.voltage_callback)

        self.fdcanusb = None
        self.qr = moteus.QueryResolution()

        self._CALC_TYPE = 1
        self.calculate_power: Callable = self._choose_power_method(self._CALC_TYPE)

        self.energies = np.array([0] * 4, dtype=np.float32)
        self.pwr_pub = rospy.Publisher("/moteus_power_rover", Float32MultiArray, queue_size=10)

        self.last_time = rospy.Time.now().to_sec()
        self.timeout = 0.5
        
    def set_stop_cb(self, data):
        self.set_stop = data

    def voltage_callback(self, event=None):
        self.voltage_publisher.publish(np.mean(self.voltage_array))
        
    def twist_to_wheel_speed_callback(self, data):
        self.x = data.linear.x / self.divisor * self.linear_ratio
        self.z = -data.angular.z / self.divisor * self.twist_ratio

        self.last_time = rospy.Time.now().to_sec()

    def calculate_motor_values(self):
        self.right_motor = self.x - self.z
        self.left_motor = self.x + self.z

    @staticmethod
    def _choose_power_method(calc_type) -> Callable:
        if calc_type == 0:

            def dq_currents(results):
                voltage = moteus.Register.VOLTAGE
                q_current = moteus.Register.Q_CURRENT
                d_current = moteus.Register.D_CURRENT

                powers = np.array([np.sqrt((result.values[q_current] ** 2) + (result.values[d_current] ** 2))
                                   * result.values[voltage] for result in results], dtype=np.float32)
                return powers
            return dq_currents

        elif calc_type == 1:

            def torque_velocity(results):
                torque = moteus.Register.TORQUE
                velocity = moteus.Register.VELOCITY

                powers = np.array([np.abs(result.values[torque] * 2 * np.pi * result.values[velocity])
                                   for result in results], dtype=np.float32)
                return powers
            return torque_velocity

        else:
            raise ValueError("`calc_type` can only take [0, 1] as integer values.")

    async def main(self):
        # Set up query resolution for diagnostics readings
        self.qr.q_current = moteus.F32
        self.qr.d_current = moteus.F32
        self.qr.voltage = moteus.INT16
        self.qr.temperature = moteus.INT16

        self.fdcanusb = moteus.Fdcanusb(path="/dev/ttyACM0")

        c1 = moteus.Controller(id=2, query_resolution=self.qr, transport=self.fdcanusb)  # Right moteus FRONT
        c2 = moteus.Controller(id=4, query_resolution=self.qr, transport=self.fdcanusb)  # Left moteus FRONT
        c3 = moteus.Controller(id=3, query_resolution=self.qr, transport=self.fdcanusb)  # Right moteus REAR
        c4 = moteus.Controller(id=1, query_resolution=self.qr, transport=self.fdcanusb)  # Left moteus REAR
        await c1.set_stop()
        await c2.set_stop()
        await c3.set_stop()
        await c4.set_stop()

        rospy.sleep(1)
        rospy.loginfo("Connected to motei.")

        start_time = rospy.Time.now().to_sec()
        temp_time, curr_time, elapsed_time = start_time, start_time, 0

        points = 4
        counter = 0
        power_holder = np.zeros(shape=(points, 4))

        while not rospy.is_shutdown():
            if (count_mod := counter % (points - 1)) == (points - 2):
                curr_time = rospy.Time.now().to_sec()
                elapsed_time = curr_time - temp_time

            # rospy.loginfo("Not Timeout.")
            self.calculate_motor_values()

            if rospy.Time.now().to_sec() - self.last_time > self.timeout:
                self.right_motor = 0
                self.left_motor = 0
                # feedforward_torque = (1.0 if self.left_motor > 0 else -1.0),
            right_rpm = self.right_motor * self.reduction
            left_rpm = -self.left_motor * self.reduction
            
            # right_torque = (1.0 if right_rpm > 0 else -1.0)
            # left_torque = (1.0 if left_rpm > 0 else -1.0)

            if self.set_stop:
                await c1.set_stop()
                await c2.set_stop()
                await c3.set_stop()
                await c4.set_stop()
                
            else:   
                state_r_front = await c1.set_position(position=math.nan, velocity=right_rpm,
                                                        accel_limit=self.accel_limit, maximum_torque=4.0, query=True)

                state_l_front = await c2.set_position(position=math.nan, velocity=left_rpm,
                                                        accel_limit=self.accel_limit, maximum_torque=4.0, query=True)

                state_r_rear = await c3.set_position(position=math.nan, velocity=right_rpm,
                                                        accel_limit=self.accel_limit, maximum_torque=4.0, query=True)

                state_l_rear = await c4.set_position(position=math.nan, velocity=left_rpm,
                                                        accel_limit=self.accel_limit, maximum_torque=4.0, query=True)

            results = (state_r_front, state_l_front, state_r_rear, state_l_rear)
            powers: np.ndarray = self.calculate_power(results)
            power_holder[count_mod + 1] = powers

            if count_mod == (points - 2):
                self.energies += np.array([(row[0] + 3 * row[1] + 3 * row[2] + row[3]) * (elapsed_time / (8 * 3600))
                                           for row in power_holder.T])
                self.pwr_pub.publish(data=np.append(powers, self.energies).tolist())

                power_holder[0] = power_holder[-1]
                temp_time = curr_time

                rospy.loginfo(powers)
                rospy.logwarn(self.energies)

            # Voltage moving average filter
            self.voltage_array.append((state_l_front.values[moteus.Register.VOLTAGE]))
            self.voltage_array = self.voltage_array[-500:]

            # Temperature publish
            self.temperature_publisher.publish(
                data=[result.values[moteus.Register.TEMPERATURE] for result in results])

            # Wheel position publish
            self.wheel_position_publisher.publish(
                data=[result.values[moteus.Register.POSITION] for result in results])

            # rospy.loginfo([result.values[moteus.Register.COMMAND_VELOCITY] for result in results])

            # Wheel speed publish
            self.wheel_speed_publisher.publish(
                data=[state_r_front.values[moteus.Register.VELOCITY],
                      -state_l_front.values[moteus.Register.VELOCITY],  # Minus sign is to correct the direction
                      state_r_rear.values[moteus.Register.VELOCITY],  # Changing moteus' config is a better solution
                      -state_l_rear.values[moteus.Register.VELOCITY]])

            self.current_publisher.publish(
                data=[result.values[moteus.Register.Q_CURRENT] for result in results])

            modes = [result.values[moteus.Register.MODE] for result in results]
            self.mode_publisher.publish(f"Right Front Mode: {modes[0]} Right Rear Mode: {modes[2]} "
                                        f"Left Front Mode: {modes[1]} Left Rear Mode: {modes[3]}")

            with suppress(Exception):
                for i, mode in enumerate(modes):
                    if mode == 1:
                        rospy.logerr(moteus_faults[results[i].values[moteus.Register.FAULT]])

            counter += 1
            await asyncio.sleep(0.02)

        else:
            energies_lst = list(self.energies)
            rospy.logwarn(f"SPENT TOTAL ENERGIES OF THE CONTROLLERS: {energies_lst}")
            rospy.logwarn(f"TOTAL: {sum(energies_lst)}")

            home_directory = os.path.expanduser("~")
            directory = os.path.join(home_directory, "spent_energy_infos")

            if not os.path.isdir(directory):
                os.makedirs(directory)

            with open(os.path.join(directory, "energy_info_rover.csv"), "a") as file:
                data_to_write = f"{dt.datetime.fromtimestamp(start_time)},{dt.datetime.now()}"
                for item in energies_lst:
                    data_to_write += "," + str(item)

                data_to_write += "," + str(sum(energies_lst)) + "\n"
                file.write(data_to_write)


if __name__ == '__main__':
    try:
        asyncio.run(MoteusController().main())
    except (asyncio.CancelledError, KeyboardInterrupt) as e:
        rospy.signal_shutdown("Keyboard Interrupt")
