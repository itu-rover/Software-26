#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
from std_msgs.msg import Float64MultiArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf
from math import pi, sin, cos, atan2, degrees
import numpy as np

radius = 0.3

# calibration_params = np.load('/home/iturover/iturover23_ws/src/rover23_localization/scripts/calibration_params.npy', allow_pickle=True).item()

class ERC_Odometry():
    def __init__(self):
        rospy.init_node("erc_odom")

        self.odom = Odometry()
        self.imu = Imu()
        self.mag = MagneticField()

        self.odom_broadcaster = tf.TransformBroadcaster()

        self.pos = Point(0,0,0)
        self.vx = 0 
        self.vy = 0
        self.linear_accel = [0,0]
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.orientation = [0,0,0,0]
        self.wheels = [0.0,0.0,0.0,0.0]

        self.rate = rospy.Rate(10)
        self.current = rospy.Time.now()
        self.last = rospy.Time.now()

        self.imu_sub = rospy.Subscriber("/imu/data", Imu, callback=self.imu_cb)
        self.mag_sub = rospy.Subscriber("imu/mag", MagneticField, callback=self.mag_cb)
        self.wheel_sub = rospy.Subscriber("/drive_system/wheel_angular_velocities", Float64MultiArray, callback=self.wheel_cb)

        self.odom_pub = rospy.Publisher("/erc/odom", Odometry, queue_size=10)

        self.run()



    def imu_cb(self, msg:Imu):
        self.imu = msg
        self.roll, self.pitch, self.yaw = euler_from_quaternion([self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w])
        self.orientation = quaternion_from_euler(self.roll, -self.pitch, self.yaw)
        
    # def mag_cb(self, msg:MagneticField):
    #     self.mag = msg
    #     self.mag.magnetic_field.x = (msg.magnetic_field.x - calibration_params['offset_x']) / calibration_params['scale_x']
    #     self.mag.magnetic_field.y = (msg.magnetic_field.y - calibration_params['offset_y']) / calibration_params['scale_y']
        # rospy.loginfo(self.mag)

    def convert_rpm(self, rpm:float):          
        v_linear = (rpm/60) * (2*pi*radius)

        return v_linear

    def wheel_cb(self, msg:Float64MultiArray):
        vel = list(msg.data)
        for i in range(len(vel)):
            if ((vel[i] > 0) and (vel[i]<20)): vel[i] = 0  
            self.wheels[i] = self.convert_rpm(vel[i]/(64*7*10))


    def calculate_heading(self):
        heading = atan2(self.mag.magnetic_field.y, self.mag.magnetic_field.x)
        heading_degrees = degrees(heading)
        
        if heading_degrees < 0:
            heading_degrees += 130 #360

        return heading_degrees

    def negate_gravity(self):
        self.linear_accel[0] = self.imu.linear_acceleration.x - (9.81 * sin(-self.pitch))
        self.linear_accel[1] = self.imu.linear_acceleration.y - (9.81 * sin(self.roll))
        
        # rospy.loginfo(f"\n roll = {self.roll:3f} \n pitch = {-self.pitch:.3f} \n yaw = {self.yaw:.3f}")\
        # rospy.loginfo(f"\n ax = {(self.linear_accel[0]):3f} --- {(self.imu.linear_acceleration.x):3f} \n ay = {self.linear_accel[1]:.3f} --- {(self.imu.linear_acceleration.y):3f}")
        


    def run(self):
        while not rospy.is_shutdown():
            self.current = rospy.Time.now()
            dt = (self.current - self.last).to_sec()

            self.negate_gravity()
            
            # rospy.loginfo(self.calculate_heading())

            v_avg = 0
            for vel in self.wheels:
                v_avg += vel
            v_avg /= 4

            self.vx = v_avg * cos(self.yaw)
            self.vy = v_avg * sin(self.yaw) 

            self.pos.x += (self.vx * dt) 
            self.pos.y += (self.vy * dt)

            self.odom.header.frame_id = "odom"
            self.odom.child_frame_id = "base_link"
            self.odom.header.stamp = self.current

            self.odom.pose.pose = Pose(self.pos, Quaternion(self.orientation[0], self.orientation[1], self.orientation[2], self.orientation[3]))
            # self.odom.pose.pose = Pose(self.pos, Quaternion(self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w))
            self.odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(self.imu.angular_velocity.x, self.imu.angular_velocity.y, self.imu.angular_velocity.z))

            self.odom_pub.publish(self.odom)

            # self.odom_broadcaster.sendTransform((self.pos.x, self.pos.y, self.pos.z), 
            #                                     self.orientation,
            #                                     self.current,
            #                                     "base_link",
            #                                     "odom")

            self.last = self.current
            self.rate.sleep()



if __name__ == '__main__':
    ERC_Odometry()

