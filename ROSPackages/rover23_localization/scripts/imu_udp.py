#!/usr/bin/python3
import socket
import struct
import rospy
from sensor_msgs.msg import Imu

#--config--
LOCAL_IP = "192.168.1.2" # IP to listen on
LOCAL_PORT = 6300 # Port of the IP to listen on
FRAME_ID = "imu_link" # the name of the coordinate frame of the imu

class imu_udp():
    def __init__(self):
        self.pub = rospy.Publisher("/imu/data", Imu, queue_size=10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        try:
            self.sock.bind((LOCAL_IP, LOCAL_PORT))
            rospy.loginfo(f"listening for UDP messages on {LOCAL_IP}:{LOCAL_PORT}")
        except Exception as e:
            rospy.loginfo(f"failed to bind socket: {e}")
            exit(1)
        
    def parse_and_create(self, data):
        parsed_data = struct.unpack("<16f", data)

        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = FRAME_ID

        imu_msg.linear_acceleration.x = parsed_data[0] / 1048576.0
        imu_msg.linear_acceleration.y = parsed_data[1] / 1048576.0
        imu_msg.linear_acceleration.z = parsed_data[2] / 1048576.0

        imu_msg.angular_velocity.x = parsed_data[3] / 67108864
        imu_msg.angular_velocity.y = parsed_data[4] / 67108864
        imu_msg.angular_velocity.z = parsed_data[5] / 67108864

        imu_msg.orientation.w = parsed_data[9]
        imu_msg.orientation.x = parsed_data[10]
        imu_msg.orientation.y = parsed_data[11]
        imu_msg.orientation.z = parsed_data[12]

        # set covariance to "unknown" as per ROS convention (first element is -1), the EKF will handle estimating the covariance
        imu_msg.linear_acceleration_covariance[0] = -1
        imu_msg.angular_velocity_covariance[0] = -1
        imu_msg.orientation_covariance[0] = -1

        # euler angles and magnetic field data are ignored

        return imu_msg
    
    def run(self):
        while not rospy.is_shutdown():
            try:
                data, addr = self.sock.recvfrom(1024)  # buffer size
                imu_msg = self.parse_and_create(data)
                self.pub.publish(imu_msg)

            except Exception as e:
                rospy.loginfo(f"error receiving or parsing data: {e}")

if __name__ == "__main__":
    try:
        rospy.init_node("imu_udp_node")
        imu_listener = imu_udp()
        imu_listener.run()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("shutting down imu_udp_node.")



