#!/usr/bin/python3
import socket
import threading
import rospy
from std_msgs.msg import Float64MultiArray, String

# Settings
LOCAL_IP = "192.168.1.5"  # IP to listen on 
LOCAL_PORT = 6102          # Port to listen on
REMOTE_IP = "192.168.1.2"  # IP to send messages to
REMOTE_PORT = 6101         # Port to send messages to
BUFFER_SIZE = 1024         # Size of incoming buffer



class UDP():
    def __init__(self):
        # Create UDP socket
        rospy.init_node('udp_listener')
        self.rate = rospy.Rate(10)  
        self.float = [0,0,0,0,0]
        self.timestamp = [0,0]
        self.last_timestamp = 0
        self.current_timestamp = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        rospy.Subscriber("/drive_system/vcu_data", String, self.send_wheel)
        self.pub = rospy.Publisher("/gnss_data", Float64MultiArray, queue_size=10)
        
        try:
            self.sock.bind((LOCAL_IP, LOCAL_PORT))
            print(f"Listening for UDP messages on {LOCAL_IP}:{LOCAL_PORT}")
        except Exception as e:
            print(f"Failed to bind socket: {e}")
            exit(1)

        # Start threads
        # recv_thread = threading.Thread(target=self.receive_gnss, daemon=True)
        # recv_thread.start()
        # self.send_messages()
        self.run()

    def receive_gnss(self):
        try:
            self.sock.sendto("0009".encode("ascii"), (REMOTE_IP, REMOTE_PORT))
            data, addr = self.sock.recvfrom(BUFFER_SIZE)

            self.float[0] = int.from_bytes(data[16:20],byteorder='little',signed=True) * (1e-7) #lat
            self.float[1] = int.from_bytes(data[20:24],byteorder='little',signed=True) * (1e-7) #long
            self.float[2] = 0 #-int.from_bytes(data[21:23],byteorder='little') * (1e-4) #yaw
            self.float[3] = int.from_bytes(data[24:28],byteorder='little') * (1e-5) #cog 
            self.float[4] = int.from_bytes(data[28:32],byteorder='little') * (1e-3) #sog
            rospy.loginfo(f"{self.float[0]:.7f}     {self.float[1]:.7f}     {self.float[3]:.2f}     {self.float[4]:.2f}")

            float_pub = Float64MultiArray()
            float_pub.data = self.float
            self.pub.publish(float_pub)

            # for i in data:
            #     print(i)
            
        except Exception as e:
                print(f"[ERROR] Receiving: {e}")


    def send_wheel(self, msg:String):
        
        self.timestamp[0] = 1 if self.timestamp[0] == 0 else 0

        pid = str(self.timestamp[0]) + str(self.timestamp[1]) + "09" + msg.data

        self.sock.sendto(pid.encode(), (REMOTE_IP, REMOTE_PORT))
        data, addr = self.sock.recvfrom(BUFFER_SIZE)

        self.current_timestamp = data[0]

        self.float[0] = int.from_bytes(data[16:20],byteorder='little',signed=True) * (1e-7) #lat
        self.float[1] = int.from_bytes(data[20:24],byteorder='little',signed=True) * (1e-7) #long
        self.float[2] = 0 #-int.from_bytes(data[21:23],byteorder='little') * (1e-4) #yaw
        self.float[3] = int.from_bytes(data[24:28],byteorder='little') * (1e-5) #cog 
        self.float[4] = int.from_bytes(data[28:32],byteorder='little') * (1e-3) #sog

        # if self.current_timestamp == self.last_timestamp:
        #     self.float[4] = 0
        #     rospy.logerr("VCU Communication Loss")


        rospy.loginfo(f"{self.float[0]:.7f}     {self.float[1]:.7f}     {self.float[3]:.2f}     {self.float[4]:.2f}")
        # rospy.loginfo("-------------------------------")
        # for i in data:
        #     print(i)
        # rospy.loginfo("-------------------------------")

        float_pub = Float64MultiArray()
        float_pub.data = self.float
        self.pub.publish(float_pub)

        self.last_timestamp = self.current_timestamp
    


    def run(self):
        while not rospy.is_shutdown():
            # self.receive_gnss()

            self.rate.sleep()



if __name__ == "__main__":
    UDP() 


# "0008"
