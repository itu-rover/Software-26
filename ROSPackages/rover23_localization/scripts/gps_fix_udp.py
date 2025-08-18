#!/usr/bin/python3
import socket
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

# Settings
LOCAL_IP = "192.168.1.5"  # IP to listen on 
LOCAL_PORT = 2003          # Port to listen on
REMOTE_IP = "192.168.1.2"  # IP to send messages to
REMOTE_PORT = 6101         # Port to send messages to
BUFFER_SIZE = 1024         # Size of incoming buffer



class GPS_UDP():
    def __init__(self):
        # Create UDP socket
        rospy.init_node('gps_fix_udp')
        self.rate = rospy.Rate(10)  
        self.fix = NavSatFix()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        

        self.fix_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
        self.cog_pub = rospy.Publisher("/cog_data", Float32, queue_size=10)

        try:
            self.sock.bind((LOCAL_IP, LOCAL_PORT))
            print(f"Listening for UDP messages on {LOCAL_IP}:{LOCAL_PORT}")
        except Exception as e:
            print(f"Failed to bind socket: {e}")
            exit(1)
    

        self.run()

    def receive_gnss(self):
            try:
                # self.sock.sendto("0009".encode("ascii"), (REMOTE_IP, REMOTE_PORT))
                data, addr = self.sock.recvfrom(BUFFER_SIZE)

                lat = int.from_bytes(data[0:4],byteorder='little',signed=True) * (1e-7) #lat
                long = int.from_bytes(data[4:8],byteorder='little',signed=True) * (1e-7) #long
                cog = int.from_bytes(data[8:12],byteorder='little') * (1e-5) #cog 
                print(f"lat: {lat:.7f} long: {long:.7f} cog°: {cog:.2f}")
                
                self.fix.latitude = lat
                self.fix.longitude = long
                self.fix.header.frame_id = 'odom'
                self.fix.header.stamp = rospy.Time.now()
                self.fix_pub.publish(self.fix)

                val = Float32()
                val.data = cog
                self.cog_pub.publish(val)
                
            except Exception as e:
                print(f"[ERROR] Receiving: {e}")


    def run(self):
        while not rospy.is_shutdown():
            self.receive_gnss()

            self.rate.sleep()



if __name__ == "__main__":
    GPS_UDP() 

