import socket
import rospy
from std_msgs.msg import Int16

speed_val = 0
def callbck(msg):
    global speed_val
    speed_val = msg.data

def send_udp_command(ip, port, command):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(command.encode(), (ip, port))
    sock.close()

def main():
    global speed_val
    rospy.init_node("gripper_teleop")
    rospy.Subscriber("gripper_commands", Int16, callbck)
    # Raspberry Pi'nin IP adresini ve hedef port numarasını ayarlayın
    ip = "192.168.1.100"  # Raspberry Pi'nin IP adresini buraya girin
    port = 12345  # Hedef port numarasını buraya girin
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        # SxF komutunu oluşturacak olan x değerini 1 ile 9 arasında rastgele seçin
        command = "S{}F".format(speed_val)
        # UDP komutunu gönder
        send_udp_command(ip, port, command)
        rate.sleep()

if __name__ == "__main__":
    main()
    
