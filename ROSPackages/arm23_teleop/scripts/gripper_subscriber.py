import socket
import serial
import time
def receive_udp_commands(ip, port, serr):
    # UDP soketini oluştur
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))

    print("UDP dinleniyor...")
    while True:
        data, addr = sock.recvfrom(1024)  # 1024 byte'a kadar veri al
        serr.write(data)
        time.sleep(0.02)


def main():
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=0.02)
    # Raspberry Pi'nin IP adresini ve dinlenecek port numarasını ayarlayın
    ip = "192.168.1.100"  # Raspberry Pi'nin IP adresini buraya girin
    port = 12345  # Dinlenecek port numarasını buraya girin

    # UDP komutlarını dinle
    receive_udp_commands(ip, port, ser)

if __name__ == "__main__":
    main()
