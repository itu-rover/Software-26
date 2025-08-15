from pymavlink import mavutil
import time

print("Bağlanıyor...")

# UDP bağlantısı
vehicle = mavutil.mavlink_connection('udpin:localhost:14550', autoreconnect=True)

# HEARTBEAT bekle
vehicle.wait_heartbeat(timeout=15)
print("HEARTBEAT alındı.")

# Sistem ID kontrolü
while vehicle.target_system == 0:
    print("Sistem ID alınamadı, tekrar deneniyor...")
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        vehicle.target_system = msg.get_srcSystem()
    time.sleep(1)

print(f"Bağlantı başarılı! Sistem ID: {vehicle.target_system}")

def get_alt():
    message = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking= True)
    alt=message.relative_alt
    alt = alt/1000
    return alt
def takeoff(alt):
    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component,mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    while True: 
        current_alt= get_alt()
        if current_alt< alt:
            print(f"Anlik irtifa {current_alt}")
        elif current_alt >=  alt:
            print("Istenilen irtifaya ulasildi ")
            break

vehicle.set_mode("GUIDED")
vehicle.arducopter_arm()
print("arac arm edildi")
takeoff(10)