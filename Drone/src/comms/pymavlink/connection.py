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

# Pil bilgisi
battery = vehicle.recv_match(type='BATTERY_STATUS', blocking=True, timeout=5)
if battery:
    print(f"Pil Durumu: %{battery.battery_remaining}")
else:
    print("Pil bilgisi alınamadı.")

# Uçuş modu
hb = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
if hb:
    mode = mavutil.mode_string_v10(hb)
    print(f"Uçuş Modu: {mode}")
else:
    print("Uçuş modu alınamadı.")
