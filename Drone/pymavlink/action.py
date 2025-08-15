from pymavlink import mavutil
import time

print("Bağlanıyor...")

# 🔹 UDP bağlantısı (SITL komutundaki --out ile aynı olmalı)
vehicle = mavutil.mavlink_connection('udp:127.0.0.1:14550', autoreconnect=True)

# 🔹 HEARTBEAT bekle
vehicle.wait_heartbeat(timeout=15)
print(f"HEARTBEAT alındı. Sistem ID: {vehicle.target_system}")

# 🔹 GUIDED moda geç
mode_id = vehicle.mode_mapping()["GUIDED"]
vehicle.mav.set_mode_send(
    vehicle.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)
print("GUIDED moduna geçildi.")

# 🔹 Arm et
vehicle.arducopter_arm()
print("Araç ARM edildi, motorlar hazır.")

time.sleep(2)

# 🔹 Kalkış fonksiyonu
def takeoff(alt):
    print(f"{alt} metreye kalkış yapılıyor...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt
    )
    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            current_alt = msg.relative_alt / 1000.0
            print(f"Anlık irtifa: {current_alt:.2f} m")
            if current_alt >= alt * 0.95:
                print("İstenen irtifaya ulaşıldı ")
                break
        time.sleep(0.5)

takeoff(10)

# 🔹 Hareket etme (NED koordinat sistemi)
def move_ned(north, east, down):
    vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
        10,  # zaman damgası (ms)
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        int(0b0000111111111000),  # sadece pozisyon kullan
        north, east, down,
        0, 0, 0,  # hız
        0, 0, 0,  # ivme
        0, 0      # yaw
    ))

# 🔹 Waypoint gitme
def go_to(lat, lon, alt):
    vehicle.mav.mission_item_send(
        vehicle.target_system,
        vehicle.target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2, 0, 0, 0, 0, 0,
        lat, lon, alt
    )

print("🚁 Komutlar hazır. move_ned() veya go_to() ile hareket verebilirsin.")
