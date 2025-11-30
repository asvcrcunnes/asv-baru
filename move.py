import cv2
import imagezmq
import json
import time
import math
from imutils.video import VideoStream
from pymavlink import mavutil


# --- CONFIGURATION ---
SERVER_IP = '192.168.1.100'  # REPLACE with your PC's IP
JPEG_QUALITY = 50            # 0 to 100 (Lower is faster, 50 is good balance)

# Menghubungkan ke flight controller melalui port USB/serial
print("Connecting to vehicle...")
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print("Connected to vehicle!")

# Override nilai RC1-RC4
rc1_value_awal = 1500  # Nilai untuk RC channel 1 (range: 1000-2000)
rc2_value_awal = 1500  # Nilai untuk RC channel 2 (range: 1000-2000)
rc3_value_awal = 1000  # Nilai untuk RC channel 3 (range: 1000-2000)
rc4_value_awal = 1500  # Nilai untuk RC channel 4 (range: 1000-2000)

# lurus#
rc1_value_lurus = 1500  # Nilai untuk RC channel 2 (range: 1000-2000)
rc3_value_lurus = 1200

# kiri sitik
rc1_value_kiri = 1300
# kanan sitik
rc1_value_kanan = 1800

# --- BUZZER FUNCTION (NEW) ---
last_buzz_time = 0
def play_search_buzzer(conn):
    global last_buzz_time
    # Bunyikan buzzer setiap 1.5 detik sekali agar tidak membanjiri buffer FC
    if time.time() - last_buzz_time > 1.5:
        # Nada: Beep pendek
        tune = "L8 O5 c" 
        try:
            conn.mav.play_tune_v2_send(
                conn.target_system,
                conn.target_component,
                1, # 1 = QBasic format
                tune.encode('ascii')
            )
            last_buzz_time = time.time()
        except:
            pass
# -----------------------------

def override_rc_channels(connection, rc1_value, rc2_value, rc3_value, rc4_value):
    # Mengirimkan perintah COMMAND_LONG untuk override channel RC1-RC4
    try:
        connection.mav.rc_channels_override_send(
            # target_system (ID dari flight controller)
            connection.target_system,
            # target_component (biasanya ID dari flight controller)
            connection.target_component,
            rc1_value,  # Nilai override untuk RC channel 1 (1000-2000)
            rc2_value,  # Nilai override untuk RC channel 2 (1000-2000)
            rc3_value,  # Nilai override untuk RC channel 3 (1000-2000)
            rc4_value,  # Nilai override untuk RC channel 4 (1000-2000)
            # Set remaining channels (RC5 to RC8) to 0 (no override)
            0, 0, 0, 0
        )
    except Exception as e:
        print(f"Failed to override RC channels: {e}")


def get_vehicle_data():
    # Menerima pesan MAVLink
    msg_gps = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    msg_att = connection.recv_match(type='ATTITUDE', blocking=True)
    msg_bat = connection.recv_match(type='SYS_STATUS', blocking=True)
    msg_vfr = connection.recv_match(type='VFR_HUD', blocking=True)
    msg_rc = connection.recv_match(type='RC_CHANNELS', blocking=True)

    # Coba untuk menerima pesan SERVO_OUTPUT_RAW, tapi tidak wajib blocking=True
    msg_servo = connection.recv_match(type='SERVO_OUTPUT_RAW', blocking=False)

    # Mengambil data GPS (lokasi, ketinggian, dan ground course)
    gps = {
        'lat': msg_gps.lat / 1e7,   # Latitude
        'lon': msg_gps.lon / 1e7,   # Longitude
        'alt': msg_gps.alt / 1e3,   # Altitude (meters)
        'cog': msg_gps.hdg / 100.0  # Course over ground (degrees)
    }

    # Mengambil data status baterai
    batt_status = {
        'voltage': msg_bat.voltage_battery / 1000.0,  # Tegangan baterai dalam volt
        'current': msg_bat.current_battery / 100.0,   # Arus baterai dalam Ampere
        'level': msg_bat.battery_remaining            # Level baterai dalam %
    }

    # Mengambil data attitude (roll, pitch, yaw)
    attitude = {
        'roll': math.degrees(msg_att.roll),
        'pitch': math.degrees(msg_att.pitch),
        'yaw': math.degrees(msg_att.yaw)
    }

    # Mengambil data kecepatan tanah dalam beberapa satuan
    speed = {
        'ground_speed': msg_vfr.groundspeed,    # Groundspeed dalam m/s
        'kmh': msg_vfr.groundspeed * 3.6,       # Groundspeed dalam km/h
        'knot': msg_vfr.groundspeed * 1.94384   # Groundspeed dalam knot
    }

    # Mengambil data tambahan lainnya
    heading = msg_vfr.heading  # Compass heading
    baro = msg_vfr.alt  # Altitude relatif dari barometer

    # Mengambil data RC channel
    rc_channels = {
        'rc1': msg_rc.chan1_raw,
        'rc2': msg_rc.chan2_raw,
        'rc3': msg_rc.chan3_raw,
        'rc4': msg_rc.chan4_raw,
        'rc5': msg_rc.chan5_raw,
        'rc6': msg_rc.chan6_raw
    }

    # Mengambil nilai servo output jika tersedia
    if msg_servo:
        servo_output = {
            'steer': msg_servo.servo1_raw,  # Servo 1
            'th_mid': msg_servo.servo3_raw,  # Servo 3
            'th_left': msg_servo.servo5_raw,  # Servo 5
            'th_right': msg_servo.servo7_raw  # Servo 7
        }
    else:
        # Jika pesan servo tidak ada, set sebagai None atau nilai default
        servo_output = {
            'steer': None,
            'th_mid': None,
            'th_left': None,
            'th_right': None
        }

    # Mendapatkan mode kendaraan
    msg_heartbeat = connection.recv_match(type='HEARTBEAT', blocking=True)
    mode = mavutil.mode_string_v10(msg_heartbeat)

    # Memeriksa apakah kendaraan armed
    is_armed = msg_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED > 0

    # Memeriksa apakah kendaraan dapat di-arm
    is_armable = (
        mode in ["GUIDED", "AUTO", "STABILIZE"] and
        batt_status['level'] > 20 and  # Level baterai lebih dari 20%
        msg_bat.current_battery >= 0  # Status sistem tidak menunjukkan masalah
    )

    # Mengembalikan semua data dalam satu dictionary
    return {
        "gps": gps,
        "bat_status": batt_status,
        "heading": heading,
        "speed": speed,
        "baro": baro,
        "attitude": attitude,
        "rc_channels": rc_channels,  # Data RC
        "servo_output": servo_output,  # Data Servo
        "mode": mode,                  # Mode kendaraan
        "is_armed": is_armed,           # Status armed
        "is_armable": is_armable,        # Status armable
    }
    


# 1. Start the Camera in a separate thread (Fix 2)
# This allows frames to be read constantly in the background
print("Starting Camera Thread...")
vs = VideoStream(src=0).start()
time.sleep(2.0) # Let camera warm up

# 2. Setup ZMQ Sender
sender = imagezmq.ImageSender(connect_to=f'tcp://{SERVER_IP}:5555')

print(f"Client sending to {SERVER_IP} at 640x640...")

while True:
    # 3. Read the latest frame from the thread (Instant)
    frame = vs.read()
    
    # 4. Force Resize to 640x640 (User Requirement)
    # Note: If your camera is 4:3 (e.g. 640x480), this will stretch the image slightly
    frame = cv2.resize(frame, (640, 640))

    # 5. Compress to JPEG (Fix 1)
    # We send compressed bytes, not the raw pixel array
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
    ret, jpg_buffer = cv2.imencode('.jpg', frame, encode_param)

    # 6. Send JPEG and Wait for Reply
    # Note: We use send_jpg instead of send_image
    # We send "{}" because we have no extra sensor data right now
    try:
        reply_bytes = sender.send_jpg("{}", jpg_buffer)
        
        # 7. Process Reply
        feedback = json.loads(reply_bytes.decode('utf-8'))
        
        offset = feedback.get("offset", 0)
        status = feedback.get("status", "WAITING")
        
        if status == "TRACKING":
            print(f"Action: {offset}")
            if offset < -20:
                override_rc_channels(connection, rc1_value_kiri,
                                     rc2_value_awal, rc3_value_lurus, rc4_value_awal)
            if offset > 20:
                override_rc_channels(connection, rc1_value_kanan,
                                     rc2_value_awal, rc3_value_lurus, rc4_value_awal)
            elif -20 <= offset <= 20:
                override_rc_channels(connection, rc1_value_awal,
                                     rc2_value_awal, rc3_value_lurus, rc4_value_awal)
        else:
            print(f"Status: {status}")
            # --- TAMBAHAN BUZZER ---
            play_search_buzzer(connection)
            # -----------------------
            override_rc_channels(connection, rc1_value_awal,
                                     rc2_value_awal, rc3_value_awal, rc4_value_awal)
            
            
    except Exception as e:
        print(f"Communication Error: {e}")
    # Optional: Stop gracefully
    # (Note: cv2.imshow isn't needed here if you want max speed, 
    # but useful for debugging if a screen is attached to Pi)
    # cv2.imshow("Pi View", frame)
    # if cv2.waitKey(1) == ord('q'): break