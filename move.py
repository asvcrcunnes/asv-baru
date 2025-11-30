import cv2
import imagezmq
import json
import time
import math
import threading
from imutils.video import VideoStream
from pymavlink import mavutil

# --- CONFIGURATION ---
SERVER_IP = '192.168.1.100'
JPEG_QUALITY = 50
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
PYTHON_DRIVE_MODE = 'MANUAL' 

# --- SHARED DATA ---
vehicle_state = {
    "mode": "UNKNOWN", # Mode terakhir yang valid
    "voltage": 0,
    "armed": False,
    "last_hb": 0 # Waktu heartbeat terakhir diterima
}

command_state = {
    "rc": [1500, 1500, 1000, 1500],
    "buzzer": False
}

data_lock = threading.Lock()

# --- RC DEFINITIONS ---
RC_NEUTRAL = [1500, 1500, 1000, 1500] 
RC_LURUS   = [1500, 1500, 1200, 1500]
RC_KIRI    = [1300, 1500, 1200, 1500]
RC_KANAN   = [1800, 1500, 1200, 1500]

print("Connecting...")
connection = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
connection.wait_heartbeat()
print("Connected!")

def mavlink_worker():
    global vehicle_state
    last_rc_send = 0
    last_buzz = 0
    
    while True:
        # 1. READ DATA
        msg = connection.recv_match(blocking=False)
        if msg:
            m_type = msg.get_type()
            
            if m_type == 'HEARTBEAT':
                # Ambil raw string mode
                raw_mode = mavutil.mode_string_v10(msg).upper()
                
                # --- SOLUSI STABILITAS ---
                # Hanya update jika string BENAR (tidak mengandung "0X")
                # Ini mencegah glitch "MODE (0x000)" mengubah status jadi False
                if "0X" not in raw_mode and raw_mode != "UNKNOWN":
                    armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED > 0
                    with data_lock:
                        vehicle_state["mode"] = raw_mode
                        vehicle_state["armed"] = armed
                        vehicle_state["last_hb"] = time.time() # Catat waktu heartbeat

            elif m_type == 'SYS_STATUS':
                with data_lock:
                    vehicle_state["voltage"] = msg.voltage_battery / 1000.0

        # 2. SEND COMMANDS (10Hz)
        if time.time() - last_rc_send > 0.1:
            with data_lock:
                curr_mode = vehicle_state["mode"]
                last_hb_time = vehicle_state["last_hb"]
                target_rc = command_state["rc"]
                buzz_active = command_state["buzzer"]

            # --- SAFETY CHECK ---
            # Jika Heartbeat hilang lebih dari 2 detik, paksa Release (0)
            # Ini untuk mencegah kapal "nyangkut" di mode MANUAL kalau kabel putus
            is_lost = (time.time() - last_hb_time) > 2.0

            try:
                # Logika Drive: Hanya jika Mode MANUAL dan Koneksi Aman
                if (curr_mode == PYTHON_DRIVE_MODE) and (not is_lost):
                    
                    connection.mav.rc_channels_override_send(
                        connection.target_system, connection.target_component,
                        target_rc[0], target_rc[1], target_rc[2], target_rc[3], 
                        0, 0, 0, 0
                    )
                    
                    if buzz_active and (time.time() - last_buzz > 1.5):
                        tune = "L8 O5 c"
                        connection.mav.play_tune_v2_send(
                            connection.target_system, connection.target_component,
                            1, tune.encode('ascii')
                        )
                        last_buzz = time.time()
                else:
                    # Mode ACRO / Glitch Parah / Lost Connection -> RELEASE
                    connection.mav.rc_channels_override_send(
                        connection.target_system, connection.target_component,
                        0, 0, 0, 0, 0, 0, 0, 0
                    )
            except:
                pass
            
            last_rc_send = time.time()
        
        time.sleep(0.01)

# Start Thread
mav_thread = threading.Thread(target=mavlink_worker)
mav_thread.daemon = True
mav_thread.start()

# --- MAIN CAMERA LOOP ---
def update_cmd(rc_vals, buzzer_on):
    with data_lock:
        command_state["rc"] = rc_vals
        command_state["buzzer"] = buzzer_on

print("Starting Camera...")
vs = VideoStream(src=0).start()
time.sleep(2.0)
sender = imagezmq.ImageSender(connect_to=f'tcp://{SERVER_IP}:5555')
print(f"Sending to {SERVER_IP}...")

try:
    while True:
        frame = vs.read()
        frame = cv2.resize(frame, (640, 640))
        ret, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])

        with data_lock:
            mode_disp = vehicle_state["mode"]

        msg_header = f"Mode:{mode_disp}"

        try:
            reply = sender.send_jpg(msg_header, jpg)
            data = json.loads(reply.decode('utf-8'))
            
            offset = data.get("offset", 0)
            status = data.get("status", "WAITING")
            
            print(f"FC: {mode_disp} | Status: {status} | Off: {offset}")

            if status == "TRACKING":
                if offset < -20:
                    update_cmd(RC_KIRI, False)
                elif offset > 20:
                    update_cmd(RC_KANAN, False)
                else:
                    update_cmd(RC_LURUS, False)
            else:
                update_cmd(RC_NEUTRAL, True)

        except Exception as e:
            print(f"Error: {e}")

finally:
    connection.close()
    vs.stop()