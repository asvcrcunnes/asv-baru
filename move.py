import cv2
import imagezmq
import json
import time
import math
import threading
from imutils.video import VideoStream
from pymavlink import mavutil

# --- CONFIGURATION ---
SERVER_IP = '192.168.1.100'  # Ganti IP PC
JPEG_QUALITY = 50
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# Mode di mana Python mengambil alih kendali
PYTHON_DRIVE_MODE = 'MANUAL' 

# --- SHARED DATA & LOCK ---
vehicle_state = {
    "mode": "UNKNOWN",
    "voltage": 0,
    "armed": False
}

command_state = {
    "rc": [1500, 1500, 1000, 1500], # [Roll, Pitch, Throttle, Yaw]
    "buzzer": False
}

data_lock = threading.Lock()

# --- RC VALUES CONFIG ---
# [Roll, Pitch, Throttle, Yaw]
RC_NEUTRAL = [1500, 1500, 1000, 1500] 
RC_LURUS   = [1500, 1500, 1200, 1500]
RC_KIRI    = [1300, 1500, 1200, 1500]
RC_KANAN   = [1800, 1500, 1200, 1500]

# --- MAVLINK CONNECTION ---
print("Connecting to vehicle...")
connection = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE)
connection.wait_heartbeat()
print("Connected!")

# --- MAVLINK WORKER THREAD ---
def mavlink_worker():
    global vehicle_state
    last_rc_send = 0
    last_buzz = 0
    
    while True:
        # 1. READ DATA (Non-blocking)
        msg = connection.recv_match(blocking=False)
        if msg:
            m_type = msg.get_type()
            if m_type == 'HEARTBEAT':
                # Ambil mode, pastikan Uppercase agar matching mudah
                mode = mavutil.mode_string_v10(msg).upper()
                armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED > 0
                with data_lock:
                    vehicle_state["mode"] = mode
                    vehicle_state["armed"] = armed
            elif m_type == 'SYS_STATUS':
                with data_lock:
                    vehicle_state["voltage"] = msg.voltage_battery / 1000.0

        # 2. SEND COMMANDS (10Hz)
        if time.time() - last_rc_send > 0.1:
            # Ambil snapshot data thread-safe
            with data_lock:
                curr_mode = vehicle_state["mode"]
                target_rc = command_state["rc"]
                buzz_active = command_state["buzzer"]

            try:
                # LOGIKA PENGAMBILALIHAN
                if curr_mode == PYTHON_DRIVE_MODE:
                    # Mode MANUAL -> Python Drive
                    connection.mav.rc_channels_override_send(
                        connection.target_system, connection.target_component,
                        target_rc[0], target_rc[1], target_rc[2], target_rc[3], 
                        0, 0, 0, 0
                    )
                    
                    # Buzzer Logic (Hanya bunyi saat Python drive & Object Lost)
                    if buzz_active and (time.time() - last_buzz > 1.5):
                        tune = "L8 O5 c"
                        connection.mav.play_tune_v2_send(
                            connection.target_system, connection.target_component,
                            1, tune.encode('ascii')
                        )
                        last_buzz = time.time()
                else:
                    # Mode ACRO/STABILIZE -> Release control (0)
                    connection.mav.rc_channels_override_send(
                        connection.target_system, connection.target_component,
                        0, 0, 0, 0, 0, 0, 0, 0
                    )
            except Exception:
                pass
            
            last_rc_send = time.time()
        
        time.sleep(0.01) # Prevent CPU hogging

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
        
        # Compress
        ret, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])

        # Get Mode for display
        with data_lock:
            mode_disp = vehicle_state["mode"]

        msg_header = f"Mode:{mode_disp}"

        try:
            reply = sender.send_jpg(msg_header, jpg)
            data = json.loads(reply.decode('utf-8'))
            
            offset = data.get("offset", 0)
            status = data.get("status", "WAITING")
            
            print(f"FC: {mode_disp} | Vision: {status} | Off: {offset}")

            # Update Logic (Thread Mavlink yang akan eksekusi)
            if status == "TRACKING":
                if offset < -20:
                    update_cmd(RC_KIRI, False)
                elif offset > 20:
                    update_cmd(RC_KANAN, False)
                else:
                    update_cmd(RC_LURUS, False)
            else:
                # Lost/Waiting -> Stop & Bunyikan Buzzer
                update_cmd(RC_NEUTRAL, True)

        except Exception as e:
            print(f"Error: {e}")

finally:
    print("Cleanup...")
    connection.close()
    vs.stop()