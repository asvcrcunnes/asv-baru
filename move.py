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

# --- GLOBAL SHARED VARIABLES (Untuk pertukaran data antar Thread) ---
# Data yang dibaca dari Drone
vehicle_state = {
    "mode": "UNKNOWN",
    "voltage": 0,
    "armed": False
}

# Perintah yang akan dikirim ke Drone
command_state = {
    "rc1": 1500, "rc2": 1500, "rc3": 1000, "rc4": 1500, # Default RC
    "buzzer_active": False # Trigger buzzer
}

# Lock untuk mencegah tabrakan data saat baca/tulis bersamaan
data_lock = threading.Lock()

# --- MAVLINK CONNECTION ---
print("Connecting to vehicle...")
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print("Connected to vehicle!")

# --- FUNGSI THREAD MAVLINK ---
def mavlink_worker():
    """
    Fungsi ini berjalan di thread terpisah.
    Tugasnya: Update data sensor & Kirim perintah RC/Buzzer tanpa mengganggu kamera.
    """
    global vehicle_state, command_state
    
    last_rc_send_time = 0
    last_buzz_time = 0
    
    while True:
        # 1. BACA DATA (Non-blocking)
        # Kita loop pesan yang masuk untuk mencari HEARTBEAT atau SYS_STATUS
        msg = connection.recv_match(blocking=False)
        
        if msg:
            msg_type = msg.get_type()
            
            if msg_type == 'HEARTBEAT':
                # Ambil Mode dan Status Armed
                current_mode = mavutil.mode_string_v10(msg)
                is_armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED > 0
                
                with data_lock:
                    vehicle_state["mode"] = current_mode
                    vehicle_state["armed"] = is_armed
                    
            elif msg_type == 'SYS_STATUS':
                # Ambil Voltase Baterai
                voltage = msg.voltage_battery / 1000.0
                with data_lock:
                    vehicle_state["voltage"] = voltage

        # 2. KIRIM RC OVERRIDE (Setiap 0.1 detik / 10Hz agar responsif tapi tidak spam)
        if time.time() - last_rc_send_time > 0.1:
            with data_lock:
                r1, r2, r3, r4 = command_state["rc1"], command_state["rc2"], command_state["rc3"], command_state["rc4"]
                buzz_needed = command_state["buzzer_active"]
            
            try:
                connection.mav.rc_channels_override_send(
                    connection.target_system, connection.target_component,
                    r1, r2, r3, r4, 0, 0, 0, 0
                )
            except:
                pass
            last_rc_send_time = time.time()

            # 3. KIRIM BUZZER (Jika diminta oleh Main Thread)
            if buzz_needed and (time.time() - last_buzz_time > 1.5):
                tune = "L8 O5 c"
                try:
                    connection.mav.play_tune_v2_send(
                        connection.target_system, connection.target_component,
                        1, tune.encode('ascii')
                    )
                    last_buzz_time = time.time()
                except:
                    pass
        
        # Tidur sebentar agar CPU tidak 100%
        time.sleep(0.01)

# --- START MAVLINK THREAD ---
print("Starting MAVLink Thread...")
mav_thread = threading.Thread(target=mavlink_worker)
mav_thread.daemon = True # Thread akan mati otomatis jika program utama stop
mav_thread.start()

# --- DEFINISI NILAI RC ---
rc_defaults = [1500, 1500, 1000, 1500] # RC1, RC2, RC3, RC4
# lurus
rc_lurus = [1500, 1500, 1200, 1500] 
# kiri sitik (RC1 berubah)
rc_kiri = [1300, 1500, 1200, 1500]
# kanan sitik (RC1 berubah)
rc_kanan = [1800, 1500, 1200, 1500]

def update_rc_command(rc_list, buzzer_on=False):
    """Update nilai global agar Thread Mavlink mengirimnya"""
    with data_lock:
        command_state["rc1"] = rc_list[0]
        command_state["rc2"] = rc_list[1]
        command_state["rc3"] = rc_list[2]
        command_state["rc4"] = rc_list[3]
        command_state["buzzer_active"] = buzzer_on

# --- MAIN CAMERA LOOP ---
print("Starting Camera Thread...")
vs = VideoStream(src=0).start()
time.sleep(2.0)

sender = imagezmq.ImageSender(connect_to=f'tcp://{SERVER_IP}:5555')
print(f"Client sending to {SERVER_IP}...")

try:
    while True:
        # 1. Ambil & Proses Gambar
        frame = vs.read()
        frame = cv2.resize(frame, (640, 640))
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        ret, jpg_buffer = cv2.imencode('.jpg', frame, encode_param)

        # 2. Ambil Mode Kendaraan dari Thread sebelah (Non-Blocking)
        # Data ini sudah diupdate otomatis oleh mavlink_worker di background
        current_mode_display = "UNKNOWN"
        with data_lock:
            current_mode_display = vehicle_state["mode"]

        # Kita kirim Mode juga ke server PC (opsional, disisipkan di string message)
        # Format msg: "Mode:MANUAL"
        msg_header = f"Mode:{current_mode_display}"

        try:
            # 3. Kirim Gambar & Terima Balasan
            reply_bytes = sender.send_jpg(msg_header, jpg_buffer)
            feedback = json.loads(reply_bytes.decode('utf-8'))
            
            offset = feedback.get("offset", 0)
            status = feedback.get("status", "WAITING")
            
            # Print Mode untuk debugging di terminal Raspberry Pi
            print(f"Status: {status} | Mode: {current_mode_display} | Offset: {offset}")

            # 4. Logika Kontrol (Hanya mengupdate variabel, pengiriman dilakukan Thread Mavlink)
            if status == "TRACKING":
                if offset < -20:
                    update_rc_command(rc_kiri, buzzer_on=False)
                elif offset > 20:
                    update_rc_command(rc_kanan, buzzer_on=False)
                else: # -20 <= offset <= 20
                    update_rc_command(rc_lurus, buzzer_on=False)
            else:
                # WAITING / SEARCHING -> Bunyikan Buzzer
                update_rc_command(rc_defaults, buzzer_on=True)
                
        except Exception as e:
            print(f"Communication Error: {e}")

finally:
    print("Closing connection...")
    # vs.stop() # dan cleanup lainnya