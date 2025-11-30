import cv2
import imagezmq
import json
import time
from imutils.video import VideoStream

# --- CONFIGURATION ---
SERVER_IP = '192.168.1.100'  # REPLACE with your PC's IP
JPEG_QUALITY = 50            # 0 to 100 (Lower is faster, 50 is good balance)

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
        else:
            print(f"Status: {status}")
            
    except Exception as e:
        print(f"Communication Error: {e}")

    # Optional: Stop gracefully
    # (Note: cv2.imshow isn't needed here if you want max speed, 
    # but useful for debugging if a screen is attached to Pi)
    # cv2.imshow("Pi View", frame)
    # if cv2.waitKey(1) == ord('q'): break