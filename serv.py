import cv2
import imagezmq
import json
import numpy as np
from ultralytics import YOLO

# --- CONFIGURATION ---
model = YOLO('model.pt') # Load your custom .pt file here

# Update with your specific Class IDs
CLASS_IDS = {
    'red_ball': 0, 
    'green_ball': 1
}

# Initialize Hub
image_hub = imagezmq.ImageHub(open_port='tcp://*:5555')

print("Server Ready: Waiting for 640x640 compressed stream...")

while True:
    # 1. Receive JPEG Buffer (Fix 1)
    # msg is the text (json), jpg_buffer is the compressed image
    msg, jpg_buffer = image_hub.recv_jpg()
    
    # 2. Decode JPEG to OpenCV Image
    frame = cv2.imdecode(np.frombuffer(jpg_buffer, dtype='uint8'), -1)
    
    # Get frame dimensions (Verify it is 640x640)
    rows, cols = frame.shape[:2]
    frame_center_x = cols // 2

    # 3. YOLO Inference
    results = model(frame, verbose=False)
    
    # 4. Logic (Red/Green Midpoint)
    detected_objects = {}
    
    for r in results:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            # Map ID to Name
            for name, target_id in CLASS_IDS.items():
                if cls_id == target_id:
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    center_x = (x1 + x2) / 2
                    center_y = (y1 + y2) / 2
                    detected_objects[name] = (center_x, center_y)

    # 5. Construct Response
    response = {}
    if 'red_ball' in detected_objects and 'green_ball' in detected_objects:
        rx, ry = detected_objects['red_ball']
        gx, gy = detected_objects['green_ball']

        midpoint_x = (rx + gx) / 2
        midpoint_y = (ry + gy) / 2
        
        offset_value = int(midpoint_x - frame_center_x)

        response = {
            "status": "TRACKING",
            "offset": offset_value
        }
        
        # Visuals
        cv2.line(frame, (int(rx), int(ry)), (int(gx), int(gy)), (0, 255, 255), 2)
        cv2.circle(frame, (int(midpoint_x), int(midpoint_y)), 8, (0, 0, 255), -1)
    else:
        response = {
            "status": "WARNING",
            "offset": 0
        }
        cv2.putText(frame, "MISSING OBJECTS", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

    # 6. Send Reply
    image_hub.send_reply(json.dumps(response).encode('utf-8'))

    # 7. Display
    cv2.imshow("Server Logic", frame)
    if cv2.waitKey(1) == ord('q'):
        break