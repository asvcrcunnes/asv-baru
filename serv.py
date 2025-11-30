import cv2
import imagezmq
import json
import numpy as np
from ultralytics import YOLO

# --- CONFIGURATION ---
model = YOLO('model.pt') 

# UPDATE THESE IDs TO MATCH YOUR MODEL
# Check your model.names to find out which number is which
CLASS_IDS = {
    'red_ball': 0, 
    'green_ball': 1
}

# Initialize ZMQ
image_hub = imagezmq.ImageHub(open_port='tcp://*:5555')

print("Server Ready: Tracking Red/Green Midpoint...")

while True:
    # 1. Receive Image & Data
    msg_str, frame = image_hub.recv_image()
    
    # Calculate Frame Center (X-axis only)
    frame_height, frame_width = frame.shape[:2]
    frame_center_x = frame_width // 2

    # 2. Run Inference
    results = model(frame, verbose=False)
    
    # 3. Parse Detections
    detected_objects = {} # Store found objects here: {'red_ball': [x,y], 'green_ball': [x,y]}

    for r in results:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            
            # Get the center of this specific bounding box
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # Map ID to Name
            # We iterate through our config to find the name matches
            for name, target_id in CLASS_IDS.items():
                if cls_id == target_id:
                    # If we found a red or green ball, store its center
                    # Note: If multiple exist, this takes the last one found. 
                    # To get the highest confidence, sort results first.
                    detected_objects[name] = (center_x, center_y)

    # 4. The Logic Core
    response = {}
    
    has_red = 'red_ball' in detected_objects
    has_green = 'green_ball' in detected_objects

    if has_red and has_green:
        # Get coordinates
        rx, ry = detected_objects['red_ball']
        gx, gy = detected_objects['green_ball']

        # Calculate the Midpoint between the two objects
        midpoint_x = (rx + gx) / 2
        midpoint_y = (ry + gy) / 2 # We calculate Y but use X for steering

        # Calculate Offset (Steering Value)
        # Result: 
        # 0 = Perfect Center
        # Positive (+) = Object is to the Right
        # Negative (-) = Object is to the Left
        offset_value = int(midpoint_x - frame_center_x)

        response = {
            "status": "TRACKING",
            "offset": offset_value,
            "midpoint": [int(midpoint_x), int(midpoint_y)]
        }
        
        # Visualization (Draw line between balls and a dot at the midpoint)
        cv2.line(frame, (int(rx), int(ry)), (int(gx), int(gy)), (255, 255, 0), 2)
        cv2.circle(frame, (int(midpoint_x), int(midpoint_y)), 5, (0, 0, 255), -1)

    else:
        # Missing one or both
        response = {
            "status": "WARNING",
            "message": "Both objects not detected",
            "offset": 0
        }
        # Visual Warning
        cv2.putText(frame, "WARNING: MISSING OBJECTS", (50, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # 5. Send Feedback
    image_hub.send_reply(json.dumps(response).encode('utf-8'))

    # 6. Display Debug View on Server
    cv2.imshow("Server Logic", frame)
    if cv2.waitKey(1) == ord('q'):
        break