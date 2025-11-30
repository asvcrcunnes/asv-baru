import cv2
import imagezmq
import json

# Replace with PC IP
sender = imagezmq.ImageSender(connect_to='tcp://127.0.0.1:5555')

cap = cv2.VideoCapture(0)

print("Pi Client Started...")

while True:
    success, frame = cap.read()
    if not success: break

    # 1. Send Image
    # We send an empty dict "{}" as message because we don't have sensor data to send right now
    reply_bytes = sender.send_image("{}", frame)

    # 2. Parse Feedback
    feedback = json.loads(reply_bytes.decode('utf-8'))
    
    status = feedback.get("status")
    offset = feedback.get("offset")

    # 3. Decision Logic
    if status == "WARNING":
        print("⚠️  WARNING: Waiting for both balls...")
        # Code to stop motors here
        
    elif status == "TRACKING":
        # DEAD ZONE: If value is very small (e.g., -5 to 5), consider it centered
        if abs(offset) < 10: 
            print(f"✅ CENTERED (0)")
        elif offset > 0:
            print(f"➡️  Turn RIGHT (+{offset})")
        else:
            print(f"⬅️  Turn LEFT ({offset})")

    # (Optional) Break loop
    if cv2.waitKey(1) == ord('q'):
        break