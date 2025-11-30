import cv2
from ultralytics import YOLO

# 1. Load the model
model = YOLO("model.pt")
model.to('cuda')

# 2. Open the webcam (0 is usually the default camera)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Press 'q' to quit.")

while True:
    # 3. Read a frame from the camera
    success, frame = cap.read()
    if not success:
        break

    # 4. Run YOLO inference on the frame
    # stream=True is efficient for video loops (generators)
    results = model(frame, stream=True)

    # 5. Visualize the results on the frame
    # We loop through results (usually just 1 for a single frame)
    for r in results:
        # Plot the detections (boxes, labels) directly onto the frame
        annotated_frame = r.plot()
        
        # Display the frame
        cv2.imshow("YOLO Inference", annotated_frame)

    # 6. Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 7. Release resources
cap.release()
cv2.destroyAllWindows()