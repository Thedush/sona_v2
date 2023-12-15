import cv2

# Open the camera
camera = cv2.VideoCapture(0)  # 0 represents the default camera

# Check if the camera is opened successfully
if not camera.isOpened():
    print("Failed to open camera")
    exit()

# Capture a frame
ret, frame = camera.read()

# Check if the frame is captured successfully
if not ret:
    print("Failed to capture frame")
    exit()
print(frame.shape)
# Save the frame as an image
cv2.imwrite("final2.jpg", frame)

# Release the camera
camera.release()

# Display a message
print("Photo saved as captured_photo.jpg")

