import cv2
from pyzbar import pyzbar

# Initialize the video capture object
cap = cv2.VideoCapture(0)

while True:
    # Read frames from the camera
    ret, frame = cap.read()

    # Convert the frame to grayscale for better QR code detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect QR codes in the frame
    barcodes = pyzbar.decode(gray)

    for barcode in barcodes:
        # Extract the bounding box location of the barcode
        (x, y, w, h) = barcode.rect

        # Draw a rectangle around the barcode
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Decode the barcode data
        barcode_data = barcode.data.decode("utf-8")
        barcode_type = barcode.type

        # Print the barcode data and type
        print("Barcode Data:", barcode_data)
        print("Barcode Type:", barcode_type)
    image =  cv2.resize(frame, (500, 600))

    # Display the frame
    cv2.imshow("QR Code Reader", image)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close the windows
cap.release()
cv2.destroyAllWindows()
