import cv2
import numpy as np

def nothing(x):
    pass

# Load the image
image = cv2.imread('following/lane_following/final.jpg')

# Create a window
cv2.namedWindow('HLS Mask')

# Create trackbars for HLS values
cv2.createTrackbar('Hue Min', 'HLS Mask', 0, 179, nothing)
cv2.createTrackbar('Hue Max', 'HLS Mask', 179, 179, nothing)
cv2.createTrackbar('Lightness Min', 'HLS Mask', 0, 255, nothing)
cv2.createTrackbar('Lightness Max', 'HLS Mask', 255, 255, nothing)
cv2.createTrackbar('Saturation Min', 'HLS Mask', 0, 255, nothing)
cv2.createTrackbar('Saturation Max', 'HLS Mask', 255, 255, nothing)

while True:
   
    # Get current trackbar positions
    hue_min = cv2.getTrackbarPos('Hue Min', 'HLS Mask')
    hue_max = cv2.getTrackbarPos('Hue Max', 'HLS Mask')
    lightness_min = cv2.getTrackbarPos('Lightness Min', 'HLS Mask')
    lightness_max = cv2.getTrackbarPos('Lightness Max', 'HLS Mask')
    saturation_min = cv2.getTrackbarPos('Saturation Min', 'HLS Mask')
    saturation_max = cv2.getTrackbarPos('Saturation Max', 'HLS Mask')

   
    # sobel
    '''
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # Apply x or y gradient with the OpenCV Sobel() function
    # and take the absolute value
    # if orient == 'x':
    abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1))
    # if orient == 'y':
    #     abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1))
    # Rescale back to 8 bit integer
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))
    # Create a copy and apply the threshold
    binary_output = np.zeros_like(scaled_sobel)
    # Here I'm using inclusive (>=, <=) thresholds, but exclusive is ok too
    binary_output[(scaled_sobel >= saturation_min) & (scaled_sobel <= saturation_max)] = 1
    '''
    # sobel magnitude
    '''
    sobel_kernel=3
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    abs_sobel = np.sqrt(sobelx**2 + sobely**2)
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel)) 
    mag_binary = np.zeros_like(scaled_sobel)
    mag_binary[(scaled_sobel >= saturation_min) & (scaled_sobel <= saturation_max)] = 1
    '''

    # sobel direction change the saturation_mim to 2000 and saturation_max to 2000,
    '''
    sobel_kernel=3
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    abs_sobelx = np.absolute(sobelx)
    abs_sobely = np.absolute(sobely)
    grad_dir = np.arctan2(abs_sobely, abs_sobelx)
    dir_binary = np.zeros_like(grad_dir)
    dir_binary[(grad_dir >= saturation_min/1000) & (grad_dir <= saturation_max/1000)] = 1
    '''
    
    # color slider 

    # Convert the image to HLS
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    H_channel = hls[:,:,0]
    L_channel = hls[:,:,1]
    s_channel = hls[:,:,2]

     # Define lower and upper HLS values for masking
    lower_hls = np.array([hue_min, saturation_min, lightness_min])
    upper_hls = np.array([hue_max, saturation_max, lightness_max])

    #lower_hls = np.array([6, 13, 130])
    #upper_hls = np.array([51, 255, 255])
    
    # Create the HLS mask
    mask = cv2.inRange(hls, lower_hls, upper_hls)

    # Apply the mask to the original image
    masked_image = cv2.bitwise_and(hls, hls, mask=mask)
    gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, saturation_min, saturation_max, cv2.THRESH_BINARY_INV)
    blur = cv2.GaussianBlur(thresh,(3, 3), 0)
    canny = cv2.Canny(blur, 40, 60)
    masked_image = cv2.resize(masked_image, (554, 500))
    thresh = cv2.resize(thresh, (554, 500))
    # Show the masked image
    cv2.imshow('HLS Mask', masked_image)
    # cv2.imshow('HLS Mask', thresh)
    # cv2.imshow('H channel', H_channel)
    # cv2.imshow('L channel', L_channel)
    # cv2.imshow('S channel', s_channel)
    # cv2.imshow("gray", gray)
    # cv2.imshow("canny", canny)
    # Break the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
