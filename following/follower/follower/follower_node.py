#!/usr/bin/env python3
"""
A ROS2 node used to control a differential drive robot with a camera,
so it follows the line in a Robotrace style track.
You may change the parameters to your liking.
"""
__author__ = "Navaneeth"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from std_msgs.msg import Int16MultiArray,Int16,Float64MultiArray
import numpy as np
import cv2
import cv_bridge
import os
from matplotlib import pyplot as plt, cm, colors
from std_msgs.msg import Bool
from pyzbar import pyzbar
import cv2.aruco as aruco
import time
bridge = cv_bridge.CvBridge()

## User-defined parameters: (Update these values to your liking)
# Minimum size for a contour to be considered anything
MIN_AREA = 500 

# Minimum size for a contour to be considered part of the track
MIN_AREA_TRACK = 5000

# Robot's speed when following the line
LINEAR_SPEED = 0.4


PID_TUNING = 0
slider_KP = 0 
slider_KD = 0
slider_KI = 0
slider_KPL = 0



# Proportional constant to be applied on speed when turning 
# (Multiplied by the error value)
P_Value = 0.0
I_Value = 0.0
D_Value = 0.0
robot_state = 0
zone = [0,0]
KP = 0.8
KD = 0.8
KI = 0.3
KPL = 1.0
PreviousError = 0
integral = 0


# adjustable kp 
KPSmall = 0.2
KPlarge = 0.5
error_range = 50


counter = 0
#KI = 0.02
#KD = 0.0

# If the line is completely lost, the error value shall be compensated by:
LOSS_FACTOR = 1.2

# Send messages every $TIMER_PERIOD seconds
TIMER_PERIOD = 0.1

# When about to end the track, move for ~$FINALIZATION_PERIOD more seconds
FINALIZATION_PERIOD = 4

# The maximum error value for which the robot is still in a straight line
MAX_ERROR = 30

# out of line timer in sec
line_timer = 3 # used to set the delay to stop , if the line not present
start_time = 0
timer_reset_counter = 0

# black out
black_timer_reset_counter = 0
black_timer = 2 # adjust the black time
black_start_time = 0
Previousz = 0
Previousx = 0

left_fit = 0
line_percentage = 0
frame_skip = 0
frame_skip_count = 0
# BGR values to filter only the selected color range
lower_bgr_values = np.array([31,  42,  53])
upper_bgr_values = np.array([255, 255, 255])

def crop_size(height, width):
    """
    Get the measures to crop the image
    Output:
    (Height_upper_boundary, Height_lower_boundary,
     Width_left_boundary, Width_right_boundary)
    """
    ## Update these values to your liking.

    return (1*height//3, height, width//4, 3*width//4)


# Global vars. initial values
image_input = 0
error = 0
just_seen_line = False
just_seen_right_mark = False
should_move = False
right_mark_count = 0
finalization_countdown = None


'''
Lane following using vision starts here
'''
def abs_sobel_thresh(image, orient='x', sobel_kernel=3, thresh=(0, 255)):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    # isX = True if orient == 'x' else False
    if orient == 'x':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 1, 0))
    if orient == 'y':
        abs_sobel = np.absolute(cv2.Sobel(gray, cv2.CV_64F, 0, 1))
    # sobel = cv2.Sobel(gray, cv2.CV_64F, isX, not isX)
    # abs_sobel = np.absolute(sobel)
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel)) 
    grad_binary = np.zeros_like(scaled_sobel)
    grad_binary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1
   
    return grad_binary

def mag_thresh(image, sobel_kernel=3, mag_thresh=(0, 255)):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    abs_sobel = np.sqrt(sobelx**2 + sobely**2)
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel)) 
    mag_binary = np.zeros_like(scaled_sobel)
    mag_binary[(scaled_sobel >= mag_thresh[0]) & (scaled_sobel <= mag_thresh[1])] = 1

    return mag_binary

def dir_threshold(image, sobel_kernel=3, thresh=(0, np.pi/2)):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=sobel_kernel)
    abs_sobelx = np.absolute(sobelx)
    abs_sobely = np.absolute(sobely)
    grad_dir = np.arctan2(abs_sobely, abs_sobelx)
    dir_binary = np.zeros_like(grad_dir)
    dir_binary[(grad_dir >= thresh[0]) & (grad_dir <= thresh[1])] = 1

    return dir_binary

def apply_thresholds(image, ksize=3):
    gradx = abs_sobel_thresh(image, orient='x', sobel_kernel=ksize, thresh=(10, 100))
    grady = abs_sobel_thresh(image, orient='y', sobel_kernel=ksize, thresh=(10, 100))
    mag_binary = mag_thresh(image, sobel_kernel=ksize, mag_thresh=(10, 100))
    dir_binary = dir_threshold(image, sobel_kernel=ksize, thresh=(0.14, 0.4))

    combined = np.zeros_like(dir_binary)
    combined[((gradx == 1) & (grady == 1)) | ((mag_binary == 1) & (dir_binary == 1))] = 1
    
    return combined

def apply_color_threshold(image):
    
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = np.array([27, 46, 65])
    upper_white = np.array([33, 255, 233])
    mask = cv2.inRange(hls, lower_white, upper_white)
    hls_result = cv2.bitwise_and(image, image, mask = mask)

    return hls_result

def compare_images(image1, image2, image1_exp="Image 1", image2_exp="Image 2"):
    f, (ax1, ax2) = plt.subplots(1, 2, figsize=(24, 9))
    f.tight_layout()
    ax1.imshow(image1)
    ax1.set_title(image1_exp, fontsize=50)
    ax2.imshow(image2)
    ax2.set_title(image2_exp, fontsize=50)
    plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)

def color_transform_hsv(image):
    
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # H_channel = hls[:,:,0]
    # L_channel = hls[:,:,1]
    # s_channel = hls[:,:,2]

    global robot_state
    if robot_state == 3:     # right purple color
        lower_hls = np.array([126,0, 0])
        upper_hls = np.array([154, 255,255]) 
    else:                                             # green color
        lower_hls = np.array([41,73, 57])
        upper_hls = np.array([90, 255,225])
    if robot_state == 2:                      #left brown color
        lower_hls = np.array([1,0, 0])   # Define lower and upper HSL values for masking
        upper_hls = np.array([21, 255,230])

    # print(lower_hls,upper_hls)
    # print(type(robot_state))
    
    # Create the HLS mask
    mask = cv2.inRange(hls, lower_hls, upper_hls)

    # Apply the mask to the original image
    hls_result = cv2.bitwise_and(hls, hls, mask=mask)
   
    gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 70 , 255, cv2.THRESH_BINARY)
    blur = cv2.GaussianBlur(thresh,(3, 3), 0)
    canny = cv2.Canny(blur, 40, 60)


    return hls_result,gray, thresh, blur, canny


def combine_threshold2(s_binary, combined):
    combined_binary = np.zeros_like(combined)
    combined_binary[(s_binary == 1) | (combined == 1)] = 1

    return combined_binary

def combine_threshold1(s_binary, combined):
    combined_binary = np.zeros_like(combined)
    combined_binary[(s_binary == 255) | (combined == 255)] = 255

    return combined_binary


def perspectiveWarp(inpImage):
    # print(inpImage.shape)
    # Get image size
    img_size = (inpImage.shape[1], inpImage.shape[0])

    # Perspective points to be warped
    src = np.float32([[261, 30],[783, 30],[12, 755],[1014, 755] ])

    # Window to be shown
    dst = np.float32([[0, 30],
                      [1014, 30],
                      [0, 755],
                      [1014, 755]])

    # Matrix to warp the image for birdseye window
    matrix = cv2.getPerspectiveTransform(src, dst)
    # Inverse matrix to unwarp the image for final window
    minv = cv2.getPerspectiveTransform(dst, src)
    birdseye = cv2.warpPerspective(inpImage, matrix, img_size)

    # Get the birdseye window dimensions
    height, width = birdseye.shape[:2]

    # Divide the birdseye view into 2 halves to separate left & right lanes
    birdseyeLeft  = birdseye[0:height, 0:width // 2]
    birdseyeRight = birdseye[0:height, width // 2:width]

    return birdseye, birdseyeLeft, birdseyeRight, minv

def get_histogram(binary_warped):
    histogram = np.sum(binary_warped[binary_warped.shape[0]//4:,:], axis=0)
    
    return histogram

def slide_window(binary_warped, histogram):
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    midpoint = np.int32(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint
    # print(leftx_base,rightx_base)
    nwindows = 10
    window_height = np.int32(binary_warped.shape[0]/nwindows)
    nonzero = binary_warped.nonzero() # split non zero row and non zero colom and the row idex will be in 1st element and nonzero colomn index in second element
    # print(nonzero)
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # print(nonzerox.shape,nonzeroy.shape)
    # print(binary_warped.shape[0])
    
    leftx_current = leftx_base
    rightx_current = rightx_base
    margin = 100 # adjust as per the line 
    minpix = 50
    left_lane_inds = []
    right_lane_inds = []
    
    line_count = 0
    line_threshold = 500
    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        # win_xright_low = rightx_current - margin
        # win_xright_high = rightx_current + margin
        win_xleft_low_ = win_xleft_low
        if win_xleft_low < 0:
            win_xleft_low_ = 0
        cropped_img = binary_warped[win_y_low:win_y_high, win_xleft_low_:win_xleft_high]
        non_zero_line = np.count_nonzero(cropped_img)
        if non_zero_line > line_threshold and window > 5:
            line_count = line_count + 1
        # print(non_zero_line)
        # print((win_xleft_low,win_y_low),(win_xleft_high,win_y_high))
        cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),
        (0,255,0), 2) 
        # cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),
        # (0,255,0), 2) 
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        # good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        # (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        # print(len(good_left_inds))
        left_lane_inds.append(good_left_inds)
        # right_lane_inds.append(good_right_inds)
        if len(good_left_inds) > minpix:
            leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
            # print(leftx_current)
        # if len(good_right_inds) > minpix:        
        #     rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))
    line_percentage = line_count/(4)
    sliding_percentage = line_count/nwindows
    # print("sliding percentage",sliding_percentage)
    

    left_lane_inds = np.concatenate(left_lane_inds)
    # right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    # print(leftx)
    # rightx = nonzerox[right_lane_inds]
    # righty = nonzeroy[right_lane_inds] 

    left_fit = np.polyfit(lefty, leftx, 2)
    # right_fit = np.polyfit(righty, rightx, 2)

    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    # right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    # print(len(left_fitx),len(ploty))
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    # out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    
    # plt.imshow(out_img)
    # plt.plot(left_fitx, ploty, color='yellow')
    # # plt.plot(right_fitx, ploty, color='yellow')
    # plt.xlim(0, 1280)
    # plt.ylim(720, 0)
    
    # return ploty, left_fit, right_fit
    return ploty, left_fit,out_img, line_percentage


def skip_sliding_window(binary_warped, left_fit):
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 100
    left_lane_inds = ((nonzerox > (left_fit[0]*(nonzeroy**2) + left_fit[1]*nonzeroy + 
    left_fit[2] - margin)) & (nonzerox < (left_fit[0]*(nonzeroy**2) + 
    left_fit[1]*nonzeroy + left_fit[2] + margin))) 

    # right_lane_inds = ((nonzerox > (right_fit[0]*(nonzeroy**2) + right_fit[1]*nonzeroy + 
    # right_fit[2] - margin)) & (nonzerox < (right_fit[0]*(nonzeroy**2) + 
    # right_fit[1]*nonzeroy + right_fit[2] + margin)))  

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    # rightx = nonzerox[right_lane_inds]
    # righty = nonzeroy[right_lane_inds]
    left_fit = np.polyfit(lefty, leftx, 2)
    # right_fit = np.polyfit(righty, rightx, 2)
    ploty = np.linspace(0, binary_warped.shape[0]-1, binary_warped.shape[0] )
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    # right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
  
    
    ################################ 
    ## Visualization
    ################################ 
    
    out_img = np.dstack((binary_warped, binary_warped, binary_warped))*255
    window_img = np.zeros_like(out_img)
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    # out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx-margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx+margin, 
                                  ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    # right_line_window1 = np.array([np.transpose(np.vstack([right_fitx-margin, ploty]))])
    # right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx+margin, 
    #                               ploty])))])
    # right_line_pts = np.hstack((right_line_window1, right_line_window2))

    cv2.fillPoly(window_img, np.int_([left_line_pts]), (0,255, 0))
    # cv2.fillPoly(window_img, np.int_([right_line_pts]), (0,255, 0))
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)
    
    # plt.imshow(result)
    # plt.plot(left_fitx, ploty, color='yellow')
    # # plt.plot(right_fitx, ploty, color='yellow')
    # plt.xlim(0, 1920)
    # plt.ylim(1080, 0)
    
    ret = {}
    ret['leftx'] = leftx
    # ret['rightx'] = rightx
    ret['left_fitx'] = left_fitx
    # ret['right_fitx'] = right_fitx
    ret['ploty'] = ploty
    
    return ret,result

def measure_curvature(ploty, lines_info):
    ym_per_pix = 1/780 
    xm_per_pix = 0.5/1024 

    leftx = lines_info['left_fitx']
    # rightx = lines_info['right_fitx']

    leftx = leftx[::-1]  
    # rightx = rightx[::-1]  

    y_eval = np.max(ploty)
    left_fit_cr = np.polyfit(ploty*ym_per_pix, leftx*xm_per_pix, 2)
    # right_fit_cr = np.polyfit(ploty*ym_per_pix, rightx*xm_per_pix, 2)
    left_curverad = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
    # right_curverad = ((1 + (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5) / np.absolute(2*right_fit_cr[0])
    # print(left_curverad, 'm', right_curverad, 'm')
    # print(left_curverad, 'm')
    return left_curverad


def draw_lane_lines(original_image, warped_image, Minv, draw_info):

    leftx = draw_info['leftx']
    # rightx = draw_info['rightx']
    left_fitx = draw_info['left_fitx']
    # right_fitx = draw_info['right_fitx']
    ploty = draw_info['ploty']
    canvas = np.zeros((original_image.shape[0], original_image.shape[1], 3), dtype=np.uint8)

    warp_zero = np.zeros_like(original_image).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    # pts_left = np.array(np.vstack([left_fitx, ploty]))
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    
    # pts_left = pts_left.astype(int)
    # print (pts_left[0])
    # pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack(pts_left)
    points = pts_left.reshape((-1, 1, 2))
    # mean_x = np.mean((left_fitx, right_fitx), axis=0)
    # pts_mean = np.array([np.flipud(np.transpose(np.vstack([mean_x, ploty])))])

    # cv2.fillPoly(canvas, np.int_([points]), (0, 255, 0))
    height, width = original_image.shape[:2]

    # Draw a vertical line in the middle
    line_color = (0, 0, 255)  # Red color (BGR format)
    line_thickness = 2
    line_start = (width // 2, 0)  # Starting point at the top
    line_end = (width // 2, height)  # Ending point at the bottom
    cv2.line(original_image, line_start, line_end, line_color, line_thickness)

    # Draw a point at the center of the image
    point_color = (0, 255, 0)  # Green color (BGR format)
    point_radius = 4
    point_center = (width // 2, height // 2)  # Center point
    cv2.circle(original_image, point_center, point_radius, point_color, -1)
   
    # print(points[height // 2][0][0])
    error =int( points[height // 2][0][0] - width // 2)

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_color = (0, 0, 255)  # White color (BGR format)
    cv2.putText(original_image, f"Error: {error}", (10, 30), font, font_scale, font_color, 4)

    
    error_points = np.int_(points[height // 3][0])
    cv2.circle(original_image, (error_points[0],error_points[1]) , point_radius, (0,0,255), -1)
    #cv2.circle(original_image, np.int_(points[height // 2][0]) , point_radius, (0,0,255), -1)


    cv2.polylines(canvas, np.int_([points]), isClosed=False, color=(255, 0, 0), thickness=4)
    
    # print(error)
    # cv2.fillPoly(color_warp, np.int_([pts_mean]), (0, 255, 255))

    # newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
    result = cv2.addWeighted(original_image, 1, canvas, 0.3, 0)

    return result,error


def start_follower_callback(request, response):
    """
    Start the robot.
    In other words, allow it to move (again)
    """
    global should_move
    global right_mark_count
    global finalization_countdown
    print("got data")
    should_move = True
    right_mark_count = 0
    finalization_countdown = None
    return response

def stop_follower_callback(request, response):
    """
    Stop the robot
    """
    global should_move
    global finalization_countdown
    should_move = False
    finalization_countdown = None
    return response

def image_callback(msg):
    """
    Function to be called whenever a new Image message arrives.
    Update the global variable 'image_input'
    """
    # global image_input
    
    # node.get_logger().info('Received image')

    global zone
    global error
    global image_input
    global just_seen_line
    global just_seen_right_mark
    global should_move
    global right_mark_count
    global finalization_countdown
    global cap
    global robot_state
    global counter
    global slider_KPL,slider_KP,slider_KD,slider_KI
    global KP,KD,KI,KPL
    global integral
    # global zone
    global PreviousError
    global line_timer,timer_reset_counter,start_time
    global left_fit,line_percentage,frame_skip,frame_skip_count
    global Previousx,Previousz,black_start_time,black_timer,black_timer_reset_counter
    global KPSmall,KPlarge,error_range 
    #global KD
    image_input = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    counter = counter + 1
    
    # print("c ", counter)
    ids = []
    try:
        # Read a frame from the video
        # ret, frame = cap.read()
        # ret, image = cap.read()
        # frame = resize_image(image, 864)
        frame = image_input
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        hls_result,gray, thresh, blur, canny = color_transform_hsv(frame)
        


        
        combined_binary = combine_threshold1(canny,thresh)
        # non_zero_image = np.count_nonzero(combined_binary)
        # print(non_zero_image)
        birdseye, birdseyeLeft, birdseyeRight, minv = perspectiveWarp(combined_binary)
        histogram = get_histogram(combined_binary)
        frame_skip = frame_skip + 1
        # if frame_skip > frame_skip_count :
        #cv2.imshow("Original",hls_result)
            # print("detection")
        ploty, left_fit,sliding_image,line_percentage = slide_window(combined_binary, histogram)
        frame_skip = 0
       
            
        
        draw_info,result_image = skip_sliding_window(combined_binary, left_fit)
        final_image,error = draw_lane_lines(frame,birdseye,minv,draw_info)
        # print("line percentage",line_percentage)
        # resized_image = cv2.resize(final_image, (800, 500))

 
        
        '''
        Display image
        '''
        # plt.plot(histogram)
        # plt.show()
        # if cv2.waitKey(25) & 0xFF == ord('q'):
        #     break
        # compare_images(image, combined, "Original Image", "Gradient Thresholds")
        # compare_images(frame,RGB_img)
        cv2.imshow("Original",hls_result)
        # cv2.imshow("canny", canny)
        # cv2.imshow("thresh",thresh)
        #cv2.imshow("combined_method1",combined_binary)
        #cv2.imshow("wrapped",birdseye)
        #cv2.imshow("sliding_window",sliding_image)
        cv2.imshow("result",final_image)
        # cv2.imshow("final_image",resized_image)
        # cv2.imwrite("test_image.png",frame)
        # video_writer.write(resized_image)


        cv2.waitKey(1)
        if PID_TUNING:
            # print("KP",slider_KP,"KD",slider_KD,"KI",slider_KI,"KPL",slider_KPL)
            KPL = slider_KPL
            KP = slider_KP
            KD = slider_KD
            KI = slider_KI

        '''
        # print("KP",KP,"KD",KD,"KI",KI,"KPL",KPL)

        error_ = float(error/1000.0)

        # Calculate the proportional term
        proportional = error_

        # Calculate the integral term
        integral += error_

        # Calculate the derivative term
        derivative = error_ - PreviousError

        # Calculate the PID control signal
        pid_output = KP * proportional + KI * integral + KD * derivative
        pid_output_ = pid_output * -1
        
        if PID_TUNING :
            pid_msg = Float64MultiArray()
            pid_msg.data.append(error_ *100)
            pid_msg.data.append(KP * proportional * 100)
            pid_msg.data.append(KD * derivative *100)
            pid_msg.data.append(KI * integral * 100)
            pid_msg.data.append(pid_output_ * 100)
            pid_publisher.publish(pid_msg)
        ''' 
        # print("error",error_ *1000,"derivative",derivative*1000,"pid_output ",pid_output)
        error_ = float(error/1000.0)
        if error_ < error_range and error > (-1 * error_range):
            output = error_ * KPSmall * -1
            print(output)
        else:
            output = error_ * KPlarge * -1
        
        

        # Store the previous error for the next iteration
        PreviousError = error_


        message = Twist()
        lin_speed = float(error/1000.0) * KPL
        
        if lin_speed >= 0:
            lin_speed_actual = LINEAR_SPEED - lin_speed
        if lin_speed < 0 :
            lin_speed_actual = LINEAR_SPEED + lin_speed
        if lin_speed_actual > LINEAR_SPEED :
            lin_speed_actual = LINEAR_SPEED
        if lin_speed_actual < 0:
            lin_speed_actual = 0.0
        # print("Linear speed",lin_speed_actual)
        message.linear.x = lin_speed_actual
        # message.linear.x = LINEAR_SPEED
        # Determine the speed to turn and get the line in the center of the camera.
        #if robot_state == 3:
            #KP_ = 0.663
        #else:
            #KP_ = KP
        # print(KP)
        P_Value = float(error/1000.0) * -KP 
        #- KD *(float(error/1000.0) - PreviousError)
        #print("f",(float(error/1000.0)),"P", PreviousError,"PV", P_Value,"FP", ((float(error/1000.0) - PreviousError)))
        #PreviousError = float(error/1000.0)
        
        # max error 400
        # if -100 < error < 100:
        #     global I_Value
        #     I_Value = I_Value + (KI*float(error/1000.0))


        # message.angular.z = P_Value + I_Value
        # message.angular.z = pid_output_
        message.angular.z = output
        print("Error: {:.4f} | Linear X: {:.4f} | Angular Z: {:.4f}, ".format(error_,message.linear.x , message.angular.z))
        
        #line_percentage = True
        # Publish the message to 'cmd_vel'
        # if line_percentage > 0.05 and robot_state:
        # print ("robot_state",robot_state)
        # print ("zone: ",zone)
        # print (robot_state)
        # pri
        
        line_stop = False
        if line_percentage < 1.0 :
             
            timer_reset_counter = timer_reset_counter + 1
            if timer_reset_counter == 1:
                start_time = time.time()
            else:
                current_time = time.time()
                if current_time - start_time >= line_timer:
                    line_stop = True
        else:
            line_stop = False
            timer_reset_counter = 0

        # print ("line percentage ", line_percentage,"line stop",line_stop)


            
        # print("robot_state ",robot_state,"zone ",zone[0],"line stop ",line_stop )
        # if robot_state and not zone[0] and not line_stop:
        
        if robot_state and not zone[0]:
            # print("rs", robot_state)
            publisher.publish(message)
           
        else:
            empty_message = Twist()
            publisher.publish(empty_message)
        black_timer_reset_counter = 0
        Previousx = lin_speed_actual
        # Previousz = pid_output_
    except Exception as e:
        print(e)
        # black_timer_reset_counter = black_timer_reset_counter + 1
        # if black_timer_reset_counter == 1:
        #         black_start_time = time.time()
        # else:
        #     black_current_time = time.time()
        #     if black_current_time - black_start_time >= black_timer:
        message = Twist()
        message.linear.x = 0.0
        message.angular.z = 0.0
        publisher.publish(message)
            # else:
            #     message.linear.x = Previousx
            #     message.angular.z = Previousz
                

            
        
        




def robot_state_callback(msg):
    # a =5
    global robot_state
    robot_state = msg.data
    # print("message data",msg)



def zone_callback(msg):
    #  a = 5
    global zone
    zone = msg.data
    # print("message data",zone)


def resize_image(image, width):
    # (1944, 2592, 3)
    # Get the original image dimensions
    orig_height, orig_width = image.shape[:2]
    # print(orig_height,orig_width)

    # Calculate the aspect ratio
    aspect_ratio = width / float(orig_width)

    # Calculate the new height based on the aspect ratio
    height = int(orig_height * aspect_ratio)

    # Resize the image
    resized_image = cv2.resize(image, (width, height))

    return resized_image

def timer_callback():
    """
    Function to be called when the timer ticks.
    According to an image 'image_input', determine the speed of the robot
    so it can follow the contour
    """
    global zone
    global error
    global image_input
    global just_seen_line
    global just_seen_right_mark
    global should_move
    global right_mark_count
    global finalization_countdown
    global cap
    global robot_state
    global counter
    #global PreviousError
    #global KD
    
    counter = counter + 1
    
    # print("c ", counter)
    ids = []
    try:
        # Read a frame from the video
        # ret, frame = cap.read()
        ret, image = cap.read()
        frame = resize_image(image, 864)
        gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        hls_result,gray, thresh, blur, canny = color_transform_hsv(frame)
        


        
        combined_binary = combine_threshold1(canny,thresh)
        birdseye, birdseyeLeft, birdseyeRight, minv = perspectiveWarp(combined_binary)
        histogram = get_histogram(combined_binary)
        
        #cv2.imshow("Original",hls_result)
        ploty, left_fit,sliding_image,line_percentage = slide_window(combined_binary, histogram)
            
        
        draw_info,result_image = skip_sliding_window(combined_binary, left_fit)
        final_image,error = draw_lane_lines(frame,birdseye,minv,draw_info)
        print("line percentage",line_percentage)
        # resized_image = cv2.resize(final_image, (800, 500))

        # Detect QR codes in the frame
        # Convert the frame to grayscale
        

        # Define the dictionary of ArUco markers
       # aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

        # Define the parameters for marker detection
       # parameters = aruco.DetectorParameters_create()

        # Detect ArUco markers
        #corners, ids, rejected = aruco.detectMarkers(gray_image, aruco_dict, parameters=parameters)
        
        
        # print(ids[0][0])

        # Draw detected markers on the frame
        #frame = aruco.drawDetectedMarkers(frame, corners, ids)

        # Display the resulting frame
        # cv2.imshow('ArUco Marker Detection', frame)
        #msg = Bool()
        #if ids:
        #    if ids[0][0] == 10 :
        #        msg.data = True
         #       stop_publisher.publish(msg)
        #else:
         #   msg.data = False
        #    stop_publisher.publish(msg)

        
        '''
        Display image
        '''
        # plt.plot(histogram)

        # if cv2.waitKey(25) & 0xFF == ord('q'):
        #     break
        # compare_images(image, combined, "Original Image", "Gradient Thresholds")
        # compare_images(frame,RGB_img)
        cv2.imshow("Original",hls_result)
        # cv2.imshow("canny", canny)
        # cv2.imshow("thresh",thresh)
        #cv2.imshow("combined_method1",combined_binary)
        #cv2.imshow("wrapped",birdseye)
        # cv2.imshow("sliding_window",sliding_image)
        cv2.imshow("result",final_image)
        # cv2.imshow("final_image",resized_image)
        # cv2.imwrite("test_image.png",frame)
        # video_writer.write(resized_image)


        cv2.waitKey(1)

        message = Twist()
        lin_speed = float(error/1000.0) * KPL
        
        if lin_speed >= 0:
            lin_speed_actual = LINEAR_SPEED - lin_speed
        if lin_speed < 0 :
            lin_speed_actual = LINEAR_SPEED + lin_speed
        if lin_speed_actual > LINEAR_SPEED :
            lin_speed_actual = LINEAR_SPEED
        if lin_speed_actual < 0:
            lin_speed_actual = 0.0
        # print("Linear speed",lin_speed_actual)
        message.linear.x = lin_speed_actual
        # message.linear.x = LINEAR_SPEED
        # Determine the speed to turn and get the line in the center of the camera.
        #if robot_state == 3:
            #KP_ = 0.663
        #else:
            #KP_ = KP
        print(KP)
        P_Value = float(error/1000.0) * -KP 
        #- KD *(float(error/1000.0) - PreviousError)
        #print("f",(float(error/1000.0)),"P", PreviousError,"PV", P_Value,"FP", ((float(error/1000.0) - PreviousError)))
        #PreviousError = float(error/1000.0)
        
        # max error 400
        # if -100 < error < 100:
        #     global I_Value
        #     I_Value = I_Value + (KI*float(error/1000.0))


        # message.angular.z = P_Value + I_Value
        message.angular.z = P_Value
        print("Error: {} | Linear X: {} | Angular Z: {}, ".format(error,message.linear.x , message.angular.z))
        
        #line_percentage = True
        # Publish the message to 'cmd_vel'
        # if line_percentage > 0.05 and robot_state:
        print ("robot_state",robot_state)
        print ("zone: ",zone)
        if robot_state and not zone[0] :
            # print("rs", robot_state)
            publisher.publish(message)
           
        else:
            empty_message = Twist()
            publisher.publish(empty_message)
    except Exception as e:
        print(e)
        message = Twist()
        message.linear.x = 0.0
        message.angular.z = 0.0
        publisher.publish(message)
        
def angular_kp_callback(value):
    global slider_KP
    slider_KP = value/10.0
def angular_ki_callback(value):
    global slider_KI
    slider_KI = value/10.0
def angular_kd_callback(value):
    global slider_KD
    slider_KD = value/10.0
def linera_kp_callback(value):
    global slider_KPL
    slider_KPL = value/10.0



def main():
    rclpy.init()
    global node
    node = Node('follower')

    global publisher,cap,stop_publisher,pid_publisher
    if PID_TUNING :
        cv2.namedWindow("Image_output")
        cv2.createTrackbar('angular_KP', 'Image_output', 0, 255, angular_kp_callback)
        cv2.createTrackbar('angular_KD', 'Image_output', 0, 255, angular_kd_callback)
        cv2.createTrackbar('angular_Ki', 'Image_output', 0, 255, angular_ki_callback)
        cv2.createTrackbar('linear_kp', 'Image_output', 0, 255, linera_kp_callback)
        
    publisher = node.create_publisher(Twist, 'nav_vel', rclpy.qos.qos_profile_system_default)
    pid_publisher = node.create_publisher(Float64MultiArray, 'pid', rclpy.qos.qos_profile_system_default)
    #stop_publisher = node.create_publisher(Bool,'/stop_robot',10)
    subscription = node.create_subscription(Image, 'undistorted_image',
                                            image_callback,undistorted_imageundistorted_image,
                                            rclpy.qos.qos_profile_sensor_data)
    plc_subscription = node.create_subscription(Int16, 'robot_state_topic', robot_state_callback, 10)
    rfid_subscriber = node.create_subscription(Int16MultiArray, 'zone', zone_callback, 10)

    # timer = node.create_timer(TIMER_PERIOD, timer_callback)
    # cap = cv2.VideoCapture(0)

    # Check if the video file was successfully opened
    # if not cap.isOpened():
    #     print("Error opening video file")
    #     exit()
    start_service = node.create_service(Empty, 'start_follower', start_follower_callback)
    stop_service = node.create_service(Empty, 'stop_follower', stop_follower_callback)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

try:
    main()
except (KeyboardInterrupt, rclpy.exceptions.ROSInterruptException):
    empty_message = Twist()
    #empty_bool = Bool()
    #empty_bool.data = False
    publisher.publish(empty_message)
    #stop_publisher.publish(empty_bool)

    node.destroy_node()
    rclpy.shutdown()
    exit()
