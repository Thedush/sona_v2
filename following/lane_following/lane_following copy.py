import cv2
import numpy as np
import os
from scipy import optimize
from matplotlib import pyplot as plt, cm, colors

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
    lower_white = np.array([84, 0, 0])
    upper_white = np.array([147, 255, 255])
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
    H_channel = hls[:,:,0]
    L_channel = hls[:,:,1]
    s_channel = hls[:,:,2]

     # Define lower and upper HLS values for masking
    lower_hls = np.array([84, 0, 0])
    upper_hls = np.array([147, 255, 255])
    
    # Create the HLS mask
    mask = cv2.inRange(hls, lower_hls, upper_hls)

    # Apply the mask to the original image
    hls_result = cv2.bitwise_and(hls, hls, mask=mask)
   
    gray = cv2.cvtColor(hls_result, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV)
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
    nwindows = 9
    window_height = np.int32(binary_warped.shape[0]/nwindows)
    nonzero = binary_warped.nonzero() # split non zero row and non zero colom and the row idex will be in 1st element and nonzero colomn index in second element
    # print(nonzero)
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    print(nonzerox.shape,nonzeroy.shape)
    # print(binary_warped.shape[0])
    leftx_current = leftx_base
    rightx_current = rightx_base
    margin = 200 # adjust as per the line 
    minpix = 50
    left_lane_inds = []
    right_lane_inds = []

    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window+1)*window_height
        win_y_high = binary_warped.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        # win_xright_low = rightx_current - margin
        # win_xright_high = rightx_current + margin
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
    print(len(left_fitx),len(ploty))
    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    # out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]
    
    # plt.imshow(out_img)
    # plt.plot(left_fitx, ploty, color='yellow')
    # # plt.plot(right_fitx, ploty, color='yellow')
    # plt.xlim(0, 1280)
    # plt.ylim(720, 0)
    
    # return ploty, left_fit, right_fit
    return ploty, left_fit


def skip_sliding_window(binary_warped, left_fit):
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 300
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
    
    plt.imshow(result)
    plt.plot(left_fitx, ploty, color='yellow')
    # plt.plot(right_fitx, ploty, color='yellow')
    plt.xlim(0, 1280)
    plt.ylim(720, 0)
    
    ret = {}
    ret['leftx'] = leftx
    # ret['rightx'] = rightx
    ret['left_fitx'] = left_fitx
    # ret['right_fitx'] = right_fitx
    ret['ploty'] = ploty
    
    return ret

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
    print(left_curverad, 'm')
    return left_curverad
    



ksize=3
if __name__ == '__main__':

    frame = cv2.imread("2023-06-01-161010.jpg")
    # method 1 for finding color and edge 
    hls_result,gray, thresh, blur, canny = color_transform_hsv(frame)
    
    # print(canny[70])

    # method 2 for finding color and edge
    # Converting BGR color to RGB color format
    RGB_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    combined = apply_thresholds(frame)


    # combined threshold 
    combined_binary = combine_threshold1(canny,thresh)
    birdseye, birdseyeLeft, birdseyeRight, minv = perspectiveWarp(combined_binary)
    histogram = get_histogram(birdseye)
    ploty, left_fit = slide_window(birdseye, histogram)
    draw_info = skip_sliding_window(birdseye, left_fit)
    left_curverad = measure_curvature(ploty, draw_info)
    # plt.plot(histogram)


    # compare_images(image, combined, "Original Image", "Gradient Thresholds")
    # compare_images(frame,RGB_img)
    # cv2.imshow("Original",frame)
    # cv2.imshow("canny", canny)
    # # cv2.imshow("combined_edge",combined)
    # cv2.imshow("combined_method1",combined_binary)
    # cv2.imshow("wrapped",birdseye)


    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    #Displaying image using plt.imshow() method

    
    # hold the window
    # plt.imshow(edges_binary)
    plt.waitforbuttonpress()
    plt.close('all')