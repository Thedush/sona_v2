import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class HSVAdjustmentNode(Node):
    def __init__(self):
        super().__init__('hsv_adjustment_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.subscription  # prevent unused variable warning

        # Initialize OpenCV window and trackbars for HSV adjustments
        cv2.namedWindow("Image")
        cv2.namedWindow("Image_output")
        cv2.createTrackbar('Hue Min', 'Image', 0, 255, self.lower_h_callback)
        cv2.createTrackbar('Hue Max', 'Image', 179, 255, self.upper_h_callback)
        cv2.createTrackbar('Saturation Min', 'Image', 0, 255, self.lower_s_callback)
        cv2.createTrackbar('Saturation Max', 'Image', 255, 255, self.upper_s_callback)
        cv2.createTrackbar('value Min', 'Image', 0, 255, self.lower_v_callback)
        cv2.createTrackbar('value Max', 'Image', 255, 255, self.upper_v_callback)
        cv2.createTrackbar('Threshold Min', 'Image', 0, 255, self.lower_t_callback)
        cv2.createTrackbar('Threshold Max', 'Image', 255, 255, self.upper_t_callback)
        cv2.createTrackbar('Toggle_switch', 'Image', 0, 2, self.toggle)


        # Initialize HSV values
        self.lower_h = 0
        self.upper_h = 179
        self.lower_s = 0
        self.upper_s = 255
        self.lower_v = 0
        self.upper_v = 255
        self.lower_t = 0
        self.upper_t = 255
        self.toggle = 0


        # Initialize CvBridge
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # print("receiving")
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {str(e)}")
            return

        # Apply HSV adjustments
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_hls = np.array([self.lower_h, self.lower_s, self.lower_v])
        upper_hls = np.array([self.upper_h, self.upper_s , self.upper_v])
        mask = cv2.inRange(hsv_image, lower_hls, upper_hls)
        masked_image = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
        gray = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray, self.lower_t, self.upper_t, cv2.THRESH_BINARY)
        blur = cv2.GaussianBlur(thresh,(3, 3), 0)
        canny = cv2.Canny(blur, 40, 60)
        masked_image = cv2.resize(masked_image, (554, 500))
        thresh = cv2.resize(thresh, (554, 500))
        # hsv_image[:, :, 0] = (hsv_image[:, :, 0] + self.hue) % 180
        # hsv_image[:, :, 1] += self.saturation
        # hsv_image[:, :, 2] += self.value
        # adjusted_image = cv2.cvtColor(hsv_image, cv2.COLOR_HSV2BGR)
        if self.toggle == 0:
            # Show the adjusted image
            cv2.imshow("Image_output", masked_image)
        if self.toggle == 1:
            # Show the adjusted image
            cv2.imshow("Image_output", blur)
        if self.toggle == 2:
            # Show the adjusted image
            cv2.imshow("Image_output", canny)
        cv2.waitKey(1)
    
    def toggle(self, value):
        self.toggle = value

    def lower_h_callback(self, value):
        self.lower_h = value

    def upper_h_callback(self, value):
        self.upper_h = value

    def lower_s_callback(self, value):
        self.lower_s = value

    def upper_s_callback(self, value):
        self.upper_s = value

    def lower_v_callback(self, value):
        self.lower_v = value

    def upper_v_callback(self, value):
        self.upper_v = value
    
    def lower_t_callback(self, value):
        self.lower_t = value

    def upper_t_callback(self, value):
        self.upper_t = value

def main(args=None):
    rclpy.init(args=args)
    node = HSVAdjustmentNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
