import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from message_filters import TimeSynchronizer, Subscriber
from rclpy.qos import QoSProfile
import message_filters
# import rospy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data


class Undistort(Node):
    def __init__(self):
        super().__init__('undistort_node')
        # self.subscription = self.create_subscription(
        #     Image,
        #     '/camera/image',
        #     self.image_callback,
        #     rclpy.qos.qos_profile_sensor_data)
        # self.subscription = self.create_subscription(
        #     Image,
        #     '/camera/camera_info',
        #     self.info_callback,
        #     rclpy.qos.qos_profile_sensor_data)

        # image_sub = message_filters.Subscriber('/camera/image', Image)
        # custom_qos_profile = rclpy.rmw_qos_profile_sensor_data
        self.image_sub = message_filters.Subscriber(self, Image, '/camera/image', qos_profile=qos_profile_sensor_data)
        # print("image_sub")
        # info_sub = message_filters.Subscriber('/camera/camera_info', CameraInfo)
        self.info_sub = message_filters.Subscriber(self, CameraInfo, '/camera/camera_info', qos_profile=qos_profile_sensor_data)
        # print("info_sub")
        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.info_sub], 10, 0.2)
        ts.registerCallback(self.callback)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/undistorted_image', 10)
        self.dim=(640,480)
        
    def callback(self, image_sub, info_sub):
        # self.startswitch = 1
        # print("callback")
        try:
            
            cv_image = self.bridge.imgmsg_to_cv2(image_sub, desired_encoding='passthrough')
            if cv_image is None:
                print("Error loading the image.")

            # self.get_logger().info(f"Received message of type: {type(info_sub.d)}")
            # print(list(info_sub.d))
            # my_mat = np.zeros((3, 3), dtype=np.float32)
            # my_mat = info_sub.d
            # // Create a UMat object
            # cv2::UMat myMat = cv::UMat::zeros(3, 3, CV_32FC1);

            # // Create a smart pointer to manage the UMat
            # cv2::Ptr<cv::UMat> myMatPtr = cv::makePtr<cv::UMat>(myMat);
            value_k = np.array(info_sub.k).reshape([3, 3])
            value_d = np.array(info_sub.d).reshape([1,4])
            # print(value_d)
            # self.get_logger().info(f"Received message of type: {type(info_sub.d)}")
            
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(value_k, value_d, np.eye(3), value_k, self.dim, cv2.CV_16SC2)
            new_image = cv2.remap(cv_image, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            undistorted_msg = self.bridge.cv2_to_imgmsg(new_image, encoding='bgr8')
            self.publisher.publish(undistorted_msg)
            # print("DONE")


        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

        
def main(args=None):
    rclpy.init(args=args)
    print("Start")
    undistort_node = Undistort()
    # print("Node Initialised")
    rclpy.spin(undistort_node)
    undistort_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    
    main()
        


        