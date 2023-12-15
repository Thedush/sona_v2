from struct import pack
import cv2 as cv
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
#assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import glob
import numpy as np
from cv_bridge import CvBridge
import yaml
import tarfile
from io import BytesIO
import time
import numpy.linalg
import math
import subprocess


CHESS_BOARD_DIM = (8,6)

class CameraCal(Node):
    def __init__(self):
        super().__init__('camera_cal')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.image_operation = self.create_subscription(
            Int16,
            'image_operation',  # Replace with your actual topic name
            self.image_operation_callback,
            10  # QoS profile, adjust as needed
        )
        self.image_operation
        self.subscription
        self.cv_bridge = CvBridge()
        self.key = 0
        self.db =[]
        self.counter = 0  # image_counter
        self.startswitch = 1
        
        
        path = self.create_dir()
        print("PATH", path)
        

    def image_callback(self, msg):
        try:
            if self.startswitch:  
                cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                copyFrame=cv_image.copy()
                gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
                
                #  cv.waitKey(1)
                
                image, board_detected = self.detect_checker_board(cv_image, gray, criteria, CHESS_BOARD_DIM)
                copyFrame = self.create_frame(cv_image,copyFrame,board_detected)
                
            #  show_frame(board_detected,copyFrame,n=0)

        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))      
    
    def image_operation_callback(self, msg):
        self.key = msg.data

        self.get_logger().info(f'Received: {msg.data}')


    def detect_checker_board(self,image, grayImage, criteria, boardDimension):
            
            ret, corners = cv.findChessboardCorners(grayImage, boardDimension, flags=cv.CALIB_USE_INTRINSIC_GUESS)
            
            if ret == True:
                # print("ret = True - corner detected")
                corners1 = cv.cornerSubPix(grayImage, corners, (3, 3), (-1, -1), criteria)
                #print("cornerSubPix done")
                image = cv.drawChessboardCorners(image, boardDimension, corners1, ret)
                #print("drawChessboardCorners done")

            # else:
            #     print("ret = False - corner not detected")

            return image, ret

    def create_dir(self):
            
            # Specify the ROS2 workspace path
            ros2_workspace = "/home/pramitha/camera_ws"

                # Specify the Python package name
            package_name = "calibration"

                # Set the desired directory path within the package
            image_dir_path = os.path.join(ros2_workspace, 'src', package_name, 'images')

            CHECK_DIR = os.path.isdir(image_dir_path)
                # if directory does not exist create
            if not CHECK_DIR:
                    os.makedirs(image_dir_path)
                    print(f'"{image_dir_path}" Directory is created')
            else:
                    print(f'"{image_dir_path}" Directory already Exists.')

            return image_dir_path


    def create_frame(self,cv_image,copyFrame, board_detected):
        

        
        cv.putText(
            cv_image,
            f"saved_img : {self.counter}",
            (30, 40),
            cv.FONT_HERSHEY_PLAIN,
            1.4,
            (0, 255, 0),
            2,
            cv.LINE_AA,
        )

        # window_name = "frame"
        # cv.namedWindow(window_name, cv.WINDOW_NORMAL)
        # cv.resizeWindow(window_name, 700,700)

        # window_name_copy = "copyFrame"
        # cv.namedWindow(window_name_copy, cv.WINDOW_NORMAL)
        # cv.resizeWindow(window_name_copy, 500, 500)


        cv.imshow("window_name", cv_image)
        # cv.imshow("window_name_copy", copyFrame)
        cv.waitKey(1)


        
        if self.key == 2:
            # print("counter : ",self.key)
            self.calculate_matrix() 
            self.key = 0   
            

        if self.key == 1 and board_detected == True:
            # print("counter : ",self.key)
            # storing the checker board image
            ros2_workspace = "/home/pramitha/camera_ws"
            package_name = "calibration"
            image_dir_path = os.path.join(ros2_workspace, 'src', package_name, 'images')
            # print(image_dir_path)
            cv.imwrite(f"{image_dir_path}/image{self.counter}.png", copyFrame)
            # print()
            self.key = 0

            print(f"saved image number {self.counter}")
            self.counter += 1  # incrementing the image counter
             


    def calculate_matrix(self):
        print("calculate matrix") 
        subpix_criteria = (cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER, 30, 0.1)
        calibration_flags = cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv.fisheye.CALIB_FIX_SKEW #+cv.fisheye.CALIB_CHECK_COND

        objp = np.zeros((1, CHESS_BOARD_DIM[0]*CHESS_BOARD_DIM[1], 3), np.float32)
        objp[0,:,:2] = np.mgrid[0:CHESS_BOARD_DIM[0], 0:CHESS_BOARD_DIM[1]].T.reshape(-1, 2)
        #_img_shape = None
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        ros2_workspace = "/home/pramitha/camera_ws"
        package_name = "calibration"
        image_dir_path = os.path.join(ros2_workspace, 'src', package_name, 'images')

    # # image_dir_path = "images"

        files = os.listdir(image_dir_path)
        for file in files:
            #print(file)
            imagePath = os.path.join(image_dir_path, file)
            # print(imagePath)

            img = cv.imread(imagePath)
            _img_shape = (640,480)
            gray1 = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
            gray = cv.resize(gray1,_img_shape)
    #     # Find the chess board corners
            ret, corners = cv.findChessboardCorners(gray, CHESS_BOARD_DIM, cv.CALIB_CB_ADAPTIVE_THRESH+cv.CALIB_CB_FAST_CHECK+cv.CALIB_CB_NORMALIZE_IMAGE)
    #     # If found, add object points, image points (after refining them)
            if ret == True:
                objpoints.append(objp)
                cv.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
                imgpoints.append(corners)
        N_OK = len(objpoints)
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        rms, K, D, rvecs, tvecs = \
            cv.fisheye.calibrate(
                objpoints,
                imgpoints,
                _img_shape,
                K,
                D,
                rvecs,
                tvecs,
                calibration_flags,
                (cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            )

        print("Found " + str(N_OK) + " valid images for calibration")

        a = 0.0
        # self.D = D.flat[:4].reshape(-1, 1)
        # self.K=K
        
        self._img_shape = _img_shape
        P = np.zeros((3, 4), dtype=np.float64)
        R = np.eye(3, dtype=np.float64)


        P[:3,:3] = K[:3,:3]
        P[0,0] /= (1. + a)
        P[1,1] /= (1. + a)

        mapx, mapy = cv.fisheye.initUndistortRectifyMap(K, D, R, P, _img_shape, cv.CV_32FC1)

        # print("K=np.array(" + str(K.tolist()) + ")")
        # print("D=np.array(" + str(D.tolist()) + ")")
        # print("P=np.array(" + str(P.tolist()) + ")")
        # print("R=np.array(" + str(R.tolist()) + ")")
        # self.P=P
        # self.R=R

        cv.destroyAllWindows()
        self.startswitch = 0
        # print("calculate matrix complee()")
        # taradd('ost.yaml', self.yaml()
        # self.yaml(D,K,R,P,_img_shape)
        self.store(D,K,R,P,_img_shape)
        
    def store(self,d, k, r, p, size):
         print("done with calculate_matrix and storing them")
         assert k.shape == (3, 3)
         assert r.shape == (3, 3)
         assert p.shape == (3, 4)
        #  data: [" + ", ".join(["%8f" % d[i,0] for i in range(d.shape[0])]) + "]\n"
       
         calmessage = (""
            + "image_width: " + str(size[0]) + "\n"
            + "image_height: " + str(size[1]) + "\n"
            + "camera_name: "  + "camera" + "\n"
            + "camera_matrix:\n"
            + "  rows: 3\n"
            + "  cols: 3\n"
            + "  data: [" + ", ".join(["%8f" % i for i in k.reshape(1,9)[0]]) + "]\n"
            + "distortion_model: "  + "fisheye" +"\n"
            + "distortion_coefficients:\n"
            + "  rows: 1\n"
            + "  cols: 4\n"
            + "  data: [" + ", ".join(["%8f" % i for i in d.reshape(1,4)[0]]) + "]\n"
            + "rectification_matrix:\n"
            + "  rows: 3\n"
            + "  cols: 3\n"
            + "  data: [" + ", ".join(["%8f" % i for i in r.reshape(1,9)[0]]) + "]\n"
            + "projection_matrix:\n"
            + "  rows: 3\n"
            + "  cols: 4\n"
            + "  data: [" + ", ".join(["%8f" % i for i in p.reshape(1,12)[0]]) + "]\n"
            + "")
         
         print(calmessage)
         
         s = calmessage
        # Replace 'your_command_here' with the actual command you want to run
         command = """echo '""" + s + """' """ +"> /home/pramitha/camera_ws/src/calibration/config/final.yaml"
        #  print("output.yaml")
         ros2_workspace = "/home/pramitha/camera_ws"
         package_name = "calibration"
         yaml_file_path = os.path.join(ros2_workspace, 'src', package_name, 'config','output.yaml')
        #  print(command)
         self.save_yaml(command, yaml_file_path)
         return calmessage    

    

    def save_yaml(self, command, output_yaml_file):
        try:
            # Run the command and capture the output
            result = subprocess.run(command, shell=True, capture_output=True, text=True, check=True)

            # Convert the command output to a Python data structure
            output_data = yaml.safe_load(result.stdout)

            # Write the data to a YAML file
            with open(output_yaml_file, 'w') as yaml_file:
                yaml.dump(output_data, yaml_file, default_flow_style=False)

            print(f"Command output stored in {output_yaml_file}")
        
        except subprocess.CalledProcessError as e:
            print(f"Error running the command: {e}")
        
        except yaml.YAMLError as e:
            print(f"Error writing YAML file: {e}")

  
    # def do_save(self):
    #     ros2_workspace = "/home/pramitha/camera_ws"
    #     package_name = "calibration"
    #     filename = os.path.join(ros2_workspace, 'src', package_name, 'tmp/calibrationdata.tar.gz')
    #     tf = tarfile.open(filename, 'w:gz')
    #     self.do_tarfile_save(tf) # Must be overridden in subclasses
    #     tf.close()
    #     print(("Wrote calibration data to", filename))

    # def do_tarfile_save(self, tf):
    #     """ Write images and calibration solution to a tarfile object """

    #     def taradd(name, buf):
    #         if isinstance(buf, str):
    #             s = BytesIO(buf.encode('utf-8'))
    #         else:
    #             s = BytesIO(buf)
    #         ti = tarfile.TarInfo(name)
    #         ti.size = len(s.getvalue())
    #         ti.uname = 'calibrator'
    #         ti.mtime = int(time.time())
    #         tf.addfile(tarinfo=ti, fileobj=s)

    #     ims = [("left-%04d.png" % i, im) for i,(_, im) in enumerate(self.db)]
    #     for (name, im) in ims:
    #         taradd(name, cv.imencode(".png", im)[1].tostring())
    #     taradd('ost.yaml', self.yaml())
    #     # taradd('ost.txt', self.ost())
    # def get_parameters(self, corners, board, size):
        # """
        # Return list of parameters [X, Y, size, skew] describing the checkerboard view.
        # """
        # (width, height) = size
        # Xs = corners[:,:,0]
        # Ys = corners[:,:,1]
        # area = _get_area(corners, board)
        # border = math.sqrt(area)
        # # For X and Y, we "shrink" the image all around by approx. half the board size.
        # # Otherwise large boards are penalized because you can't get much X/Y variation.
        # p_x = min(1.0, max(0.0, (numpy.mean(Xs) - border / 2) / (width  - border)))
        # p_y = min(1.0, max(0.0, (numpy.mean(Ys) - border / 2) / (height - border)))
        # p_size = math.sqrt(area / (width * height))
        # skew = _get_skew(corners, board)
        # params = [p_x, p_y, p_size, skew]
        # return params
    
    # def save(self):
    #      return self.store()
    
    #      k_list = k.reshape(1, 9).tolist()[0]
    #      d_list = d.reshape(1, 4).tolist()[0]
    #      r_list = r.reshape(1, 9).tolist()[0]
    #      p_list = p.reshape(1, 12).tolist()[0]
                
    #      calmessage = {
    #     "image_width": size[0],
    #     "image_height": size[1],
    #     "camera_name":'',
    #     "camera_matrix": {
    #         "rows": 3,
    #         "cols": 3,
    #         "data": k_list,
    #     },
    #     "distortion_model": None,
    #     "distortion_coefficients": {
    #         "rows": 1,
    #         "cols": 4,
    #         "data": d_list,
    #     },
    #     "rectification_matrix": {
    #         "rows": 3,
    #         "cols": 3,
    #         "data": r_list,
    #     },
    #     "projection_matrix": {
    #         "rows": 3,
    #         "cols": 4,
    #         "data": p_list,
    #     },
    # }
    
# Accessing a specific value
# print("Image Width:", calmessage_dict["image_width"])
# print("Camera Matrix Data:", calmessage_dict["camera_matrix"]["data"])


        #  print("stored")
        #  #print(calmessage)
        #  ros2_workspace = "/home/pramitha/camera_ws"
        #  package_name = "calibration"
        #  yaml_file_path = os.path.join(ros2_workspace, 'src', package_name, 'config', 'matrix.yaml')

        # # Save the calmessage to the YAML file
        #  with open(yaml_file_path, 'w') as yaml_file:
        #       yaml.dump(calmessage, yaml_file, default_flow_style=True)
        #  return calmessage


def main(): #args=None 
    rclpy.init()  #args=args

    camera_cal = CameraCal()

    rclpy.spin(camera_cal)    

    camera_cal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    CHESS_BOARD_DIM = (8,6)
    main()

