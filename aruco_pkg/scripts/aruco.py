#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from rclpy.timer import Timer
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from geometry_msgs.msg import TransformStamped 
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R


import cv2
from cv2 import putText
import numpy as np 
import pickle



class PublisherArucoNode(Node):
    def __init__(self):
        super().__init__('publisher_arucos_node')

        # Variables
        self.camera_calibration_parameters_filename = "/home/genis/ros2/train_ws/src/final_train/aruco_pkg/scripts/calibration_chessboard.yaml"

        self.aruco_marker_name = "aruco_marker"

        # Load the camera parameters from the saved file
        cv_file = cv2.FileStorage(
        self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
        self.mtx = cv_file.getNode('K').mat()
        self.dst = cv_file.getNode('D').mat()
        cv_file.release()

        # Initialize the transform broadcaster
        self.tfbroadcaster = TransformBroadcaster(self)

        # Change the DICT_6X6_250 to another if you want another dict =============================================================> important

        self.desired_aruco_dictionary = "DICT_6X6_250"
        self.marker_length = 0.0785

        self.areaOfPolygon = 0
        
        self.id_aruco_int32 = Int32()

        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
        }



        # Timers, publishers & subscribers
        self.subscription_image = self.create_subscription(Image, '/image_raw', self.camera_callback, 10)
        self.cv_bridge = CvBridge()
        self.publisher_int32 = self.create_publisher(Int32, '/id_arucos', 10)

        self.get_logger().info("publisher_node initialized")


    def PPDisstance(self, A, B):
        return ((A[0] + B[0])**2 + (A[1] + B[1])**2)**0.5

    def PolygonArea(self, vertices):
        area = 0
        for i in range(len(vertices)):
            area += vertices[i][0] * vertices[(i+1) % len(vertices)][1] - vertices[(i+1) % len(vertices)][0] * vertices[i][1]
        return abs(area) / 2   
    
    def rotation_matrix_x(self, angle):
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        return np.array([[1, 0, 0], [0, cos_a, -sin_a], [0, sin_a, cos_a]])
    
    def rotation_matrix_y(self, angle):
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        return np.array([[cos_a, 0, sin_a],
                        [0, 1, 0],
                        [-sin_a, 0, cos_a]])

    def camera_callback(self, msg):

        # Check that we have a valid ArUco marker
        if self.ARUCO_DICT.get(self.desired_aruco_dictionary, None) is None:
            print("[INFO] ArUCo tag of '{}' is not supported".format(
            args["type"]))
            sys.exit(0)
            
        # Load the ArUco dictionary
        # print("[INFO] detecting '{}' markers...".format(
        #     self.desired_aruco_dictionary))

        # Change the DICT_6X6_250 to another if you want another dict =============================================================> important
        this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        this_aruco_parameters = cv2.aruco.DetectorParameters()

        self.id_aruco_int32.data = -1


        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
        img_copy = cv_image.copy()

        # print('----> 8')
        # Detect ArUco markers in the video frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img_copy, this_aruco_dictionary, parameters=this_aruco_parameters)

        angle_x = np.radians(270)
        angle_y = np.radians(90)

        if len(corners) > 0:
            # Flatten the ArUco IDs list
            ids = ids.flatten()
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_length, self.mtx, self.dst)
            
            for marker_corner, marker_id, rvec, tvec in zip(corners, ids, rvecs, tvecs):

                # Extract the marker corners
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners


                # Convert the (x,y) coordinate pairs to integers
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))
                def shape():
                    figurativa = ''
                    return figurativa

                self.id_aruco_int32.data = int(marker_id)
                
                # Draw the bounding box of the ArUco detection
                cv2.line(img_copy, top_left, top_right, (0, 255, 0), 2)
                cv2.line(img_copy, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(img_copy, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(img_copy, bottom_left, top_left, (0, 255, 0), 2)
                
                # Calculate and draw the center of the ArUco marker
                center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                cv2.circle(img_copy, (center_x, center_y), 4, (0, 0, 255), -1)
                
                # Draw the ArUco marker ID on the video frame
                # The ID is always located at the top_left of the ArUco marker
                cv2.putText(img_copy, str(marker_id), 
                (top_left[0], top_left[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
                
                # Create the coordinate transform
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_link'
                t.child_frame_id = f"{self.aruco_marker_name}_{marker_id}"
                # t.child_frame_id = f"{self.aruco_marker_name}"

                tvec_rotated = np.dot(self.rotation_matrix_x(angle_x), tvec[0].T).T
                #tvec_rotated = np.dot(self.rotation_matrix_y(angle_y), tvec.reshape(-1, 1))
                tvec_rotated = tvec_rotated.flatten()


                # Store the translation (position) information
                t.transform.translation.x = tvec_rotated[0]
                t.transform.translation.y = -tvec_rotated[1]
                t.transform.translation.z = tvec_rotated[2]

                # Store the rotation information
                rotation_matrix_cv = cv2.Rodrigues(rvec)[0]

                rotation_x_180 = np.array([
                    [1, 0, 0],
                    [0, -1, 0],
                    [0, 0, -1]
                ])

                combined_rotation_matrix = np.dot(rotation_x_180, rotation_matrix_cv)

                r = R.from_matrix(combined_rotation_matrix)
                quat = r.as_quat() 

                t.transform.rotation.x = quat[1]
                t.transform.rotation.y = quat[0]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                # Send the transform
                self.tfbroadcaster.sendTransform(t)

        # else:

        #     # Create the coordinate transform
        #     t = TransformStamped()
        #     t.header.stamp = self.get_clock().now().to_msg()
        #     t.header.frame_id = 'camera_link'
        #     # t.child_frame_id = f"{self.aruco_marker_name}_{marker_id}"
        #     t.child_frame_id = "none_aruco"
        #     t.transform.translation.x = 0.0
        #     t.transform.translation.y = 0.0
        #     t.transform.translation.z = 0.0

        #     t.transform.rotation.x = 0.0
        #     t.transform.rotation.y = 0.0
        #     t.transform.rotation.z = 0.0
        #     t.transform.rotation.w = 1.0

        #     self.tfbroadcaster.sendTransform(t)


        self.publisher_int32.publish(self.id_aruco_int32)

        cv2.imshow('result', cv2.cvtColor(img_copy, cv2.COLOR_BGR2RGB))
        cv2.waitKey(10)



def main(args=None):
    rclpy.init(args=args)
    node = PublisherArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()