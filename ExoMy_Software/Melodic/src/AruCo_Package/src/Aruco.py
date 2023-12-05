#!/usr/bin/env python
import rospy
from AruCo_Package.msg import ArucoData
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetectorNode:
    def __init__(self):
        rospy.init_node('aruco_detector_node', anonymous=True)

        # Initialize the image subscriber and bridge
        self.image_sub = rospy.Subscriber('/pi_cam/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()

        # Initialize the ArUco detector parameters
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params =  cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Initialize the ArUco ID and translation vector publisher
        self.aruco_data_pub = rospy.Publisher('/aruco_data', ArucoData, queue_size=10)

        # Define the 3D points of the ArUco marker
        self.marker_length = 11.8  # Marker size in cm
        self.obj_points = np.array([[0, 0, 0], [self.marker_length, 0, 0], [self.marker_length, self.marker_length, 0], [0, self.marker_length, 0]], dtype=np.float32)

        # Camera matrix and distortion coefficients
        self.camera_matrix = np.array([[497.89610442, 0., 323.03372646],
                                       [0., 495.64479743, 236.339117],
                                       [0., 0., 1.]])

        self.dist_coeffs = np.array([0.16518645, -0.21558167, -0.00194396, 0.00382525, -0.2397393])

        # Initialize the image with ArUco markers publisher
        self.image_aruco_pub = rospy.Publisher('/pi_cam/image_aruco', Image, queue_size=10)

    def image_callback(self, data):
        try:
            # Convert the ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Convert the image to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
            # Detect ArUco markers
            corners, ids, _ = self.detector.detectMarkers(gray)

            # Publish ArUco IDs and translation vectors
            if ids is not None:
                for i in range(len(ids)):
                    # Use solvePnP to estimate the pose
                    success, rvec_solve, tvec_solve = cv2.solvePnP(self.obj_points, corners[i][0], self.camera_matrix, self.dist_coeffs)
                    if success:
                        # Create and publish custom message
                        aruco_msg = ArucoData()
                        aruco_msg.id = ids[i][0]
                        aruco_msg.translation.x = tvec_solve[0][0]
                        aruco_msg.translation.y = tvec_solve[1][0]
                        aruco_msg.translation.z = tvec_solve[2][0]

                        self.aruco_data_pub.publish(aruco_msg)

            # Draw boxes and IDs on the image
            cv_image_aruco = cv2.aruco.drawDetectedMarkers(cv_image.copy(), corners, ids)

            # Convert the image to ROS format and publish
            image_msg = self.bridge.cv2_to_imgmsg(cv_image_aruco, 'bgr8')
            self.image_aruco_pub.publish(image_msg)

        except Exception as e:
            rospy.logerr('Error processing image: {}'.format(str(e)))

if __name__ == '__main__':
    try:
        aruco_detector_node = ArucoDetectorNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass