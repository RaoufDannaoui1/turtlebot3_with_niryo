#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv2 import aruco

class ArucoDetection:
    def __init__(self):
        rospy.init_node('aruco_detection', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback)
        self.aruco_pub = rospy.Publisher('/aruco_markers', Image, queue_size=10)
        self.aruco_distance_pub = rospy.Publisher('/aruco_distance', Float64, queue_size=10)

        self.camera_matrix = np.array([[153.53337,   0.     , 150.50761],
                                      [0.       , 153.62598, 118.64754],
                                      [0.       ,   0.     ,   1.     ]])
        
        self.distortion_coefficients = np.array([-0.318094, 0.090092, 0.000346, -0.000410, 0.0])

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters_create()
        
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
            if ids is not None:
                for i in range(len(ids)):
                    # print(f"Detected ArUco marker {ids[i]}")
                    rotation_vectors, translation_vectors, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, self.camera_matrix, self.distortion_coefficients)

                    aruco.drawDetectedMarkers(cv_image, corners, ids)
                    # aruco.drawAxis(cv_image, self.camera_matrix, self.distortion_coefficients, rotation_vectors, translation_vectors, 0.1)

                    # x,y = int(corners[1][0][0][0]),int(corners[1][0][0][1])

                    distance_to_tag = np.linalg.norm(translation_vectors[0])
                    print(f"detected marker {ids[i]} at distance {distance_to_tag} meters")

                    self.aruco_distance_pub.publish(Float64(distance_to_tag))
                
                # Convert the image back to ROS format and publish it
                aruco_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                self.aruco_pub.publish(aruco_image_msg)

        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

if __name__ == '__main__':
    try:
        aruco_detection = ArucoDetection()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
