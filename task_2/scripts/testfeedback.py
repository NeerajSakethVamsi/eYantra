#!/usr/bin/env python3

import numpy				# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
from cv2 import aruco
import math				# If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation

###################################################################
aruco_publisher = rospy.Publisher("detected_aruco", Pose2D)
aruco_msg = Pose2D()
aruco_dictionary = cv2.aruco.Dictionary_get(aruco.DICT_4X4_50)
aruco_marker_parameters = cv2.aruco.DetectorParameters_create()
###################################################################

def callback(data):
    br = CvBridge()
    rospy.loginfo("receiving camera frame!")
    get_frame = br.imgmsg_to_cv2(data,'mono8')
    current_frame = cv2.resize(get_frame,(500, 500), interpolation = cv2.INTER_LINEAR)
    
    corners, ids, rejected = cv2.aruco.detectMarkers(current_frame, aruco_dictionary, parameters= aruco_marker_parameters)
    # rospy.loginfo((corners[0])[0][0][0])
    x_coordinate = ((corners[0])[0][0][0]+(corners[0])[0][1][0]+(corners[0])[0][2][0]+(corners[0])[0][3][0])/4
    y_coordinate = ((corners[0])[0][0][1]+(corners[0])[0][1][1]+(corners[0])[0][2][1]+(corners[0])[0][3][1])/4
    x0 = (corners[0])[0][0][0]
    x1 = (corners[0])[0][1][0]
    y0 = (corners[0])[0][0][1]
    y1 = (corners[0])[0][1][1]

    
    # tan_inverse = math.atan((-((corners[0])[0][1][0]-(corners[0])[0][0][0]))/((corners[0])[0][1][1]-(corners[0])[0][0][1]))
    # tan_inverse = math.atan(-(x1-x0)/(y1-y0))
    # theta = abs(math.atan(-(x1-x0)/(y1-y0)))
    orientation = 0
    
    if x1>x0 and y1<y0:
        orientation = abs(math.atan(-(x1-x0)/(y1-y0)))
    elif x1>x0 and y1>y0:
        orientation = math.pi -abs(math.atan(-(x1-x0)/(y1-y0)))
    elif x1<x0 and y1>y0:
        orientation = abs(math.atan(-(x1-x0)/(y1-y0))) - math.pi
    elif x1<x0 and y1<y0:
        orientation = -abs(math.atan(-(x1-x0)/(y1-y0)))
    elif x1==x0 and y1>y0:
        orientation = -math.pi
    elif x1==x0 and y1<y0:
        orientation = 0
    elif x1>x0 and y1==y0:
        orientation = math.pi / 2
    elif x1<x0 and y1==y0:
        orientation = -math.pi / 2
    
    
    print(x_coordinate, y_coordinate, orientation)
    aruco_msg.x = x_coordinate
    aruco_msg.y = y_coordinate
    aruco_msg.theta = orientation
    
    aruco_publisher.publish(aruco_msg)
    cv2.imshow("camera_frame", current_frame)
    cv2.waitKey(1)
    
    

    
    
def main():
    rospy.init_node("aruco_feedback_node")
    rospy.Subscriber("overhead_cam/image_raw", Image, callback)
    rospy.spin()
if __name__ == "__main__":
    main()