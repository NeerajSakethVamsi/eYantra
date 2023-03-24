#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		[ Team-ID ]
# Author List:		[ Names of team members worked on this file separated by Comma: Name1, Name2, ... ]
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


######################## IMPORT MODULES ##########################

import numpy				# If you find it required
import rospy 				
from sensor_msgs.msg import Image 	# Image is the message type for images in ROS
from cv_bridge import CvBridge	# Package to convert between ROS and OpenCV Images
import cv2				# OpenCV Library
from cv2 import aruco 
import math				# If you find it required
from geometry_msgs.msg import Pose2D	# Required to publish ARUCO's detected position & orientation

############################ GLOBALS #############################

aruco_publisher = rospy.Publisher('detected_aruco', Pose2D)
aruco_msg = Pose2D()
aruco_dictionary = cv2.aruco.Dictionary_get(aruco.DICT_4X4_50)
aruco_marker_parameters = cv2.aruco.DetectorParameters_create()

##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def callback(data):
	# Bridge is Used to Convert ROS Image message to OpenCV image
    br = CvBridge()
    rospy.loginfo("receiving camera frame")
    get_frame = br.imgmsg_to_cv2(data, "mono8")		# Receiving raw image in a "grayscale" format
    current_frame = cv2.resize(get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)

    corners, ids, rejected = cv2.aruco.detectMarkers(current_frame, aruco_dictionary, parameters= aruco_marker_parameters)
    
    x_coordinate = ((corners[0])[0][0][0]+(corners[0])[0][1][0]+(corners[0])[0][2][0]+(corners[0])[0][3][0])/4
    y_coordinate = ((corners[0])[0][0][1]+(corners[0])[0][1][1]+(corners[0])[0][2][1]+(corners[0])[0][3][1])/4
    x0 = (corners[0])[0][0][0]
    x1 = (corners[0])[0][1][0]
    y0 = (corners[0])[0][0][1]
    y1 = (corners[0])[0][1][1]

    
    tan_inverse = math.atan((-((corners[0])[0][1][0]-(corners[0])[0][0][0]))/((corners[0])[0][1][1]-(corners[0])[0][0][1]))
    tan_inverse = math.atan(-(x1-x0)/(y1-y0))
    theta = abs(math.atan(-(x1-x0)/(y1-y0)))
    
    orientation = 0

    # if x1>x0 and y1<y0:
    #     orientation = abs(math.atan(-(x1-x0)/(y1-y0)))
    # elif x1>x0 and y1>y0:
    #     orientation = math.pi -abs(math.atan(-(x1-x0)/(y1-y0)))
    # elif x1<x0 and y1>y0:
    #     orientation = abs(math.atan(-(x1-x0)/(y1-y0))) - math.pi
    # elif x1<x0 and y1<y0:
    #     orientation = -abs(math.atan(-(x1-x0)/(y1-y0)))
    # elif x1==x0 and y1>y0:
    #     orientation = -math.pi
    # elif x1==x0 and y1<y0:
    #     orientation = 0
    # elif x1>x0 and y1==y0:
    #     orientation = math.pi / 2
    # elif x1<x0 and y1==y0:
    #     orientation = -math.pi / 2

    if x1 > x0:
        if y1 == y0:
            orientation = 0
        elif y1 < y0:
            orientation = math.pi*3/2 + abs(math.atan(-(x1-x0)/(y1-y0)))
        elif y1 > y0:
            orientation = math.pi/2 - abs(math.atan(-(x1-x0)/(y1-y0)))
    elif x1 < x0:
        if y1 == y0:
            orientation = 0
        elif y1 < y0:
            orientation = math.pi*2 - abs(math.atan(-(x1-x0)/(y1-y0)))
        elif y1 > y0:
            orientation = math.pi/2 +  abs(math.atan(-(x1-x0)/(y1-y0)))
    elif x0 == x1 and y0 == y1:
        orientation = 0
        
        
    
    
    print(x_coordinate, y_coordinate, orientation)

    aruco_msg.x = x_coordinate
    aruco_msg.y = y_coordinate
    aruco_msg.theta = orientation
    
    aruco_publisher.publish(aruco_msg)
    cv2.imshow("camera_frame", current_frame)
    cv2.waitKey(1)
    

	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use OpenCV to find ARUCO MARKER from the IMAGE
	#	-> You are allowed to use any other library for ARUCO detection, 
	#        but the code should be strictly written by your team and
	#	   your code should take image & publish coordinates on the topics as specified only.  
	#	-> Use basic high-school geometry of "TRAPEZOIDAL SHAPES" to find accurate marker coordinates & orientation :)
	#	-> Observe the accuracy of aruco detection & handle every possible corner cases to get maximum scores !

	############################################
      
def main():
	rospy.init_node('aruco_feedback_node')  
	rospy.Subscriber('overhead_cam/image_raw', Image, callback)
	rospy.spin()
  
if __name__ == '__main__':
  main()
