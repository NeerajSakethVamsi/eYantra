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


################### IMPORT MODULES #######################

import rospy
import signal		# To handle Signals by OS/user
import sys		# To handle Signals by OS/user

from geometry_msgs.msg import Wrench		# Message type used for publishing force vectors
from geometry_msgs.msg import PoseArray	# Message type used for receiving goals
from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback

import time
import math		# If you find it useful

from tf.transformations import euler_from_quaternion	# Convert angles

################## GLOBAL VARIABLES ######################

PI = 3.14

x_goals = []
y_goals = []
theta_goals = []
j=0
a=0
hola_x = 0
hola_y = 0
hola_theta = 0
p_x = 0
p_y = 0
p_t = 0
v_1 =0
v_2 = 0
v_3 = 0
global_err_x = 0
global_err_y = 0
global_err_theta = 0
local_err_x = 0
local_err_y = 0
vel_x = 0
vel_y = 0
vel_z = 0
right_wheel_pub = None
left_wheel_pub = None
front_wheel_pub = None


##################### FUNCTION DEFINITIONS #######################

# NOTE :  You may define multiple helper functions here and use in your code

def signal_handler(sig, frame):
	  
	# NOTE: This function is called when a program is terminated by "Ctr+C" i.e. SIGINT signal 	
	print('Clean-up !')
	cleanup()
	sys.exit(0)

def cleanup():
	global v_1,v_2,v_3
	############ ADD YOUR CODE HERE ############
	v_3 = 0
	v_1 = 0
	v_2 = 0
	# INSTRUCTIONS & HELP : 
	#	-> Not mandatory - but it is recommended to do some cleanup over here,
	#	   to make sure that your logic and the robot model behaves predictably in the next run.

	############################################
  
  
def task2_goals_Cb(msg : PoseArray):
	global x_goals, y_goals, theta_goals
	x_goals.clear()
	y_goals.clear()
	theta_goals.clear()

	for waypoint_pose in msg.poses:
		x_goals.append(waypoint_pose.position.x)
		y_goals.append(waypoint_pose.position.y)

		orientation_q = waypoint_pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		theta_goal = euler_from_quaternion (orientation_list)[2]
		theta_goals.append(theta_goal)

def aruco_feedback_Cb(msg : Pose2D):
	global hola_x,hola_y,hola_theta
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Receive & store the feedback / coordinates found by aruco detection logic.
	#	-> This feedback plays the same role as the 'Odometry' did in the previous task.

	############################################

	hola_x = msg.x
	hola_y = msg.y
	hola_theta = msg.theta
	# print(hola_x, hola_y, hola_theta)


#def inverse_kinematics():
	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Use the target velocity you calculated for the robot in previous task, and
	#	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
	#	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
	############################################


def main():

	global right_wheel_pub,front_wheel_pub,left_wheel_pub,p_x,p_y,p_t,v_1,v_2,v_3,global_err_x,global_err_y,global_err_theta,local_err_x,local_err_y,vel_x,vel_y,vel_z,j


	rospy.init_node('controller_node')

	#signal.signal(signal.SIGINT, signal_handler)

	# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
	#	Use the below given topics to generate motion for the robot.
	right_wheel_pub = rospy.Publisher('/right_wheel_force', Wrench, queue_size=10)
	front_wheel_pub = rospy.Publisher('/front_wheel_force', Wrench, queue_size=10)
	left_wheel_pub = rospy.Publisher('/left_wheel_force', Wrench, queue_size=10)

	rospy.Subscriber('detected_aruco',Pose2D,aruco_feedback_Cb)
	rospy.Subscriber('task2_goals',PoseArray,task2_goals_Cb)

	rate = rospy.Rate(100)

	vel_r = Wrench()
	vel_f = Wrench()
	vel_l = Wrench()

	
	vel_r.force.x = 0
	vel_r.force.y = 0
	vel_r.force.z = 0
	vel_r.torque.x = 0
	vel_r.torque.y = 0
	vel_r.torque.z = 0
	vel_f.force.x = 0
	vel_f.force.y = 0
	vel_f.force.z = 0
	vel_f.torque.x = 0
	vel_f.torque.y = 0
	vel_f.torque.z = 0
	vel_l.force.x = 0
	vel_l.force.y = 0
	vel_l.force.z = 0
	vel_l.torque.x = 0
	vel_l.torque.y = 0
	vel_l.torque.z = 0

	############ ADD YOUR CODE HERE ############

	# INSTRUCTIONS & HELP : 
	#	-> Make use of the logic you have developed in previous task to go-to-goal.
	#	-> Extend your logic to handle the feedback that is in terms of pixels.
	#	-> Tune your controller accordingly.
	# 	-> In this task you have to further implement (Inverse Kinematics!)
	#      find three omni-wheel velocities (v1, v2, v3) = left/right/center_wheel_force (assumption to simplify)
	#      given velocity of the chassis (Vx, Vy, W)
	#	   

	p_x = 0.6
	p_y = 0.6
	p_t = 2

		
	while not rospy.is_shutdown():
		if j<len(x_goals) :	
			global_err_x = (x_goals[j]-hola_x)
			global_err_y = (y_goals[j]-hola_y)
			global_err_theta = (theta_goals[j]-hola_theta)
			
			local_err_x = ((global_err_x*math.cos(global_err_theta))+(global_err_y*math.sin(global_err_theta)))
			local_err_y = ((-global_err_x*math.sin(global_err_theta))+(global_err_y*math.cos(global_err_theta)))
			print(local_err_x,local_err_y,global_err_theta)
			vel_x = p_x*local_err_x
			vel_y = p_y*local_err_y
			vel_z = p_t*global_err_theta


			# v_1 = vel_r.force.x
			
			# v_2 = vel_f.force.x
			
			# v_3 = vel_l.force.x

			v_2 = vel_x-vel_z

			v_1 = -(vel_x/2)+math.sqrt(3)*(vel_y/2)-vel_z

			v_3 = -(vel_x/2)-math.sqrt(3)*(vel_y/2)-vel_z

			

			if local_err_x<5 and local_err_x>-5 and local_err_y<5 and local_err_y>-5 and j<=len(x_goals) and global_err_theta>-0.15 and global_err_theta<0.15:
				j=j+1
				

			if j==len(x_goals):
				j=0
				v_1 =0
				v_2 =0
				v_3 = 0

			#
			# Calculate Error from feedback

			# Change the frame by using Rotation Matrix (If you find it required)

			# Calculate the required velocity of bot for the next iteration(s)
			
			# Find the required force vectors for individual wheels from it.(Inverse Kinematics)

			# Apply appropriate force vectors

			# Modify the condition to Switch to Next goal (given position in pixels instead of meter

			vel_r.force.x = v_1
			vel_f.force.x = v_2
			vel_l.force.x = v_3

			right_wheel_pub.publish(vel_r)
			front_wheel_pub.publish(vel_f)
			left_wheel_pub.publish(vel_l)
			
			rate.sleep()

    ############################################

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

