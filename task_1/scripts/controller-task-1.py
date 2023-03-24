#!/usr/bin/env python3

from cmath import cos, sin
import rospy

# publishing to /cmd_vel with msg type: Twist
from geometry_msgs.msg import Twist
# subscribing to /odom with msg type: Odometry
from nav_msgs.msg import Odometry

# for finding sin() cos() 
import math

# Odometry is given as a quaternion, but for the controller we'll need to find the orientaion theta by converting to euler angle
from tf.transformations import euler_from_quaternion

from geometry_msgs.msg import PoseArray

hola_x = 0
hola_y = 0
hola_theta = 0
pi = math.pi
a_x = 0
b = 0
c = 0
x_goals = [0]
y_goals = [0]
theta_goals = [1]
i=0
hola_theta_converted = 0

def task1_goals_Cb(msg):
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


def odometryCb(msg : Odometry):
	global hola_x, hola_y, hola_theta,a_x,b,c
	hola_x = msg.pose.pose.position.x
	hola_y = msg.pose.pose.position.y
	hola_theta = msg.pose.pose.orientation.w
	
	a_x = msg.pose.pose.orientation.x
	b = msg.pose.pose.orientation.y
	c = msg.pose.pose.orientation.z

	# Write your code to take the msg and update the three variables

def main():
	global i,hola_theta_converted
	# Initialze Node
	# We'll leave this for you to figure out the syntax for 
	# initialising node named "controller"
	rospy.init_node('controller', anonymous=True)
	
	# Initialze Publisher and Subscriber
	# We'll leave this for you to figure out the syntax for
	# initialising publisher and subscriber of cmd_vel and odom respectively
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rospy.Subscriber("odom", Odometry, odometryCb)
	rospy.Subscriber('task1_goals', PoseArray, task1_goals_Cb)



	# Declare a Twist message
	vel = Twist()
	# Initialise the required variables to 0
	# <This is explained below>
	vel.linear.x = 0
	vel.linear.y = 0
	vel.linear.z = 0
	vel.angular.x = 0
	vel.angular.y = 0
	vel.angular.z = 0
	
	# For maintaining control loop rate.
	rate = rospy.Rate(100)
	# Initialise variables that may be needed for the control loop
	# For ex: x_d, y_d, theta_d (in **meters** and **radians**) for defining desired goal-pose.
	# x_d = x_goals[i]
	# y_d = y_goals[i]
	# theta_d = theta_goals[i]
	# and also Kp values for the P Controller
	p_x = 0.6
	p_y = 0.6
	p_w = 1.75

	#
	# 
	# Control Loop goes here
	#
	#

	while not rospy.is_shutdown():
		if i<len(x_goals):
		# Find error (in x, y and theta) in global frame
		# the /odom topic is giving pose of the robot in global frame
		# the desired pose is declared above and defined by you in global frame
		# therefore calculate error in global frame
			a = euler_from_quaternion([a_x,b,c,hola_theta])
			global_err_x = (x_goals[i]-hola_x)
			global_err_y = (y_goals[i]-hola_y)

			hola_theta_converted = a[2]
			global_err_theta = (theta_goals[i]-hola_theta_converted)
		# (Calculate error in body frame)
		# But for Controller outputs robot velocity in robot_body frame, 
		# i.e. velocity are define is in x, y of the robot frame, 
		# Notice: the direction of z axis says the same in global and body frame
		# therefore the errors will have have to be calculated in body frame.
		#
		# This is probably the crux of Task 1, figure this out and rest should be fine.
		
			local_err_x = ((global_err_x*math.cos(a[2]))+(global_err_y*math.sin(a[2])))
			local_err_y = ((-global_err_x*math.sin(a[2]))+(global_err_y*math.cos(a[2])))

		# Finally implement a P controller 
		# to react to the error with velocities in x, y and theta.
			vel_x = p_x*local_err_x
			vel_y = p_y*local_err_y
			vel_z = p_w*global_err_theta

		# Safety Check
		# make sure the velocities are within a range.
		# for now since we are in a simulator and we are not dealing with actual physical limits on the system 
		# we may get away with skipping this step. But it will be very necessary in the long run.

			vel.linear.x = vel_x
			vel.linear.y = vel_y
			vel.angular.z = vel_z

			if local_err_x <= 0.03 and local_err_x >= -0.03 and local_err_y <= 0.03 and local_err_y >= -0.03 and global_err_theta < pi/180 and global_err_theta>-pi/270 and i<len(x_goals) :
				i=i+1
				vel.linear.x = 0
				vel.linear.y = 0
				vel.angular.z = 0

			pub.publish(vel)
			rate.sleep()
	


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass