#! /usr/bin/env python

import rospy # Import the Python library for ROS
import tf
from math import degrees, cos, sin, sqrt, pi
from nav_msgs.msg import Odometry # Import the Odometry message from the nav_msgs.msg package

pub = rospy.Publisher('odometry_body_frame',Odometry, queue_size=1) # Create a Publisher object, that will publish on the /odometry_body_frame topic messages of type Odometry

# We don't use this function
def to_positive(angle):
	while angle<0:
		angle=angle + 2*pi
	return angle - pi

# Var created and initiated as empty list
old_pos_x = []
old_pos_y = []
old_pitch = []
old_yaw = []

# Define a function called 'odometry_callback' that receives a parameter named 'odometry_msg'
def odometry_callback(odometry_msg):

	# Odometry msg unpacked, Var created and initialized as the msg data
	odom_position_x = odometry_msg.pose.pose.position.x
	odom_position_y = odometry_msg.pose.pose.position.y

	ori_x = odometry_msg.pose.pose.orientation.x
	ori_y = odometry_msg.pose.pose.orientation.y
	ori_z = odometry_msg.pose.pose.orientation.z
	ori_w = odometry_msg.pose.pose.orientation.w
	
	g_vel_x = odometry_msg.twist.twist.linear.x
	g_vel_y = odometry_msg.twist.twist.linear.y

	quaternion = (ori_x, ori_y, ori_z, ori_w)

	# The data we get from mocap is in quaternion form and we transform to euler
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll  = euler[0]
	pitch = euler[1]
	yaw   =  euler[2]
	
	# Compute body frame velocity
	b_vel_x = g_vel_x * cos(yaw) - g_vel_y * sin(yaw)
	b_vel_y = g_vel_x * sin(yaw) + g_vel_y * cos(yaw)   
	b_vel = abs(sqrt(b_vel_x ** 2 + b_vel_y ** 2)) # Forward velocity of the car

	# Packing data, create a variable that contains message to publish
	qualisys_data = Odometry() # Create a var of type Odometry
	qualisys_data.pose.pose.position.x = odom_position_x
	qualisys_data.pose.pose.position.y = odom_position_y
	qualisys_data.pose.pose.orientation.z = yaw
	qualisys_data.twist.twist.linear.x = b_vel
	
	pub.publish(qualisys_data) # Publish the message within the 'qualisys_data' variable

rospy.init_node('qualTBD5_odom') # Initiate a Node named 'qualTBD5_odom'
sub = rospy.Subscriber('SVEA5/odom', Odometry, odometry_callback) # Create a Subscriber object that will listen to the /SVEA5/odom topic and will cal the 'odometry_callback' function each time it reads something from the topic

rospy.spin() # Keeps node from exiting until the node has been shutdown
