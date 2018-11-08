#! /usr/bin/env python

import rospy
import tf

import pdb
 
from nav_msgs.msg import Odometry

def to_positive(angle):
	while angle<0:
		angle=angle+360
	return angle
	
old_pos_x = []
old_pos_y = []
old_pitch=[]
old_yaw=[]

def callback(odometry_msg):

	odom_position_x = odometry_msg.pose.pose.position.x
	odom_position_y = odometry_msg.pose.pose.position.y
	#odom_position_z = odometry_msg.pose.pose.position.z

	ori_x = odometry_msg.pose.pose.orientation.x
	ori_y = odometry_msg.pose.pose.orientation.y
	ori_z = odometry_msg.pose.pose.orientation.z
	ori_w = odometry_msg.pose.pose.orientation.w

	#quaternion = odometry_msg.pose.pose.orientation
	quaternion = (ori_x, ori_y, ori_z, ori_w)


	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll  = degrees(euler[0])
	pitch = degrees(euler[1])
	yaw   =  degrees(euler[2])
	
	roll = to_positive(roll)
	pitch = to_positive(pitch)
	yaw = to_positive(yaw)

	#dt = odometry_msg.header.seq
	
	vel_x= (odom_position_x-old_pos_x[-1])/dt
	vel_y= (odom_position_y-old_pos_y[-1])/dt

	ang_vel_pitch = (pitch-old_pitch[-1])/dt
	ang_vel_yaw = (yaw-old_yaw[-1])/dt
		

	old_pos_x.append(odom_position_x)
	old_pos_y.append(odom_position_y)
	old_pitch.append(pitch)
	old_yaw.append(yaw)
	
	pdb.set_trace()
	
	
rospy.init_node('topic_subscriber')
sub = rospy.Subscriber('SVEA5/Odom', Odometry, callback)

rospy.spin()
