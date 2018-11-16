#! /usr/bin/env python

import rospy
from math import degrees, pi

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs import PoseArray

rospy.init_node('laser_information')
pub = rospy.Publisher('angle_distance_obstacles', PoseArray, queue_size=1)

def callback_laserscan(scan):
    if not len(yaw_list)==0:
	distance_list = []
	angle_list = []

        for i in range(len(scan.ranges)):
            robot_yaw = yaw_list[-1]
            angle = scan.angle_min + robot_yaw + i * scan.angle_increment
	    distance_list.append(scan.ranges[i])
	    angle_list.append(angle)

	for i in range(len(distance_list)):
		laserscan_data = Pose()
		laserscan_data.position.x = distance_list[i] 
		laserscan_data.position.y = angle_list[i] 
	
		pub.publish(laserscan_data)

yaw_list =[]
def callback_mocap(odometry_msg):
    global yaw_list
    yaw_list.append(odometry_msg.pose.pose.orientation.z)


sub = rospy.Subscriber('/odometry_body_frame', Odometry, callback_mocap)
sub = rospy.Subscriber('/scan', LaserScan, callback_laserscan)

rospy.spin()
