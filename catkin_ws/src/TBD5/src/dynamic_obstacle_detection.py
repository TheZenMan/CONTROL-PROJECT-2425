#! /usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

rospy.init_node('dynamic_obstacle_detection')

x_list=[]
y_list=[]
d_list=[]


def callback_mocap(odometry_msg):
    global ranges
    global x_list
    global y_list
    global d_list

    if not len(ranges) == 0:
	x_pos = odometry_msg.pose.pose.position.x
        y_pos = odometry_msg.pose.pose.position.y
	for i in range(len(ranges)):
		o_distance = ranges[i]
		o_angle = angle_min + i * increment
		x_o = np.cos(o_angle) * o_distance
		y_o = np.sin(o_angle) * o_distance
		x_g = x_o + x_pos
		y_g = y_o +y_pos
		d = math.sqrt(math.pow(x_g, 2) + math.pow(y_g, 2))
		x_list.append(x_g)
		y_list.append(y_g)
    		d_list.append(d)


def callback_lidar(scan):

    global ranges
    global angle_min
    global increment
    global t
    t=0
    ranges = scan.ranges
    angle_min = scan.angle_min
    increment = scan.angle_increment
    scan_time = scan.scan_time
    t += scan_time
    x,y,x_n,y_n = obstacle_detection(t,x_list,y_list,d_list)

def obstacle_detection(t,x_list,y_list,d_list):
    global scan_time
    x = []
    y = []
    x_n = []
    y_n = []
    if t == scan_time:
	x_1 = x_list
	y_1 = y_list
	d_1 = d_list
    if t == 1000* scan_time:
	x_2 = x_list
	y_2 = y_list
        d_2 = d_list
    for i in range(len(d_1)):
	if  abs(d_1[i] - d_2[i]) > 0.01:
		x.append(x_1[i])
		y.append(y_1[i])
		x_n.append(x_2[i])
		y_n.append(y_2[i])
	return x,y,x_n,y_n



def main():
    t = 0
    mocap_sub = rospy.Subscriber('odometry_body_frame', Odometry, callback_mocap)
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback_lidar)
    rospy.spin()

if __name__ == '__main__':
    main()

