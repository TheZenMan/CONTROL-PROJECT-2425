#! /usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

rospy.init_node('dynamic_obstacle_detection')
ranges=[]
x_list=[]
y_list=[]
d_list=[]
t=0
x = []
y = []
x_n = []
y_n = []

def callback_mocap(odometry_msg):
    global ranges
    global x_list
    global y_list
    global d_list


    if not len(ranges) == 0:
	x_pos = odometry_msg.pose.pose.position.x
        y_pos = odometry_msg.pose.pose.position.y
       # x_list=[]
        #y_list=[]
        #d_list=[]

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
               # print (len(d_1))
       # x,y,x_n,y_n = obstacle_detection(t,x_list,y_list,d_list)

def callback_lidar(scan):

    global ranges
    global angle_min
    global increment
    global t
    global scan_time

    ranges = scan.ranges
    angle_min = scan.angle_min
    increment = scan.angle_increment
    scan_time = scan.scan_time
    #print(len(ranges))
    t += scan_time
    obstacle_detection(t,x_list,y_list,d_list)
  #  t += scan_time*len(ranges)
    if t > 0.18:
        compare(x_1,y_1,d_1,x_2,y_2,d_2)

def obstacle_detection(t,x_list,y_list,d_list):

    global x_1
    global y_1
    global d_1
    global x_2
    global y_2
    global d_2
    print(t)
    if 0.05 < t < 0.075:
	x_1 = x_list
	y_1 = y_list
	d_1 = d_list
    #print("len",len(d_1))
    if 0.15 < t < 0.18:
	x_2 = x_list
	y_2 = y_list
        d_2 = d_list
    #print(d_2)
     #for i in range(len(d_1)):
#	if  abs(d_1[i] - d_2[i]) > 0.01:
#		x.append(x_1[i])
#		y.append(y_1[i])
#		x_n.append(x_2[i])
#		y_n.append(y_2[i])
    if t>0.18:
        print('Done')


def compare(x_1,y_1,d_1,x_2,y_2,d_2):
    #print ("len=",len(d_1))
    #print ("len1=",len(d_2))
    for i in range(len(x_1)):
        #print('d1',d_1[i])
        #print('d2',d_2[i])
        if abs(d_1[i]-d_2[i]) > 0.01:
            print ('work')
            x.append(x_1[i])
            y.append(y_1[i])
            x_n.append(x_2[i])
            y_n.append(y_2[i])
        #else:
         #   print ('no moving obstacles')
    print(x,y,x_n,y_n)
    return x,y,x_n,y_n



def main():
    mocap_sub = rospy.Subscriber('odometry_body_frame', Odometry, callback_mocap)
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback_lidar)
    rospy.spin()

if __name__ == '__main__':
    main()

