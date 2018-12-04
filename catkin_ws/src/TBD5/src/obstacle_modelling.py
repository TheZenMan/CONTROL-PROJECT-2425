#!/usr/bin/env python

import sys
import os
import rospy
import numpy as np
from math import pi, sin, cos, sqrt, atan2, radians, degrees
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import ConvexHull

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped

rospy.init_node('obstacles_modeling') # Initiate a Node named 'obstacle_modeling'
obstacle_pub = rospy.Publisher('obstacles', LaserScan, queue_size=1)


curr_scan = None
dynamic_lookahead = 3.5

ranges=[]
x_list=[]#
y_list=[]
d_list=[]

x1_list=[]#first compariosn lists
y1_list=[]
d1_list=[]

x2_list=[]# second time step comparison lists
y2_list=[]
d2_list=[]

t=0

def points(dynamic_indices, x_list, y_list):
    global obstacle_list

    obstacle_list =[]
    for i in dynamic_indices:
        obs_x_list.append(x_list[i])
        obs_y_list.append(y_list[i])
    
    for i in range(len(obs_x_list)):
        p_x1[i] = obs_x_list[i]-0.22
	p_y1[i] = obs_y_list[i]+0.47
	p_x2[i] = obs_x_list[i]+0.22
	p_y2[i] = obs_y_list[i]+0.47
	p_x3[i] = obs_x_list[i]+0.22
	p_y3[i] = obs_y_list[i]-0.47
	p_x4[i] = obs_x_list[i]-0.22
	p_y4[i] = obs_y_list[i]-0.47


   return p_x1, p_y1, p_x2, p_y2, p_x3, p_y3, p_x4, p_y4

def obstacle (pts):

	hull = ConvexHull(pts)

	fig = plt.figure()
	ax = fig.add_subplot(111, projection="3d")

	# Plot defining corner points
	ax.plot(pts.T[0], pts.T[1], pts.T[2], "ko")

	# 12 = 2 * 6 faces are the simplices (2 simplices per square face)
	for s in hull.simplices:
    	    s = np.append(s, s[0])  # Here we cycle back to the first coordinate
    	    ax.plot(pts[s, 0], pts[s, 1], pts[s, 2], "r-")

	# Make axis label
	for i in ["x", "y", "z"]:
    	    [obstacles][1]eval("ax.set_{:s}label('{:s}')".format(i, i))

	plt.show()

def compare(x_1,y_1,d_1,x_2,y_2,d_2):
    global curr_scan
    global dynamic_indices

    walking = False
    obstacle_counter=0
    x_vel_list=[]
    y_vel_list = []

    dynamic_indices = []
    for i in range(len(x_1)):
        # if abs(d_1[i]-d_2[i]) > 0.2: #use x position diff
        # if abs(d_1[i]-d_2[i]) > 0.2: #use x position diff
        if ((abs(x_1[i]-x_2[i]) > 0.1 and
             abs(x_1[i]-x_2[i]) < 0.3)  or
            (abs(y_1[i]-y_2[i]) > 0.1 and
             abs(y_1[i]-y_2[i]) < 0.3)): #use y position diff
            #print ('walking person x')
            #x.append(x_1[i])
            x_currvel=(x_1[i]-x_2[i])/0.8
            y_currvel =(y_1[i]-y_2[i])/0.8
            x_vel_list.append(x_currvel)
            y_vel_list.append(y_currvel)

            #y.append(y_1[i])
            #x_n.append(x_2[i])
            #y_n.append(y_2[i])
            #walking = True #return true if somone is moving
            obstacle_counter = obstacle_counter + 1
            dynamic_indices.append(i)
        #if abs(y_1[i]-y_2[i]) > 0.3: #use y position diff
            #print ('walking person y')
            #obstacle_counter=obstacle_counter+1
            #x.append(x_1[i])
            #y.append(y_1[i])
            #x_n.append(x_2[i])
            #y_n.append(y_2[i])
            #walking = True #return true if someone is moving
    x_velocity = 0
    y_velocity = 0
    if obstacle_counter>10:
        walking = True
        x_velocity = sum(x_vel_list)/len(x_vel_list)
        y_veloctiy = sum(y_vel_list)/len(y_vel_list)
        #print('walking person counter')

        # publish moving obstacles
        # dynamic_scan = curr_scan
        dynamic_scan = LaserScan()
        dynamic_scan.header.stamp = rospy.Time.now()
        dynamic_scan.header.frame_id = "SVEA5"
        dynamic_scan.angle_min = curr_scan.angle_min
        dynamic_scan.angle_max = curr_scan.angle_max
        dynamic_scan.angle_increment = curr_scan.angle_increment
        dynamic_scan.scan_time = curr_scan.scan_time
        dynamic_scan.range_min = curr_scan.range_min
        dynamic_scan.range_max = curr_scan.range_max
        # dynamic_ranges_list = list(dynamic_scan.ranges)
        dynamic_ranges_list = list(curr_scan.ranges)
        for i in range(len(curr_scan.ranges)):
            if not i in dynamic_indices or curr_scan.ranges[i] > dynamic_lookahead:
            # if i in dynamic_indices:
                dynamic_ranges_list[i] = float("inf")
                # dynamic_ranges_list.append(curr_scan.ranges[i])

        # dynamic_scan = LaserScan()
        dynamic_scan.ranges = tuple(dynamic_ranges_list)
        dynamic_scan_pub.publish(dynamic_scan)

    return walking, x_velocity, y_velocity


def callback_mocap(odometry_msg):
    global curr_scan

    global ranges
    global x_list
    global y_list
    global d_list
    global t
    global angle_min
    global increment
    global x1_list
    global y1_list
    global d1_list
    global x2_list
    global y2_list
    global d2_list

    # if not len(ranges) == 0:
    if not curr_scan is None:
        x_pos = odometry_msg.pose.pose.position.x
        y_pos = odometry_msg.pose.pose.position.y
        robot_yaw = odometry_msg.pose.pose.orientation.z
        x_list = []
        y_list = []
        d_list = []

        x2_list = []
        y2_list = []
        d2_list = []
        for i in range(len(curr_scan.ranges)):
            o_distance = curr_scan.ranges[i] # ranges distance
            o_angle = curr_scan.angle_min + i * curr_scan.angle_increment + robot_yaw #angle to object (check if this works)
            x_o = np.cos(o_angle) * o_distance #body coordinates of object
            y_o = np.sin(o_angle) * o_distance
            x_g = x_o + x_pos #global coordinates of object
            y_g = y_o +y_pos
            d = math.sqrt(math.pow(x_g, 2) + math.pow(y_g, 2)) #distance to object in global frame
            x_list.append(x_g)
            y_list.append(y_g)
            d_list.append(d)

        if t<0.05: #want first lists near start of time
            x1_list = x_list  # first comparison lists
            y1_list = y_list
            d1_list = d_list

        if t%0.1 < 0.01: #refresh x2, y2 and d2 every 0.5 seconds
            x2_list = x_list
            y2_list = y_list
            d2_list = d_list
        #print(len(x2_list))
        if not len(x2_list) == 0:
            walking, x_velocity, y_velocity = compare(x1_list, y1_list, d1_list, x2_list, y2_list, d2_list)
            x1_list = x2_list  # Refresh lists so we always compare every 0.5 seconds
            y1_list = y2_list
            d1_list = d2_list
            if walking ==True:
                print('Something is moving')
                print('x velocity',x_velocity)
                print('y velocity', y_velocity)

def callback_lidar(scan):
    # global ranges
    # global angle_min
    # global increment
    global t
    global curr_scan
    # global scan_time

    # ranges = scan.ranges
    # angle_min = scan.angle_min
    # increment = scan.angle_increment
    scan_time = scan.scan_time
    t += scan_time

    curr_scan = scan

p_x1, p_y1, p_x2, p_y2, p_x3, p_y3, p_x4, p_y4 = points(dynamic_indices, x_list, y_list)

pts = np.array([[p_x1, p_y1, 0], [p_x2, p_y2, 0], [p_x3, p_y3, 0], [p_x4, p_y4, 0],
                [p_x1, p_y1, 1], [p_x2, p_y2, 1], [p_x3, p_y3, 1], [p_x4, p_y4, 1]])

obstacle(pts)
            
def main():
    mocap_sub = rospy.Subscriber('odometry_body_frame', Odometry, callback_mocap)
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback_lidar)
    rospy.spin()

if __name__ == '__main__':
    main()

