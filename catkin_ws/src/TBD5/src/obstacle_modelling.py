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


x_list=[]
y_list=[]
p_x1 =0
p_y1 =0
p_x2 =0
p_y2 =0
p_x3 =0
p_y3 =0
p_x4 =0
p_y4 =0

def points(x_list, y_list):
    
    for i in range(len(x_list)):
        p_x1 = x_list[i]-0.22
	p_y1 = y_list[i]+0.47
	p_x2 = x_list[i]+0.22
	p_y2 = y_list[i]+0.47
	p_x3 = x_list[i]+0.22
	p_y3 = y_list[i]-0.47
	p_x4 = x_list[i]-0.22
	p_y4 = y_list[i]-0.47


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


def callback_dyn(dyn_msg):
	global x_obs_list
	global y_obs_list
	 
	dyn_msg.poses.position.x = x_obs
	dyn_msg.poses.position.y = y_obs
	x_obs_list.append(x_obs)
	y_obs_list.append(y_obs)

p_x1, p_y1, p_x2, p_y2, p_x3, p_y3, p_x4, p_y4 = points(x_obs, y_obs)

pts = np.array([[p_x1, p_y1, 0], [p_x2, p_y2, 0], [p_x3, p_y3, 0], [p_x4, p_y4, 0],
                [p_x1, p_y1, 1], [p_x2, p_y2, 1], [p_x3, p_y3, 1], [p_x4, p_y4, 1]])

obstacle(pts)
            
def main():
    dynamic_sub = rospy.Subscriber('dynamic_traj', PoseArray, callback_dyn)
    rospy.spin()

if __name__ == '__main__':
    main()

