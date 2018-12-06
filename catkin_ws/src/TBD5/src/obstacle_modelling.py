#!/usr/bin/env python

import sys
import os
import rospy
import numpy as np
from math import pi, sin, cos, sqrt, atan2, radians, degrees

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, Polygon

rospy.init_node('obstacles_modeling') # Initiate a Node named 'obstacle_modeling'
obstacle_pub = rospy.Publisher('obstacles', Polygon, queue_size=1)

x_obs_list = []
y_obs_list = []
x_list=[]
y_list=[]


def points(x_list, y_list):
    p_x1 =0
    p_y1 =0
    p_x2 =0
    p_y2 =0
    p_x3 =0
    p_y3 =0
    p_x4 =0
    p_y4 =0


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



def callback_dyn(dyn_msg):
	global x_obs_list
	global y_obs_list

	dyn_msg.poses.position.x = x_obs
	dyn_msg.poses.position.y = y_obs
	x_obs_list.append(x_obs)
	y_obs_list.append(y_obs)
	obstacles = Polygon()
        dynamic_traj.header.stamp = rospy.Time.now()
        dynamic_traj.header.frame_id ='qualisys'
        
	p_x1, p_y1 p_x2, p_x3, p_x4 = points(x_obs_list, y_obs_list)

	
	for i in range(len(x_obs_list):
	    	obstacle = Polygon()
	    	obstacle.polygon.points = [Point32(x= p_x1, y= p_y1, z= 0),
					   Point32(x= p_x2, y= p_y2, z= 0),
					   Point32(x= p_x3, y= p_y3, z= 0),
					   Point32(x= p_x4, y= p_y4, z= 0),
					   Point32(x= p_x1, y= p_y1, z= 1),
					   Point32(x= p_x2, y= p_y2, z= 1),
			     		   Point32(x= p_x3, y= p_y3, z= 1),
				           Point32(x= p_x4, y= p_y4, z= 1)]
	    	
	    	
	obstacle_pub.publish(obstacle)


def main():
    dynamic_sub = rospy.Subscriber('dynamic_traj', PoseArray, callback_dyn)
    rospy.spin()

if __name__ == '__main__':
    main()

