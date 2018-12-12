#!/usr/bin/env python

import sys
import os
import rospy
import numpy as np
from math import pi, sin, cos, sqrt, atan2, radians, degrees

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, PolygonStamped, Point32

rospy.init_node('obstacles_modeling') # Initiate a Node named 'obstacle_modeling'
obstacle_pub = rospy.Publisher('obs_model_dyn', PolygonStamped, queue_size=1)

x_obs=0
y_obs=0
obs_vel_x = 0


def points(x_obs, y_obs, obs_vel_x):
    p_x1 =0
    p_y1 =0
    p_x2 =0
    p_y2 =0
    p_x3 =0
    p_y3 =0
    p_x4 =0
    p_y4 =0

    if 2>obs_vel_x > 0.5:
        p_y1 = y_obs-0.22
	p_x1 = x_obs+1
	p_y2 = y_obs+0.22
	p_x2 = x_obs+1
	p_y3 = y_obs+0.22
	p_x3 = x_obs
	p_y4 = y_obs-0.22
	p_x4 = x_obs

    elif -2 < obs_vel_x <-0.5:

        p_y1 = y_obs-0.22
	p_x1 = x_obs
	p_y2 = y_obs+0.22
	p_x2 = x_obs
	p_y3 = y_obs+0.22
	p_x3 = x_obs-1
	p_y4 = y_obs-0.22
	p_x4 = x_obs-1

    elif -0.5<obs_vel_x<0.5:
        p_y1 = y_obs-0.22
	p_x1 = x_obs+0.5
	p_y2 = y_obs+0.22
	p_x2 = x_obs+0.5
        p_y3 = y_obs+0.22
	p_x3 = x_obs-0.5
	p_y4 = y_obs-0.22
	p_x4 = x_obs-0.5



    return p_x1, p_y1, p_x2, p_y2, p_x3, p_y3, p_x4, p_y4




def callback_dyn(dyn_msg):
        global x_obs
        global y_obs
	global obs_vel_x
        x_obs=0
        y_obs=0
	obs_vel_x= 0
	position_poses = dyn_msg.poses
        for dyn_pt in position_poses:
            x_obs = dyn_pt.orientation.x
            y_obs = dyn_pt.orientation.y
	    obs_vel_x = dyn_pt.orientation.z


	p_x1, p_y1, p_x2, p_y2, p_x3, p_y3, p_x4, p_y4 = points(x_obs, y_obs, obs_vel_x)

        obs_model_dyn = PolygonStamped()
        obs_model_dyn.header.stamp = rospy.Time.now()
        obs_model_dyn.header.frame_id ='qualisys'

        if (-2.5 < obs_vel_x < 2.5) and (not obs_vel_x ==0) :
            obs_model_dyn.polygon.points = [Point32(x= p_x1, y= p_y1, z= 0),
					   Point32(x= p_x2, y= p_y2, z= 0),
					   Point32(x= p_x3, y= p_y3, z= 0),
					   Point32(x= p_x4, y= p_y4, z= 0),
                                           Point32(x= p_x1, y= p_y1, z= 0),
					   Point32(x= p_x1, y= p_y1, z= 0.5),
                                           Point32(x= p_x2, y= p_y2, z= 0.5),
                                           Point32(x= p_x2, y= p_y2, z= 0),
                                           Point32(x= p_x2, y= p_y2, z= 0.5),
                                           Point32(x= p_x3, y= p_y3, z= 0.5),
			     		   Point32(x= p_x3, y= p_y3, z= 0),
                                           Point32(x= p_x3, y= p_y3, z= 0.5),
                                           Point32(x= p_x4, y= p_y4, z= 0.5),
                                           Point32(x= p_x4, y= p_y4, z= 0),
				           Point32(x= p_x4, y= p_y4, z= 0.5),
                                           Point32(x= p_x1, y= p_y1, z= 0.5)]


	obstacle_pub.publish(obs_model_dyn)

#def callback_obs_detection(detection_msg):


def main():
    dynamic_sub = rospy.Subscriber('dynamic_traj', PoseArray, callback_dyn)
    rospy.spin()

if __name__ == '__main__':
    main()

