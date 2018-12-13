#!/usr/bin/env python

import rospy
import numpy as np
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray

rospy.init_node ('dynamic_trajectory')
dynamic_traj_pub = rospy.Publisher('dynamic_traj',PoseArray,queue_size =1)
x_traj=[]
y_traj=[]

curr_scan = None
x_list=[]
y_list=[]
t=0
ranges=[]
x_pos=0
y_pos=0
x1_pos=0
y1_pos=0
x2_pos=0
y2_pos=0
vel_x=0

#v_list=[0]

def callback_mocap(odometry_msg):  #information from the Mocap
    global curr_scan
    global t
    global x_pos
    global y_pos
    global x1_pos
    global y1_pos
    global x2_pos
    global y2_pos
    global vel_x


    if not curr_scan is None:
        x_pos_o = odometry_msg.pose.pose.position.x
        y_pos_o = odometry_msg.pose.pose.position.y
        robot_yaw = odometry_msg.pose.pose.orientation.z
        x_list = []
        y_list = []
        x_pos=0
        y_pos=0
        x2_pos=0
        y2_pos=0
        vel_x=0




        for i in range(len(curr_scan.ranges)):
            if not curr_scan.ranges[i] == float("inf"):
                o_distance = curr_scan.ranges[i] #ranges distance
                o_angle = curr_scan.angle_min + i * curr_scan.angle_increment + robot_yaw #angle to object
                x_o = np.cos(o_angle) * o_distance #body coordinates of object
                y_o = np.sin(o_angle) * o_distance
                x_g = x_o + x_pos_o #global coordinates of object
                y_g = y_o + y_pos_o

                x_list.append(x_g)
                y_list.append(y_g)

        if not len(x_list)==0:
            x_pos,y_pos = averagenum(x_list,y_list)  #average of the obstacles positions
            x_traj.append(x_pos)   #creation of a trajectory on x with the average of the x position of the obstacle
            y_traj.append(y_pos)   #creation of a trajectory on y with the average of the x position of the obstacle

        if t<0.005:#want first lists near start of time
            x1_pos = x_pos
            y1_pos = y_pos
        if t%0.01 < 0.001:
            x2_pos = x_pos
            y2_pos = y_pos
            vel_x = comvelocity(x1_pos,y1_pos,x2_pos,y2_pos)
#       if not x2_pos == 0:
           # vel_x,vel_y = comvelocity(x1_pos,y1_pos,x2_pos,y2_pos)
            x1_pos = x2_pos
            y1_pos = y2_pos
           # v_list.append(vel_x)
            #print(v_list, len(v_list))
            if abs(vel_x)>0.1:
                print('vel_x',vel_x)

        dynamic_traj = PoseArray()
        dynamic_traj.header.stamp = rospy.Time.now()
        dynamic_traj.header.frame_id ='qualisys'

        for i in range(len(x_traj)):
            pose = Pose()
            pose.position.x = x_traj[i]
            pose.position.y = y_traj[i]
            pose.orientation.z = vel_x
            pose.orientation.x = x_pos
            pose.orientation.y = y_pos
            #pose.position.z = v_list[i]
            dynamic_traj.poses.append(pose)
        dynamic_traj_pub.publish(dynamic_traj)  #publisher to publish the information of the trajectories created

old_velocity1 =0
old_velocity2 =0

def comvelocity (x1,y1,x2,y2):
    global old_velocity1
    global old_velocity2
    v_x=0
    v_y=0
    v_x= (x2-x1)/0.05
    v_y= (y2-y1)/0.05

    avg_velocity = (v_x + old_velocity1 + old_velocity2)/3

    old_velocity1 = v_x
    old_velocity2 = old_velocity1
   # print('x_1',x1)
    #print('x_2',x2)
    #print('y_1',y1)
    #print('y_2',y2)

    return avg_velocity


def averagenum (x_1,y_1): #compute the average value of the position
    nsum_x = 0
    nsum_y = 0
    average_x = 0
    average_y = 0
    for i in range(len(x_1)):
        nsum_x += x_1[i]
    for j in range(len(y_1)):
        nsum_y += y_1[i]
    average_x =nsum_x / len(x_1)
    average_y =nsum_y / len(y_1)
    # print('x_pos_aver',average_x)
    return average_x, average_y


def callback_lidar(scan):  #take information from the lidar
    global t
    global curr_scan

    scan_time = scan.scan_time
    t += scan_time
    curr_scan = scan


def main():
    d_lidar_sub = rospy.Subscriber('dynamic_scan', LaserScan, callback_lidar)
    mocap_sub = rospy.Subscriber('odometry_body_frame', Odometry, callback_mocap)
    rospy.spin()

if __name__ == '__main__':
    main()
