#! /usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

rospy.init_node('people_detection')
dynamic_scan_pub = rospy.Publisher('dynamic_scan', LaserScan, queue_size=1)

curr_scan = None
dynamic_lookahead = 1.5   #range for detecting obstacles
resolution = 1   #to perform space discretization

ranges=[]
x_list=[]
y_list=[]
d_list=[]

x1_list=[]#first comparison lists
y1_list=[]
d1_list=[]

x2_list=[]# second time step comparison lists
y2_list=[]
d2_list=[]

dynamic_scan2=[]


t=0

def compare(x_1,y_1,d_1,x_2,y_2,d_2):
    """
    Compares two subsequential lidar scan to understand if the obstacles
    detected have moved and creates a new LaserScan message containing information
    on the detected obstacles.
    """
    global curr_scan
    global dynamic_indices
    global dynamic_scan2

    walking = False
    obstacle_counter=0
    x_vel_list=[]
    y_vel_list = []
    dynamic_indices = []

    for i in range(len(x_1)):

        if ((abs(x_1[i]-x_2[i])/resolution > 0.01 and
             abs(x_1[i]-x_2[i])/resolution < 0.03)  or
            (abs(y_1[i]-y_2[i])/resolution > 0.01 and
             abs(y_1[i]-y_2[i])/resolution < 0.03)): #comparison between two differen positions
            
            x_currvel=(x_1[i]-x_2[i])/0.5   #velocity based on the rate of frequency of publishing of the Lidar
            y_currvel =(y_1[i]-y_2[i])/0.5
            x_vel_list.append(x_currvel)
            y_vel_list.append(y_currvel)

            obstacle_counter = obstacle_counter + 1   #obstacle counter
            dynamic_indices.append(i)    #list that contains all the indeces that correspond to moving obstacles
      
    print(dynamic_indices) 
 
    x_velocity = 0
    y_velocity = 0
    if obstacle_counter>10:
        walking = True
        x_velocity = sum(x_vel_list)/len(x_vel_list)
        y_veloctiy = sum(y_vel_list)/len(y_vel_list)
        
        dynamic_scan = LaserScan()    #if more than 10 obstacles are identified, a new laserscan message is created
        dynamic_scan.header.stamp = rospy.Time.now()
        dynamic_scan.header.frame_id = "SVEA5"
        dynamic_scan.angle_min = curr_scan.angle_min
        dynamic_scan.angle_max = curr_scan.angle_max
        dynamic_scan.angle_increment = curr_scan.angle_increment
        dynamic_scan.scan_time = curr_scan.scan_time
        dynamic_scan.range_min = curr_scan.range_min
        dynamic_scan.range_max = curr_scan.range_max
        dynamic_ranges_list = list(curr_scan.ranges)

        for i in range(len(curr_scan.ranges)):
            if not i in dynamic_indices or curr_scan.ranges[i] > dynamic_lookahead:  #distances of elements that are not recognized as moving obstacles are set to infinite
                dynamic_ranges_list[i] = float("inf")
               
       
        dynamic_scan.ranges = tuple(dynamic_ranges_list)
        dynamic_scan_pub.publish(dynamic_scan)
 

    return walking, x_velocity, y_velocity


def callback_mocap(odometry_msg):  #takes information from the mocap system
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
    global o_distance_list

    
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
        o_distance_list =[]

        for i in range(len(curr_scan.ranges)):
            o_distance = curr_scan.ranges[i] # ranges distance
            o_angle = curr_scan.angle_min + i * curr_scan.angle_increment + robot_yaw #angle to object (check if this works)
            x_o = np.cos(o_angle) * o_distance #body coordinates of object
            y_o = np.sin(o_angle) * o_distance
            x_g = x_o + x_pos #global coordinates of object
            y_g = y_o +y_pos
            d = math.sqrt(math.pow(x_g, 2) + math.pow(y_g, 2)) #distance to object in global frame
            o_distance_list.append(o_distance)
            x_list.append(x_g)
            y_list.append(y_g)
            d_list.append(d)

        if t<0.005: #want first lists near start of time
            x1_list = x_list  # first comparison lists
            y1_list = y_list
            d1_list = d_list

        if t%0.01 < 0.001: #refresh x2, y2 and d2 every 0.5 seconds
            x2_list = x_list
            y2_list = y_list
            d2_list = d_list
       
        if not len(x2_list) == 0:
            walking, x_velocity, y_velocity = compare(x1_list, y1_list, d1_list, x2_list, y2_list, d2_list)
            x1_list = x2_list  # Refresh lists so we always compare every 0.5 seconds
            y1_list = y2_list
            d1_list = d2_list
            #if walking ==True:
                #print('Something is moving')
                #print('x velocity',x_velocity)
                #print('y velocity', y_velocity)

def callback_lidar(scan):
  
    global t
    global curr_scan
    global scan_time
 
    scan_time = scan.scan_time
    t += scan_time

    curr_scan = scan


def main():
    mocap_sub = rospy.Subscriber('odometry_body_frame', Odometry, callback_mocap)
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback_lidar)
    rospy.spin()
 
if __name__ == '__main__':
    main()
