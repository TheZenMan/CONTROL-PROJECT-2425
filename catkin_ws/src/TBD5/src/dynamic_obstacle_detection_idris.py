import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

rospy.init_node('people_detection')

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

def compare(x_1,y_1,d_1,x_2,y_2,d_2):
    walking = False
    for i in range(len(x_1)):
        if abs(x_1[i]-x_2[i]) > 0.1: #use x position diff
            print ('walking person')
            #x.append(x_1[i])
            #y.append(y_1[i])
            #x_n.append(x_2[i])
            #y_n.append(y_2[i])
            walking = True #return true if somone is moving
        elif abs(y_1[i]-y_2[i]) > 0.1: #use y position diff
            print ('walking person')
            #x.append(x_1[i])
            #y.append(y_1[i])
            #x_n.append(x_2[i])
            #y_n.append(y_2[i])
            walking = True #return true if someone is moving
    return walking


def callback_mocap(odometry_msg):
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

    if not len(ranges) == 0:
        x_pos = odometry_msg.pose.pose.position.x
        y_pos = odometry_msg.pose.pose.position.y
        robot_yaw = odometry_msg.pose.pose.orientation.z
        x_list = []
        y_list = []
        d_list = []

        x2_list = []
        y2_list = []
        d2_list = []
        for i in range(len(ranges)):
            o_distance = ranges[i] # ranges distance
            o_angle = angle_min + i * increment + robot_yaw #angle to object (check if this works)
            x_o = np.cos(o_angle) * o_distance #body coordinates of object
            y_o = np.sin(o_angle) * o_distance
            x_g = x_o + x_pos #global coordinates of object
            y_g = y_o +y_pos
            d = math.sqrt(math.pow(x_g, 2) + math.pow(y_g, 2)) #distance to object in global frame
            x_list.append(x_g)
            y_list.append(y_g)
            d_list.append(d)

        if t<0.1: #want first lists near start of time
            x1_list = x_list  # first comparison lists
            y1_list = y_list
            d1_list = d_list

        if t%0.5 <0.1: #refresh x2, y2 and d2 every 0.5 seconds
            x2_list = x_list
            y2_list = y_list
            d2_list = d_list

        if not len(x2_list) == 0:
            walking = compare(x1_list, y1_list, d1_list, x2_list, y2_list, d2_list)
            x1_list = x2_list  # Refresh lists so we always compare every 0.5 seconds
            y1_list = y2_list
            d1_list = d2_list
            print(walking)

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
    t += scan_time


def main():
    mocap_sub = rospy.Subscriber('odometry_body_frame', Odometry, callback_mocap)
    lidar_sub = rospy.Subscriber('/scan', LaserScan, callback_lidar)
    rospy.spin()

if __name__ == '__main__':
    main()