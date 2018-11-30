#! /usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import pdb

rospy.init_node('dynamic_obstacle_detection')
dynamic_scan_pub = rospy.Publisher('dynamic_scan', LaserScan, queue_size=1)

resolution = 0.50

curr_scan=[]
ranges_global=[]
ranges_global_discrete=[]
old_ranges_global_discrete=[]

x_list=[]
y_list=[]
d_list=[]
t=0
x = []
y = []
x_n = []
y_n = []

def callback_mocap(odometry_msg):
    # global ranges
    # global x_list
    # global y_list
    # global d_list


    # if not len(ranges) == 0:
        # x_pos = odometry_msg.pose.pose.position.x
        # y_pos = odometry_msg.pose.pose.position.y
       # # x_list=[]
        # #y_list=[]
        # #d_list=[]

        # for i in range(len(ranges)):
                # o_distance = ranges[i]
                # o_angle = angle_min + i * increment
                # x_o = np.cos(o_angle) * o_distance
                # y_o = np.sin(o_angle) * o_distance
                # #TODO: rotate to yaw
                # x_g = x_o + x_pos
                # y_g = y_o + y_pos
                # #TODO: check changes on x y coordinates
                # d = math.sqrt(math.pow(x_g, 2) + math.pow(y_g, 2))
                # x_list.append(x_g)
                # y_list.append(y_g)
                    # d_list.append(d)
               # print (len(d_1))
       # x,y,x_n,y_n = obstacle_detection(t,x_list,y_list,d_list)

    global curr_scan
    global ranges_global
    global ranges_global_discrete
    global old_ranges_global_discrete

    car_x = odometry_msg.pose.pose.position.x
    car_y = odometry_msg.pose.pose.position.y
    car_yaw = odometry_msg.pose.pose.orientation.z

    def scan_to_rel_poses(scan):
        poses = []

        #unpack LaserScan msg
        angle_min, angle_max = scan.angle_min, scan.angle_max
        angle_increment = scan.angle_increment
        ranges = scan.ranges

        curr_angle = angle_min
        for dist in ranges:
            # if dist == float("inf"):
            if dist > scan.range_max:
                dist = scan.range_max
            if dist < scan.range_min:
                dist = scan.range_min
            if dist < scan.range_min or dist > scan.range_max:
                print(dist)
            rel_x, rel_y = dist * math.cos(curr_angle), dist * math.sin(curr_angle)
            poses.append([rel_x, rel_y])
            curr_angle += angle_increment
        return np.array(poses)

    def rotate(origin, pt, angle):
        origin_x, origin_y = origin[0], origin[1]
        pt_x, pt_y = pt[0], pt[1]

        dist_x = pt_x - origin_x
        dist_y = pt_y - origin_y

        rot_x = origin_x + math.cos(angle) * (dist_x) - \
                           math.sin(angle) * (dist_y)
        rot_y = origin_y + math.sin(angle) * (dist_x) + \
                           math.cos(angle) * (dist_y)
        return [rot_x, rot_y]

    # convert scans to body cartesian coordinates
    scan_rel_poses = scan_to_rel_poses(curr_scan)

    # rotate relative positions by car's heading (yaw)
    scan_poses = np.array([rotate([0, 0], rel_pose, car_yaw)
                           for rel_pose in scan_rel_poses])
    # translate to robot position within coordinate frame
    scan_poses[:, 0] += car_x * np.ones(scan_poses.shape[0])
    scan_poses[:, 1] += car_y * np.ones(scan_poses.shape[0])

    ranges_global = scan_poses

    ranges_global_discrete = scan_poses / resolution
    for i in range(ranges_global_discrete.shape[0]):
        for j in range(ranges_global_discrete.shape[1]):
            # try:
            # ranges_global_discrete[i, j] = int(ranges_global_discrete[i,j])
            ranges_global_discrete[i, j] = math.floor(ranges_global_discrete[i,j])

    # compare old with new ranges_global_discrete
    #if old_ranges_global_discrete not [0,0]:
    # print(len(ranges_global_discrete))
    if not len(old_ranges_global_discrete) == 0:

        diff = ranges_global_discrete - old_ranges_global_discrete
        nonzero_indices = []
        for i in range(diff.shape[0]):
            if not diff[i, 0] == 0.0 or not diff[i, 1] == 0.0:
                nonzero_indices.append(i)

        dynamic_scan = curr_scan
        # dynamic_distance = curr_scan.ranges
        dynamic_ranges_list = list(dynamic_scan.ranges)
        for i in range(len(dynamic_ranges_list)):
            if not i in nonzero_indices:
                dynamic_ranges_list[i] = float("inf")

        # dynamic_scan = LaserScan()
        dynamic_scan.ranges = tuple(dynamic_ranges_list)
        dynamic_scan_pub.publish(dynamic_scan)
    #else:
     #   old_ranges_global_discrete = ranges_global_discrete

    #old_ranges_global_discrete = list(old_ranges_global_discrete)
    old_ranges_global_discrete = ranges_global_discrete #.append([0,0])
    #old_ranges_global_discrete = tuple(old_ranges_global_discrete)

def callback_lidar(scan):

    # global ranges
    # global angle_min
    # global increment
    # global t
    # global scan_time

    # ranges = scan.ranges
    # angle_min = scan.angle_min
    # increment = scan.angle_increment
    # scan_time = scan.scan_time
    # #print(len(ranges))
    # t += scan_time
    # obstacle_detection(t,x_list,y_list,d_list)
  # #  t += scan_time*len(ranges)
    # if t > 0.18:
        # compare(x_1,y_1,d_1,x_2,y_2,d_2)

    global curr_scan

    curr_scan = scan



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
#       if  abs(d_1[i] - d_2[i]) > 0.01:
#               x.append(x_1[i])
#               y.append(y_1[i])
#               x_n.append(x_2[i])
#               y_n.append(y_2[i])
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
    rate = rospy.Rate(10) # 39 [Hz]
    mocap_sub = rospy.Subscriber('odometry_body_frame', Odometry, callback_mocap)
    lidar_sub = rospy.Subscriber('scan', LaserScan, callback_lidar)
    rospy.spin()

if __name__ == '__main__':
    main()

