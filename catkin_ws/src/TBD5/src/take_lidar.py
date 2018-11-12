#!/usr/bin/env python

import rospy
import math
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from  low_level_interface.msg import lli_ctrl_request


rospy.init_node('lidar_process_node')
pub= rospy.Publisher('/lli/ctrl_request',lli_ctrl_request,queue_size=1)

L = 0.32  # [m] wheel base of vehicle change according to our car --> length of the car

def callback_lidar(scan):
    if not len(pos_x) == 0: #both subscribers dont start same time
        for i in range(len(scan.ranges)):
            robot_yaw = yaw_list[-1]
            distance = scan.ranges[i]
            angle = scan.angle_min + robot_yaw + i * scan.angle_increment

            if distance > scan.range_min and distance < scan.range_max: # dont accept scans that are outside ranges
                # print(scan.range_max)
                # print(scan.range_min)

                x_o = np.cos(angle) * distance +L/2
                y_o = np.sin(angle) * distance


                x_robot = pos_x[-1]
                y_robot = pos_y[-1]
                dx = x_o - x_robot
                dy = y_o - y_robot
                dist = math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))
                if dist<1:
                    target_speed = 0 # speed 0 if too close to obstacle
                    control_request = lli_ctrl_request()
                    control_request.velocity = target_speed #put this in a controller node
                    pub.publish(control_request) #publish to control request, but only if near an obstacle
                #x_laser = x_o + x_robot
                #y_laser = y_o + y_robot #mot needed?


pos_x = []
pos_y = []
yaw_list =[]
def callback_mocap(odometry_msg):
    global pos_x
    global pos_y
    global yaw_list

    pos_x, pos_y, yaw_list = [], [], [] #correct way to write this?
    pos_x.append(odometry_msg.pose.pose.position.x)
    pos_y.append(odometry_msg.pose.pose.position.y)
    yaw_list.append(odometry_msg.pose.pose.orientation.z)


def main():


    lidar_sub = rospy.Subscriber('/lidar_scan' + '/SVEA5', LaserScan, callback_lidar) #correct topic for lidar?
    mocap_sub = rospy.Subscriber('odometry_body_frame', Odometry, callback_mocap)

    rospy.spin()

if __name__ == '__main__':
    main()
#