#! /usr/bin/env python

import rospy
from math import degrees, pi

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray

rospy.init_node('laser_information')
pub = rospy.Publisher('angle_distance_obstacles', PoseArray, queue_size=1)

#yaw_list =[]
#def callback_mocap(odometry_msg):
    #global yaw_list
    #yaw_list.append(odometry_msg.pose.pose.orientation.z)

def callback_laserscan(scan):
    print('laserscan')
    #if not len(yaw_list)==0:
	#distance_list = []
     angle_list = []

     for i in range(len(scan.ranges)):
           #robot_yaw = yaw_list[-1]
           angle = scan.angle_min + i * scan.angle_increment
	   # distance_list.append(scan.ranges[i])
	   angle_list.append(angle)

	#for i in range(len(distance_list)):
	    laserscan_data = Pose()
	    laserscan_data.position.x = scan.ranges[i]
	    laserscan_data.position.y = angle_list

	    pub.publish(laserscan_data)


#sub = rospy.Subscriber('/odometry_body_frame', Odometry, callback_mocap)
sub = rospy.Subscriber('/scan', LaserScan, callback_laserscan)

rospy.spin()
