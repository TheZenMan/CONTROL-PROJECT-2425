#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

rospy.init_node('lidar_scan_node')


pub = rospy.Publisher('lidar_scan',LaserScan,queue_size=1)
rate = rospy.Rate(50)


scans = LaserScan()
scans.ranges = [2,2,4,5,6,7,66]

while not rospy.is_shutdown():
    pub.publish(scans)
    rate.sleep()