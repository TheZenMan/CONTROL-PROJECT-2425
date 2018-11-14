#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

rospy.init_node('lidar_scan_node')


pub = rospy.Publisher('lidar_scan',LaserScan,queue_size=1)
rate = rospy.Rate(50)


scan = LaserScan()
scan.ranges = [0.1,0.2,0.3]
while not rospy.is_shutdown():
    pub.publish(scan)
    rate.sleep()


