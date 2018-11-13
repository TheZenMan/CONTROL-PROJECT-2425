#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

rospy.init_node('lidar_scan_node')


pub = rospy.Publisher('lidar_scan',LaserScan,queue_size=1)
rate = rospy.Rate(50)
#time=np.linspace(0,10,100)

scan = LaserScan()
scan.ranges = [2,3,4,5,6]
#count=0
#for i in range(len(time)):
 #   for j in range(10):
  #      count+=0.4
   #     scan.ranges[j]=count
    #    print(count)

while not rospy.is_shutdown():
    pub.publish(scan)
    rate.sleep()
